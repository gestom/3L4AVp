#!/usr/bin/env python
import rospy
import math
import os
import h5py
import sys
import socket
import Queue
import threading

import numpy as np
import tf as rostf
import tensorflow as tf

import pn_provider as provider
import pn_tf_util as tf_util
import pn_indoor3d_util as indoor3d_util
from pn_model import *

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from visualization_msgs import msg as msgTemplate
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

tfListener = None
legDetectorBuffers = []
legDetectorFrame = None
maxTimeSinceLaser = rospy.Duration(0, 250000000) #secs, nanosecs
maxDistanceToObj = 0.5
radarFlags = [True, True, True, True, True, True] #xyz, intensity, range, doppler
pointnetQueue = Queue.Queue()

def legDetectorCallback(msg):
	global legDetectorBuffers, legDetectorFrame
	legDetectorFrame = msg.header.frame_id
	if msg.ns != "PEOPLE":
		return
	while msg.id >= len(legDetectorBuffers):
		legDetectorBuffers.append(None)
	legDetectorBuffers[msg.id] = msg

def radarCallback(msg):
	global legDetectorBuffers, legDetectorFrame, tfListener, pointnetQueue

	if legDetectorFrame == None or len(legDetectorBuffers) == 0:
		print("Awaiting leg detector inputs...")
		return

	now = rospy.get_rostime()
	def filterOld(x):
		if x == None or (now - maxTimeSinceLaser > x.header.stamp):
			return False
		return True
	usedlegDetectorBufs = filter(filterOld, legDetectorBuffers)

	transform = None
	try:
		transform = tfListener.lookupTransform(msg.header.frame_id, legDetectorFrame, rospy.Time(0))
	except:
		print("Aborting, unable to get tf transformation")
		return

	trans_mat = rostf.transformations.translation_matrix(transform[0])
	rot_mat = rostf.transformations.quaternion_matrix(transform[1])
	mat = np.dot(trans_mat, rot_mat)

	points = []
	labels = []

	point_generator = pc2.read_points(msg)
	for point in point_generator:

		isObject = False

		transformedPoint = np.append(point[0:3], 1.0)
		transformedPoint = np.dot(mat, transformedPoint)
		point = np.append(transformedPoint[0:3], point[3:])

		now = rospy.get_rostime()

		for obj in legDetectorBuffers:
			if obj is None or now - maxTimeSinceLaser > obj.header.stamp:
				continue
			diffX = abs(obj.pose.position.x - point[0])
			diffY = abs(obj.pose.position.y - point[1])
			distance = ((diffX ** 2) + (diffY ** 2)) ** 0.5

			if distance < maxDistanceToObj:
				isObject = True

		x, y, z = point[0], point[1], point[2]
		r, g, b = point[3], point[4], point[5] #intensity, range, doppler
		xn, yn, zn = None, None, None

		point = [x, y, z, r, g, b, xn, yn, zn]
		points.append(np.array(point, dtype=np.float32))

		if isObject:
			labels.append(1)
		else:
			labels.append(0)

	#calculate normalised vals
	pointnetQueue.put((points, labels))

class PointnetThread(threading.Thread):
	def __init__(self, queue, publisher):
		threading.Thread.__init__(self)
		self.queue = queue
		self.publisher = publisher
		self.buffer = []
		self.maxBufferSize = 10
		self.frameNumber = 0
		self.trainEveryNFrames = 3

		self.threshold = 0.8
		self.numPoints = 30
		self.nClasses = 2

		self.graph = tf.Graph()
		self.device = tf.device("/gpu:0")

		with self.graph.as_default():
			with self.device:

				self.pointclouds_pl, self.labels_pl = placeholder_inputs(1, self.numPoints)
				self.is_training_pl = tf.placeholder(tf.bool, shape=())

				self.pred = get_model(self.pointclouds_pl, self.is_training_pl)
				self.loss = get_loss(self.pred, self.labels_pl)
				self.pred_softmax = tf.nn.softmax(self.pred)

				self.saver = tf.train.Saver()

			self.config = tf.ConfigProto()
			self.config.gpu_options.allow_growth = True
			self.config.allow_soft_placement = True
			self.config.log_device_placement = True

			self.ops = {'pointclouds_pl': self.pointclouds_pl,
				'labels_pl': self.labels_pl,
				'is_training_pl': self.is_training_pl,
				'pred': self.pred,
				'loss': self.loss,
				'train_op': self.train_op,
				'merged': self.merged,
				'step': self.batch}

	def run(self):
		while True:

			msg = None
			while True:
				msg = None
				try:
					msg = self.queue.get(block=False)
				except:
					break
			
				if msg == "quit":
					print("Closing net")
					return

				self.buffer.append(msg)

				if len(self.buffer) > self.maxBufferSize:
					del self.buffer[0]

			#train
			self.frameNumber += 1
			if self.frameNumber % self.trainEveryNFrames == 0 and 1==2:
				self.train()

			#infer
			results = self.infer(msg)

			#publish
			self.publish(results)

	def train(self):

		with tf.Session(graph=self.graph, config=self.config) as sess:

			# current_data, current_label, _ = provider.shuffle_data(msg[0][:, 0:self.num_points, :], msg[1]) 
			current_data, current_label = msg

			# feed_dict = {ops['pointclouds_pl']: current_data[0:1, :, :],
			feed_dict = {ops['pointclouds_pl']: current_data,
				ops['labels_pl']: current_label,
				ops['is_training_pl']: True}

			summary, step, _, loss_val, pred_val = sess.run([ops['merged'], ops['step'], ops['train_op'], ops['loss'], ops['pred']],
				feed_dict=feed_dict)

			print(summary, step, loss_val, pred_val)

	def infer(self, msg):

		feed_dict = {pointclouds_pl: inputPoints,
					 labels_pl: labels,
					 is_training_pl: False}
		lossf, predsf = sess.run([loss, pred_softmax], feed_dict=feed_dict)

	def publish(self, points):

			points = points[0]
			points = points[:, :self.nClasses]

			classPoints = []
			for i in range(self.nClasses):
				classPoints.append([])

			for i in range(0, len(points)):
				p = predsf[i]

				#simply on strongest prob
				# classPoints[np.argmax(p)].append([points[i][0], points[i][1], points[i][2]])
				
				#using threshold
				if p[0] > self.threshold:
					classPoints[np.argmax(p)].append([points[i][0], points[i][1], points[i][2]])
				else:
					classPoints[0].append([points[i][0], points[i][1], points[i][2]])

			#publish points
			for i in range(self.nClasses):

				msg = msgTemplate.Marker()

				msg.header.frame_id = "base_radar_link";
				msg.header.stamp = rospy.Time.now();

				msg.ns = "points"
				msg.id = i
				msg.type = 7
				msg.action = 0

				msg.scale.x = 0.1
				msg.scale.y = 0.1
				msg.scale.z = 0.1

				msg.color.a = 1.0
				if i == 0:
					msg.color.r = 1.0
					msg.color.g = 0.0
					msg.color.b = 0.0
				elif i == 1:
					msg.color.r = 0.0
					msg.color.g = 1.0
					msg.color.b = 0.0
				elif i == 2:
					msg.color.r = 0.0
					msg.color.g = 0.0
					msg.color.b = 1.0
				else:
					msg.color.r = 1.0
					msg.color.g = 1.0
					msg.color.b = 1.0
					print("please add more colours")

				msg.lifetime = rospy.Duration.from_sec(1)

				msg.points = []
				for j in classPoints[i]:
					msg.points.append(Point(x = j[0], y = j[1], z = j[2]))

				self.publisher.publish(msg)


def closePointnet():
	global pointnetQueue
	pointnetQueue.put("quit")

if __name__ == '__main__':

	rospy.on_shutdown(closePointnet)
	rospy.init_node('onlineRadar', anonymous=True)

	tfListener = rostf.TransformListener()

	rospy.Subscriber("/radar/RScan", PointCloud2, radarCallback)
	rospy.Subscriber("/visualization_marker", msgTemplate.Marker, legDetectorCallback)

	pointsPublisher = rospy.Publisher('/deep_radar/out/points', msgTemplate.Marker, queue_size=0)

	pointnetThread = PointnetThread(pointnetQueue, pointsPublisher)
	pointnetThread.start()

	rospy.spin()

	pointnetQueue.put("quit")
