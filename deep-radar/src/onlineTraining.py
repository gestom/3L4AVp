#!/usr/bin/env python
import random
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
maxNumPoints = 30
pointsAllowedToDuplicate = 10
biasX, biasY = 0.0, 0.0
biasCount = 0

def legDetectorCallback(msg):
	global legDetectorBuffers, legDetectorFrame
	legDetectorFrame = msg.header.frame_id
	if msg.ns != "PEOPLE":
		return
	while msg.id >= len(legDetectorBuffers):
		legDetectorBuffers.append(None)
	legDetectorBuffers[msg.id] = msg

def radarCallback(msg):
	global legDetectorBuffers, legDetectorFrame, tfListener, pointnetQueue, maxNumPoints, biasX, biasY, biasCount

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
	objectPresent = False

	totalDiffX, totalDiffY = 0.0, 0.0
	nInCluster = 0

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
				totalDiffX += diffX
				totalDiffY += diffY
				nInCluster += 1

		x, y, z = point[0], point[1], point[2]
		r, g, b = point[3], point[4], point[5] #intensity, range, doppler
		xn, yn, zn = None, None, None
		xn, yn, zn = 0, 0, 0

		point = np.array([x, y, z, r, g, b, xn, yn, zn], dtype=np.float32)
		points.append(point)

		if isObject:
			objectPresent = True
			labels.append(1)
		else:
			labels.append(0)

	if nInCluster > 0:
		totalDiffX = totalDiffX / nInCluster
		totalDiffY = totalDiffY / nInCluster

		biasX = ((biasX * biasCount) + (totalDiffX)) / (biasCount + 1)
		biasY = ((biasY * biasCount) + (totalDiffY)) / (biasCount + 1)
		biasCount += 1

		# print(biasX, biasY)

	#todo calculate normalised vals

	if objectPresent == True:

		if len(points) < maxNumPoints - pointsAllowedToDuplicate:
			return

		if len(points) < maxNumPoints:

			# while len(points) < maxNumPoints:
			# 	lowestRange = -1
			# 	lowestRangeVal = 999
			# 	highestIntensity = -1
			# 	highestIntensityVal = -1

			# 	for idx in range(len(points)):

			# 		if labels[idx] == 0:
			# 			continue

			# 		i = points[idx]
			# 		if i[3] < lowestRangeVal:
			# 			lowestRangeVal = i[3]
			# 			lowestRange = idx
			# 		if i[4] > highestIntensityVal:
			# 			highestIntensityVal = i[4]
			# 			highestIntensity = idx

			# 	if lowestIntensity == -1 or highestRange == -1:
			# 		break

			# 	points.append()

			# 	del points[highestRange]
			# 	del labels[highestRange]

			idx = 0
			while len(points) < maxNumPoints:
				points.append(points[idx])
				labels.append(labels[idx])
				idx += 1

			print(len(points), len(labels), "l")
			pointnetQueue.put((np.array(points[:]), np.array(labels[:])))

		if len(points) == maxNumPoints:
			# pointnetQueue.put((np.array(points[:maxNumPoints]), np.array(labels[:maxNumPoints])))
			print(len(points), len(labels), "e")
			pointnetQueue.put((np.array(points[:]), np.array(labels[:])))

		if len(points) > maxNumPoints:

			while len(points) > maxNumPoints:
				lowestIntensity = -1
				lowestIntensityVal = 999
				highestRange = -1
				highestRangeVal = -1

				for idx in range(len(points)):

					if labels[idx] > 0:
						continue

					i = points[idx]
					if i[3] < lowestIntensityVal:
						lowestIntensityVal = i[3]
						lowestIntensity = idx
					if i[4] > highestRangeVal:
						highestRangeVal = i[4]
						highestRange = idx

				if lowestIntensity == -1 or highestRange == -1:
					return

				del points[highestRange]
				del labels[highestRange]

			print(len(points), len(labels), "b")
			pointnetQueue.put((np.array(points[:]), np.array(labels[:])))

class PointnetThread(threading.Thread):
	def __init__(self, queue, publisher, nPoints):
		threading.Thread.__init__(self)
		self.queue = queue
		self.publisher = publisher
		self.pointBuffer = []
		self.labelBuffer = []
		self.frameNumber = 0
		self.trainEveryNFrames = 3
		self.epochsPerMessage = 1
		self.messageDB = []

		self.batchSize = 20
		self.threshold = 0.3
		self.numPoints = nPoints
		self.nClasses = 2

		self.baseLearningRate = 0.0005
		self.decayStep = 300000.0
		self.decayRate = 0.05

		self.config = tf.ConfigProto()
		self.config.gpu_options.allow_growth = True
		self.config.allow_soft_placement = True
		self.config.log_device_placement = True

		self.graph = tf.Graph()
		self.device = tf.device("/gpu:0")
		self.sess = tf.Session(graph=self.graph, config=self.config)

		self.file = open("losslog", "w")

		with self.graph.as_default():
			with self.device:

				self.pointclouds_pl, self.labels_pl = placeholder_inputs(self.batchSize, self.numPoints)
				self.is_training_pl = tf.placeholder(tf.bool, shape=())

				self.batch = tf.Variable(0)
				self.bn_momentum = tf.train.exponential_decay(
					0.5,
					self.batch * self.batchSize,
					300000.0,
					0.5,
					staircase=True)
				self.bn_decay = tf.minimum(0.99, 1 - self.bn_momentum)
				tf.summary.scalar('bn_decay', self.bn_decay)

				self.pred = get_model(self.pointclouds_pl, self.is_training_pl, bn_decay=self.bn_decay)
				self.loss = get_loss(self.pred, self.labels_pl)
				tf.summary.scalar('loss', self.loss)

				self.correct = tf.equal(tf.argmax(self.pred, 2), tf.to_int64(self.labels_pl))
				self.accuracy = tf.reduce_sum(tf.cast(self.correct, tf.float32)) / float(self.batchSize*self.numPoints)
				tf.summary.scalar('accuracy', self.accuracy)

				self.learning_rate = tf.train.exponential_decay(
					self.baseLearningRate,
					self.batchSize,
					self.decayStep,
					self.decayRate,
					staircase=True)
				self.learning_rate = tf.maximum(self.learning_rate, 0.00001)
				tf.summary.scalar('learning_rate', self.learning_rate)
				# self.optimizer = tf.train.AdamOptimizer(self.learning_rate)
				#self.optimizer = tf.keras.optimizers.Adadelta()
				#self.train_op = self.optimizer.minimize()
				# self.train_op = self.optimizer.minimize(self.loss, [])
				self.optimizer = tf.train.AdamOptimizer(self.learning_rate)
				self.train_op = self.optimizer.minimize(self.loss, global_step=self.batch)

				self.pred_softmax = tf.nn.softmax(self.pred)

				self.saver = tf.train.Saver()

				self.merged = tf.summary.merge_all()

				init = tf.global_variables_initializer()
				self.sess.run(init, {self.is_training_pl:True})

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

			originalMsg = None
			while True:
				msg = None
				try:
					msg = self.queue.get(block=False)
				except:
					break

				if msg == "quit":
					print("Closing net")
					return

				self.pointBuffer.append(msg[0])
				self.labelBuffer.append(msg[1])

				if len(self.pointBuffer) > self.batchSize:
					del self.pointBuffer[0]
					del self.labelBuffer[0]

				originalMsg = msg

			if len(self.pointBuffer) < self.batchSize or originalMsg == None:
				continue

			self.messageDB.append(originalMsg)
			if len(self.messageDB) < self.batchSize:
				continue

			print("Training")

			#train
			preds = []
			self.frameNumber += 1
			if self.frameNumber % self.trainEveryNFrames == 0:
				preds = self.train()

			#infer
			preds = self.inferTemp()

			#publish
			self.publish(preds, originalMsg)

	def probMax(self, x):
	        if x == 1:
	                return 1
	        else:
	                return x + self.probMax(x - 1)

	def getBatches(self):

		batch = []
		labels = []

		print(range(self.batchSize))
		for _ in range(self.batchSize):

			idx = random.randint(0, self.probMax(len(self.messageDB)) - 1)
			numerator = 0

			for i in xrange(0, len(self.messageDB)):
				numerator += i
				if idx <= numerator:
					print("appending")
					batch.append(self.messageDB[i][0])
					labels.append(self.messageDB[i][1])
					break
			print("y")

		return batch, labels

	def train(self):

		for epoch in range(self.epochsPerMessage):

			batch, labels = self.getBatches()

			current_data = np.array(batch)
			current_label = np.array(labels)

			file_size = current_data.shape[0]
			num_batches = file_size

			#print(current_label)

			feed_dict = {self.ops['pointclouds_pl']: current_data[0:self.batchSize, :, :],
				self.ops['labels_pl']: current_label[0:self.batchSize],
				self.ops['is_training_pl']: True}

			summary, step, _, loss_val, pred_val = self.sess.run([self.ops['merged'], self.ops['step'], self.ops['train_op'], self.ops['loss'], self.ops['pred']],
				feed_dict=feed_dict)


			#print("lr: " + str(self.learning_rate[0]))
			#print(self.learning_rate)
			#print(loss_val)
			self.file.write(str(loss_val) + "\n")
			self.file.flush()

			return pred_val

	def inferTemp(self):

		pointBuf = []
		labelBuf = []
		for i in range(len(self.pointBuffer)):
			pointBuf.append(self.pointBuffer[-1])
			labelBuf.append(self.labelBuffer[-1])

		current_data = np.array(pointBuf)
		current_label = np.array(labelBuf)

		file_size = current_data.shape[0]
		num_batches = file_size

		#print(current_label)

		feed_dict = {self.ops['pointclouds_pl']: current_data[0:self.batchSize, :, :],
			self.ops['labels_pl']: current_label[0:self.batchSize],
			self.ops['is_training_pl']: False}

		summary, step, _, loss_val, pred_val = self.sess.run([self.ops['merged'], self.ops['step'], self.ops['train_op'], self.ops['loss'], self.ops['pred']],
			feed_dict=feed_dict)
		print(loss_val)
		self.file.write(str(loss_val) + "\n")
		self.file.flush()

		return pred_val

	# def infer(self, msg):

		# print("Running inference")

		# current_data = np.array(msg[0] * self.batchSize)
		# # current_label = np.array(msg[1] * self.batchSize)

		# file_size = current_data.shape[0]
		# num_batches = file_size

		# print(current_label)

		# feed_dict = {self.ops['pointclouds_pl']: current_data[0:self.batchSize, :, :],
		# 	self.ops['labels_pl']: current_label[0:self.batchSize],
		# 	self.ops['is_training_pl']: True}

		# summary, step, _, loss_val, pred_val = self.sess.run([self.ops['merged'], self.ops['step'], self.ops['train_op'], self.ops['loss'], self.ops['pred']],
		# 	feed_dict=feed_dict)

		# inputPoints = msg[0]
		# labels = msg[1]

		# feed_dict = {self.pointclouds_pl: inputPoints,
		# 			 self.labels_pl: labels,
		# 			 self.is_training_pl: False}
		# lossf, predsf = self.sess.run([self.loss, self.pred_softmax], feed_dict=feed_dict)

		# return (lossf, predsf)

	def publish(self, points, originalMsg):

		#print(points.shape)

		points = points[-1]
		points = points[:, :self.nClasses]

		classPoints = []
		for i in range(self.nClasses):
			classPoints.append([])

		for i in range(0, len(points)):
			p = points[i]
			pos = originalMsg[0]

			#simply on strongest prob
			# if p[1] > p[0]:
			# 	classPoints[1].append([pos[i][0], pos[i][1], pos[i][2]])
			# else:
			# 	classPoints[0].append([pos[i][0], pos[i][1], pos[i][2]])

			#using threshold
			#print(p)
			if p[1] > self.threshold:
				classPoints[1].append([pos[i][0], pos[i][1], pos[i][2]])
			else:
				classPoints[0].append([pos[i][0], pos[i][1], pos[i][2]])

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
				msg.points.append(Point(x = j[0] + biasX, y = j[1] + biasY, z = j[2]))

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

	pointnetThread = PointnetThread(pointnetQueue, pointsPublisher, maxNumPoints)
	pointnetThread.start()

	rospy.spin()

	pointnetQueue.put("quit")
