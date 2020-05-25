#!/usr/bin/env python
import random
import rospy
import math
import os
import h5py
import sys
import socket
import Queue
import copy
import threading

import numpy as np
import tf as rostf
import tensorflow as tf
from numpy.random import seed

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

tf.random.set_random_seed(seed=1)

tfListener = None
legDetectorBuffers = []
legDetectorFrame = None
maxTimeSinceLaser = rospy.Duration(0, 250000000) #secs, nanosecs
maxDistanceToObj = 0.5
radarFlags = [True, True, True, True, True, True] #xyz, intensity, range, doppler
pointnetQueue = Queue.Queue()
maxNumPoints = 40
pointsAllowedToDuplicate = 6
biasX, biasY = 0.0, 0.0
biasBufferX, biasBufferY = [], []
biasCount = 25

def legDetectorCallback(msg):
	global legDetectorBuffers, legDetectorFrame
	legDetectorFrame = msg.header.frame_id
	if msg.ns != "PEOPLE":
		return
	#we store leg detections by the id so that we can track multiple
	while msg.id >= len(legDetectorBuffers):
		legDetectorBuffers.append(None)
	legDetectorBuffers[msg.id] = msg

def radarCallback(msg):
	global legDetectorBuffers, legDetectorFrame, tfListener, pointnetQueue, maxNumPoints, biasX, biasY, biasCount, biasBufferY, biasBufferX
	
	trainable = True

	if legDetectorFrame == None or len(legDetectorBuffers) == 0:
		print("No leg detector inputs...")
		return

	#make a list of recent leg detections 
	now = rospy.get_rostime()
	def filterOld(x):
		if x == None or (now - maxTimeSinceLaser > x.header.stamp):
			return False
		return True
	usedlegDetectorBufs = filter(filterOld, legDetectorBuffers)

	#get transform from leg to radar
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

	totalDiffX, totalDiffY = 0.0, 0.0
	nInCluster = 0

	minX, minY, minZ = 0, 0, 0
	maxX, maxY, maxZ = 0, 0, 0

	point_generator = pc2.read_points(msg)
	for point in point_generator:

		isObject = False

		transformedPoint = np.append(point[0:3], 1.0)
		transformedPoint = np.dot(mat, transformedPoint)
		point = np.append(transformedPoint[0:3], point[3:])

		now = rospy.get_rostime()

		for obj in usedlegDetectorBufs:
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
		xn, yn, zn = 0, 0, 0

		if x > maxX:
			maxX = x
		if y > maxY:
			maxY = y
		if z > maxZ:
			maxZ = z
		if x < minX:
			minX = x
		if y < minY:
			minY = y
		if z < minZ:
			minZ = z			

		point = np.array([x, y, z, r, g, b, xn, yn, zn], dtype=np.float32)
		points.append(point)

		if isObject:
			labels.append(1)
		else:
			labels.append(0)

	if nInCluster == 0:
		trainable = False

	#calculate normalised values
	sceneSizeX = maxX - minX
	sceneSizeY = maxY - minY
	sceneSizeZ = maxZ - minZ
	for idx in range(len(points)):
		points[idx][6] = (points[idx][0] - minX) / sceneSizeX
		points[idx][7] = (points[idx][1] - minY) / sceneSizeY
		points[idx][8] = (points[idx][2] - minZ) / sceneSizeZ

	if nInCluster != 0:
		totalDiffX = totalDiffX / nInCluster
		totalDiffY = totalDiffY / nInCluster
		biasBufferX.append(totalDiffX)
		biasBufferY.append(totalDiffY)

		if len(biasBufferX) > biasCount:
			del biasBufferX[0]
		if len(biasBufferY) > biasCount:
			del biasBufferY[0]

		total = 0
		for i in biasBufferX:
			total += i
		biasX = total / len(biasBufferX)
		total = 0
		for i in biasBufferY:
			total += i
		biasY = total / len(biasBufferY)

	if len(points) < maxNumPoints - pointsAllowedToDuplicate:
		trainable = False
		idx = 0
		while len(points) < maxNumPoints:
			points.append(points[idx % len(points)])
			labels.append(labels[idx % len(labels)])
			idx += 1
		pointnetQueue.put((np.array(points[:]), np.array(labels[:]), trainable))

	elif len(points) < maxNumPoints:

		#order list by feature
		orderedFeatures = list(zip(points, labels))
		orderedFeatures = list(reversed(sorted(orderedFeatures, key=lambda x: x[0][3])))

		#duplicate them
		idx = 0
		while len(points) < maxNumPoints:
			points.append(orderedFeatures[idx][0])
			labels.append(orderedFeatures[idx][1])
			idx += 1
		pointnetQueue.put((np.array(points[:]), np.array(labels[:]), trainable))

	elif len(points) == maxNumPoints:
		pointnetQueue.put((np.array(points[:]), np.array(labels[:]), trainable))

	elif len(points) > maxNumPoints:
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

			del points[lowestIntensity]
			del labels[lowestIntensity]
		pointnetQueue.put((np.array(points[:]), np.array(labels[:]), trainable))

class PointnetThread(threading.Thread):
	def __init__(self, queue, publisher, nPoints):
		threading.Thread.__init__(self)
		self.queue = queue
		self.publisher = publisher
		self.pointBuffer = []
		self.labelBuffer = []
		self.frameNumber = 0
		self.trainEveryNFrames = 3
		self.epochsPerMessage = 10

		self.batchSize = 20
		self.threshold = 0.13
		self.numPoints = nPoints
		self.nClasses = 2
		self.augmentationFrames = 2
		self.augmentationDist = 0.2
		self.augmentationNoise = 0.03

		self.baseLearningRate = 0.0005
		self.decayStep = 3000.0
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

	def augment(self, msg):

		for i in range(self.augmentationFrames):

			distX = random.random() * self.augmentationDist
			distY = random.random() * self.augmentationDist

			newMsgA = copy.deepcopy(msg[0])
			newMsgB = copy.deepcopy(msg[1])

			for i in range(len(newMsgA)):
				if newMsgB[i] != 0:
					newMsgA[i][0] += distX + (random.random() * self.augmentationNoise)
					newMsgA[i][1] += distY + (random.random() * self.augmentationNoise)
					newMsgA[i][2] += (random.random() * self.augmentationNoise)
					newMsgA[i][4] += (random.random() * self.augmentationNoise)

			self.pointBuffer.append(newMsgA)
			self.labelBuffer.append(newMsgB)

		self.pointBuffer.append(msg[0])
		self.labelBuffer.append(msg[1])

		while len(self.pointBuffer) > 250:
			del self.pointBuffer[0]
			del self.labelBuffer[0]

	def run(self):
		while True:

			msg = ""
			while self.queue.qsize() > 1:
				msg = self.queue.get(block=False)
				if msg == "quit":
					print("Closing net")
					return
				if msg[2] == True:
					self.augment(msg)

			msg = self.queue.get(block=True)

			if msg == "quit":
				print("Closing net")
				return

			if msg[2] == True:
				self.augment(msg)

				#train
				preds = []
				if self.frameNumber > 100:
					self.trainEveryNFrames = 25

					# self.learning_rate = tf.train.exponential_decay(
					# 	self.baseLearningRate,
					# 	self.batchSize,
					# 	self.decayStep,
					# 	self.decayRate,
					# 	staircase=True)
					# self.learning_rate = tf.maximum(self.learning_rate, 0.00001)
					# tf.summary.scalar('learning_rate', self.learning_rate)
					# # self.optimizer = tf.train.AdamOptimizer(self.learning_rate)
					# #self.optimizer = tf.keras.optimizers.Adadelta()
					# #self.train_op = self.optimizer.minimize()
					# # self.train_op = self.optimizer.minimize(self.loss, [])
					# self.optimizer = tf.train.AdamOptimizer(self.learning_rate)
					# self.train_op = self.optimizer.minimize(self.loss, global_step=self.batch)

				if self.frameNumber % self.trainEveryNFrames == 0:
					_ = self.train()
				self.frameNumber += 1

			#infer
			preds = self.inferTemp(msg)

			#publish
			self.publish(preds, msg)

	def probMax(self, x):
	        if x == 1:
	                return 1
	        else:
	                return x + self.probMax(x - 1)

	def getBatches(self):

		batch = []
		labels = []

		for _ in range(self.batchSize):

			idx = random.randint(0, len(self.pointBuffer) - 1)
			batch.append(self.pointBuffer[idx])
			labels.append(self.labelBuffer[idx])

		return batch, labels

	def train(self):

		for epoch in range(self.epochsPerMessage):

			data, labels = self.getBatches()

			current_data = np.array(data)
			current_label = np.array(labels)

			file_size = current_data.shape[0]
			num_batches = file_size

			print("Training round ")
			#print(current_label)

			feed_dict = {self.ops['pointclouds_pl']: current_data[0:self.batchSize, :, :],
				self.ops['labels_pl']: current_label[0:self.batchSize],
				self.ops['is_training_pl']: True}

			summary, step, _, loss_val, pred_val = self.sess.run([self.ops['merged'], self.ops['step'], self.ops['train_op'], self.ops['loss'], self.ops['pred']],
				feed_dict=feed_dict)

			self.file.write(str(loss_val) + "\n")
			self.file.flush()

			print("Training round finished")

			return pred_val

	def inferTemp(self, msg):

		pointBuf = []
		labelBuf = []
		for i in range(self.batchSize):
			pointBuf.append(msg[0])
			labelBuf.append(msg[1])

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
		# print(loss_val)
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
				# msg.points.append(Point(x = j[0] + biasX, y = j[1] + biasY, z = j[2]))
				# msg.points.append(Point(x = j[0], y = j[1], z = j[2]))
				msg.points.append(Point(x = j[0], y = j[1], z = j[2]))

			self.publisher.publish(msg)

def closePointnet():
	global pointnetQueue
	pointnetQueue.put("quit")

if __name__ == '__main__':

	rospy.on_shutdown(closePointnet)
	rospy.init_node('onlineRadar', anonymous=True)

	tfListener = rostf.TransformListener()

	#rospy.Subscriber("/radar/RScan", PointCloud2, radarCallback)
        #negating movement from doppler:
	rospy.Subscriber("/radar_fixed", PointCloud2, radarCallback)
	rospy.Subscriber("/visualization_marker", msgTemplate.Marker, legDetectorCallback)

	pointsPublisher = rospy.Publisher('/deep_radar/out/points', msgTemplate.Marker, queue_size=0)

	pointnetThread = PointnetThread(pointnetQueue, pointsPublisher, maxNumPoints)
	pointnetThread.start()

	rospy.spin()

	pointnetQueue.put("quit")
