#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import laser_geometry.laser_geometry as lg
import math

from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point

import sys

import os
import sys
from pn_model import *
import pn_indoor3d_util as indoor3d_util

sess = None
loss = None
pred_softmax = None
is_training_pl = None
pointclouds_pl = None
labels_pl = None

pointsPublisher = None

num_points = 100
threshold = 0.8
nClasses = 2

def eval_one_epoch(sess, ops, points):

	current_data, current_label = indoor3d_util.room2blocks_wrapper_normalized(room_path, NUM_POINT)
	current_data = current_data[:,0:NUM_POINT,:]
	current_label = np.squeeze(current_label)
	# Get room dimension..
	data_label = np.loadtxt(room_path)
	data = data_label[:,0:6]
	max_room_x = max(data[:,0])
	max_room_y = max(data[:,1])
	max_room_z = max(data[:,2])
	
	file_size = current_data.shape[0]
	num_batches = file_size // BATCH_SIZE
	# print(file_size)

	# print(current_data.shape)

	# current_label = np.array([current_label])
	if len(current_label) == num_points:
		current_label = current_label[np.newaxis, ...]

	for batch_idx in range(num_batches):
		start_idx = batch_idx * BATCH_SIZE
		end_idx = (batch_idx+1) * BATCH_SIZE
		cur_batch_size = end_idx - start_idx

		feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
					 ops['labels_pl']: current_label[start_idx:end_idx],
					 ops['is_training_pl']: is_training}

		loss_val, pred_val = sess.run([ops['loss'], ops['pred_softmax']],
									  feed_dict=feed_dict)

		return loss_val, pred_val

def radarCallback(msg):

	global sess, loss, pred_softmax, is_training_pl, pointclouds_pl, labels_pl, pointsPublisher

	point_generator = pc2.read_points(msg)

	minX = 99999
	minY = 99999
	minZ = 99999
	maxX = 0
	maxY = 0
	maxZ = 0

	points = []
	for point in point_generator:
		if not math.isnan(point[0]) and not math.isnan(point[1]) and not math.isnan(point[2]):
			points.append([point[0], point[1], point[2], point[3], 1, point[5]])
			# points.append([point[0], point[1], point[2], 1, 1, 1])
			if point[0] < minX:
				minX = point[0]
			if point[1] < minY:
				minY = point[1]
			if point[2] < minZ:
				minZ = point[2]

			if point[0] > maxX:
				maxX = point[0]
			if point[1] > maxY:
				maxY = point[1]
			if point[2] > maxZ:
				maxZ = point[2]

	if len(points) == 0:
		print("Empty radar callback")
		return

	if len(points) < num_points:
		print("Warning, duplicating points to counter small pc")
		ctr = 0
		while len(points) < num_points:
			# points.append([points[ctr][0], points[ctr][1], points[ctr][2], 1, 1, 1])
			points.append([points[ctr][0], points[ctr][1], points[ctr][2], points[ctr][3], 1, points[ctr][5]])
			ctr += 1
	points = points[:num_points]

	for point in points:
		point.append((point[0] - minX) / (maxX - minX))
		point.append((point[1] - minY) / (maxY - minY))
		point.append((point[2] - minZ) / (maxZ - minZ))

	inputPoints = np.array([points])
	labels = np.array([[0] * num_points])
	
	print("Trying")
	feed_dict = {pointclouds_pl: inputPoints,
				 labels_pl: labels,
				 is_training_pl: False}
	lossf, predsf = sess.run([loss, pred_softmax], feed_dict=feed_dict)

	predsf = predsf[0]
	predsf = predsf[:,:nClasses]
	# print predsf

	nPoints = 0
	totalX = 0
	totalY = 0

	classPoints = []
	for i in xrange(0, nClasses):
		classPoints.append([])

	for i in range(0, len(points)):
		
		#classification
		p = predsf[i]
		#simply on strongest prob
		# classPoints[np.argmax(p)].append([points[i][0], points[i][1], points[i][2]])
		if p[0] > 0.8:
			classPoints[np.argmax(p)].append([points[i][0], points[i][1], points[i][2]])
		else:
			classPoints[1].append([points[i][0], points[i][1], points[i][2]])

	#publish points
	for i in xrange(0, nClasses):

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
		if i == 1:
		 	msg.color.r = 0.0
		 	msg.color.g = 1.0
		 	msg.color.b = 0.0
		if i == 2:
		 	msg.color.r = 0.0
		 	msg.color.g = 0.0
		 	msg.color.b = 1.0

		msg.lifetime = rospy.Duration.from_sec(1)

		msg.points = []
		for j in classPoints[i]:
			msg.points.append(Point(x = j[0], y = j[1], z = j[2]))

		pointsPublisher.publish(msg)

def listener():

	global sess, loss, pred_softmax, is_training_pl, pointclouds_pl, labels_pl, pointsPublisher

	with tf.Graph().as_default():
		with tf.device('/gpu:0'):
			pointclouds_pl, labels_pl = placeholder_inputs(1, num_points)
			is_training_pl = tf.placeholder(tf.bool, shape=())

			# simple model
			pred = get_model(pointclouds_pl, is_training_pl)
			loss = get_loss(pred, labels_pl)
			pred_softmax = tf.nn.softmax(pred)

			# Add ops to save and restore all the variables.
			saver = tf.train.Saver()

		# Create a session
		config = tf.ConfigProto()
		config.gpu_options.allow_growth = True
		config.allow_soft_placement = True
		config.log_device_placement = True
		sess = tf.Session(config=config)

		# Restore variables from disk.
		models = []
		for i in os.walk("models"):
			if i[0] == 'models':
				print i[1]
				models = i[1]
				break
		modelName = raw_input("Select model name: ")
		if modelName not in models:
			print "Not found"
			sys.exit()

		subsets = []
		for i in os.walk(os.path.join("models", modelName)):
			if i[0] == os.path.join("models", modelName):
				print i[1]
				subsets = i[1]
				break
		subset = raw_input("Select subset: ")
		if subset not in subsets:
			print("Not found")
			sys.exit()

		saver.restore(sess, os.path.join("models", modelName, subset, "model.ckpt"))
		print("Model restored.")

		pointsPublisher = rospy.Publisher('/deep_radar/out/points', msgTemplate.Marker, queue_size=0)

		rospy.init_node('radar_analysis', anonymous=False)
		rospy.Subscriber("/radar/RScan/aligned", PointCloud2, radarCallback)
		rospy.spin()

if __name__ == '__main__': 

	listener()
