#new stuff
import time
import os
import sys
import math

import h5py
import rospy
import numpy as np
import tf as rosTF
import tensorflow as tf

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point

from pn_model import *
import pn_provider
import pn_tf_util

#############################################
#Adjust these topics and topic types to match
#############################################

# laserPublisher = "/velodyne_points"
# laserPublisherType = "pointcloud"
laserPublisher = "/scan"
laserPublisherType = "laserscan"
# laserPublisher = "/visualization_marker"
# laserPublisherType = "leg_detector"
radarPublisher = "/radar/RScan"

searchAreaBoundaries = {"px": 3.6, "py": 1.5, "pz": 1.8, "nx": 0.5, "ny": -1.0, "nz": -0.4}
clusterSizeBoundaries = {"px": 0.6, "py": 0.6, "pz": 1.5, "nx": -0.6, "ny": -0.6, "nz": -0.3}

maxTimeSinceLaser = rospy.Duration(0, 250000000) #secs, nanosecs
minLaserPoints = 10
minRadarClusterSize = 29
batchSize = 20

gpuDevice = 0
radarPoints = 29

#########################################
#Variables below here don't need changing
#########################################

lp = lg.LaserProjection()
tfListener = None

positionsPublisher = None
inputBoundsPublisher = None
positivePointsPublisher = None
negativePointsPublisher = None

radarFrame = "base_radar_link"
laserFrame = None

lastKnownPosition = {"x": None, "y": None, "z": None, "t": None}

pointsDatabase = []
labelsDatabase = []

sess = None
ops = None
saver = None

def trainModel():

	global pointsDatabase, labelsDatabase, gpuDevice, sess, ops

	train_one_epoch(sess, ops)
	# eval_one_epoch(sess, ops)

def transformPositions(x, y, z, frame):

	global radarFrame, tfListener

	y = y - 0.5

	return x, y, z

	transform = None
	try:
		transform = tfListener.lookupTransform(frame, radarFrame, rospy.Time(0))
	except:
		pass

	if transform == None:
		return None, None, None

	#perform transform here
	return x, y, z

def publishPosition():

	global lastKnownPosition, radarFrame, positionsPublisher

	msg = msgTemplate.Marker()
	msg.header.frame_id = radarFrame
	msg.type = 2
	msg.action = 0

	msg.scale.x = 0.2
	msg.scale.y = 0.2
	msg.scale.z = 0.01

	msg.color.a = 1.0
	msg.color.r = 1.0
	msg.color.g = 0.0
	msg.color.b = 1.0

	msg.pose.position = Point(lastKnownPosition["x"], lastKnownPosition["y"], lastKnownPosition["z"])
	positionsPublisher.publish(msg)

def publishBounds():

	global inputBoundsPublisher, laserFrame, searchAreaBoundaries, clusterSizeBoundaries, lastKnownPosition

	msg = msgTemplate.Marker()
	msg.header.frame_id = laserFrame
	msg.type = 5
	msg.action = 0

	msg.scale.x = 0.05
	msg.scale.y = 0.05
	msg.scale.z = 0.05

	msg.id = 0
	msg.color.a = 1.0
	msg.color.r = 1.0
	msg.color.g = 0.0
	msg.color.b = 1.0

	bounds = {}
	for key in searchAreaBoundaries:
		if "x" in key:
			bounds[key] = searchAreaBoundaries[key]
		if "y" in key:
			bounds[key] = searchAreaBoundaries[key]
		if "z" in key:
			bounds[key] = searchAreaBoundaries[key]

	msg.points = generateBoxPoints(bounds)
	inputBoundsPublisher.publish(msg)

	if lastKnownPosition["x"] and lastKnownPosition["y"]:

		msg.header.frame_id = radarFrame

		msg.id = 1
		msg.color.a = 1.0
		msg.color.r = 0.0
		msg.color.g = 0.0
		msg.color.b = 1.0

		bounds = {}
		for key in clusterSizeBoundaries:
			if "x" in key:
				bounds[key] = lastKnownPosition["x"] + clusterSizeBoundaries[key]
			if "y" in key:
				bounds[key] = lastKnownPosition["y"] + clusterSizeBoundaries[key]
			if "z" in key:
				if lastKnownPosition["z"]:
					bounds[key] = lastKnownPosition["z"] + clusterSizeBoundaries[key]
				else:
					bounds[key] = clusterSizeBoundaries[key]

		msg.points = generateBoxPoints(bounds)
		inputBoundsPublisher.publish(msg)

def generateBoxPoints(sb):

	return [
		#bottom square
		Point(x=sb["nx"], y=sb["ny"], z=sb["nz"]),
		Point(x=sb["px"], y=sb["ny"], z=sb["nz"]),

		Point(x=sb["px"], y=sb["ny"], z=sb["nz"]),
		Point(x=sb["px"], y=sb["py"], z=sb["nz"]),

		Point(x=sb["px"], y=sb["py"], z=sb["nz"]),
		Point(x=sb["nx"], y=sb["py"], z=sb["nz"]),

		Point(x=sb["nx"], y=sb["py"], z=sb["nz"]),
		Point(x=sb["nx"], y=sb["ny"], z=sb["nz"]),

		#top square
		Point(x=sb["nx"], y=sb["ny"], z=sb["pz"]),
		Point(x=sb["px"], y=sb["ny"], z=sb["pz"]),

		Point(x=sb["px"], y=sb["ny"], z=sb["pz"]),
		Point(x=sb["px"], y=sb["py"], z=sb["pz"]),

		Point(x=sb["px"], y=sb["py"], z=sb["pz"]),
		Point(x=sb["nx"], y=sb["py"], z=sb["pz"]),

		Point(x=sb["nx"], y=sb["py"], z=sb["pz"]),
		Point(x=sb["nx"], y=sb["ny"], z=sb["pz"]),

		#sides
		Point(x=sb["nx"], y=sb["ny"], z=sb["nz"]),
		Point(x=sb["nx"], y=sb["ny"], z=sb["pz"]),

		Point(x=sb["nx"], y=sb["py"], z=sb["nz"]),
		Point(x=sb["nx"], y=sb["py"], z=sb["pz"]),

		Point(x=sb["px"], y=sb["py"], z=sb["nz"]),
		Point(x=sb["px"], y=sb["py"], z=sb["pz"]),

		Point(x=sb["px"], y=sb["ny"], z=sb["nz"]),
		Point(x=sb["px"], y=sb["ny"], z=sb["pz"]),
	]

def legDetectorCallback(msg):

	global lastKnownPosition, laserFrame
	print("Leg detector callback")

	if msg.ns != "PEOPLE":
		return

	laserFrame = msg.header.frame_id
	x, y, z = transformPositions(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.frame_id)

	lastKnownPosition.x = x
	lastKnownPosition.y = y
	lastKnownPosition.z = z
	lastKnownPosition.t = msg.header.stamp
	publishBounds()
	publishPosition()

def laserCallback(msg):

	global lastKnownPosition, laserFrame, laserPublisherType, searchAreaBoundaries
	print("Laser callback")

	laserFrame = msg.header.frame_id

	totalPoints = 0
	pointsUsed = 0
	totalX, totalY, totalZ = 0, 0, 0

	if laserPublisherType == "laserscan":
		msg = lp.projectLaser(msg)

	point_generator = pc2.read_points(msg)
	for point in point_generator:

		if math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2]):
			print("Warning: NaN in laser point cloud")
			return

		totalPoints += 1

		if searchAreaBoundaries["nx"] < point[0] and point[0] < searchAreaBoundaries["px"]:
			if searchAreaBoundaries["ny"] < point[1] and point[1] < searchAreaBoundaries["py"]:
				if searchAreaBoundaries["nz"] < point[2] and point[2] < searchAreaBoundaries["pz"]:

						pointsUsed += 1
						totalX += point[0]
						totalY += point[1]
						totalZ += point[2]

	if pointsUsed < minLaserPoints:
		print("Aborting, not enough laser points")
		return

	meanX = float(totalX) / float(pointsUsed)
	meanY = float(totalY) / float(pointsUsed)
	meanZ = float(totalZ) / float(pointsUsed)

	x, y, z = transformPositions(meanX, meanY, meanZ, msg.header.frame_id)

	if x == None:
		print("Unable to get transform from laser to radar")
		return

	lastKnownPosition["x"] = x
	lastKnownPosition["y"] = y
	lastKnownPosition["z"] = z
	lastKnownPosition["t"] = msg.header.stamp

	print("Mean Laser Position: %f, %f, %f, Points used: %i/%i" % (meanX, meanY, meanZ, pointsUsed, totalPoints))

	publishPosition()
	publishBounds()

def radarCallback(msg):

	global lastKnownPosition, clusterSizeBoundaries, minRadarClusterSize, radarFrame, positivePointsPublisher, negativePointsPublisher, pointsDatabase, labelsDatabase
	print("Radar callback")

	if lastKnownPosition["x"] == None:
		print "Aborting radar callback, no laser data yet"
		return

	#make sure position buffer is recent
	now = rospy.get_rostime()
	if now - maxTimeSinceLaser > lastKnownPosition["t"]:
		print "Aborting radar callback, laser position is too old"
		return

	localBounds = {}
	for key in clusterSizeBoundaries:
		if "x" in key:
			localBounds[key] = lastKnownPosition["x"] + clusterSizeBoundaries[key]
		if "y" in key:
			localBounds[key] = lastKnownPosition["y"] + clusterSizeBoundaries[key]
		if "z" in key:
			localBounds[key] = lastKnownPosition["z"] + clusterSizeBoundaries[key]

	points = []
	pointLabels = []
	usedPointsRosMessage = []
	unUsedPointsRosMessage = []

	point_generator = pc2.read_points(msg)

	for point in point_generator:

		if math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2]):
			print("Warning: NaN in radar point cloud")
			return

		if math.isnan(point[3]) or math.isnan(point[4]) or math.isnan(point[5]):
			print("Warning: NaN in radar point cloud")
			return

		insideBB = False

		if localBounds["nx"] < point[0] and point[0] < localBounds["px"]:
			if localBounds["ny"] < point[1] and point[1] < localBounds["py"]:
				if localBounds["nz"] < point[2] and point[2] < localBounds["pz"]:
					insideBB = True

		if insideBB:
			usedPointsRosMessage.append(Point(x = point[0], y = point[1], z = point[2]))
		else:
			unUsedPointsRosMessage.append(Point(x = point[0], y = point[1], z = point[2]))

		x, y, z = point[0], point[1], point[2]
		r, g, b = point[3], 1, point[5] #intensity, range, doppler
		xn, yn, zn = None, None, None

		point = [x, y, z, r, g, b, xn, yn, zn]
		points.append(np.array(point, dtype=np.float32))

		if insideBB:
			pointLabels.append(1)
		else:
			pointLabels.append(0)

	if len(points) < minRadarClusterSize:
		print("Not using radar cluster: too small")
		return

	print("Adding cluster with %i points" % (len(points)))

	m = msgTemplate.Marker()

	m.header.frame_id = radarFrame;
	m.header.stamp = rospy.Time.now();

	m.type = 7
	m.action = 0

	m.scale.x = 0.2
	m.scale.y = 0.2
	m.scale.z = 0.2

	m.color.a = 1.0
	m.color.b = 0.0

	m.color.r = 0.0
	m.color.g = 1.0

	m.points = usedPointsRosMessage
	positivePointsPublisher.publish(m)

	m.color.r = 1.0
	m.color.g = 0.0

	m.points = unUsedPointsRosMessage
	negativePointsPublisher.publish(m)

	maxX = max(points, key=lambda i: i[0])[0]
	maxY = max(points, key=lambda i: i[1])[1]
	maxZ = max(points, key=lambda i: i[2])[2]
	minX = min(points, key=lambda i: i[0])[0]
	minY = min(points, key=lambda i: i[1])[1]
	minZ = min(points, key=lambda i: i[2])[2]

	for i in points:
		i[6] = (i[0] - minX) / (maxX - minX)
		i[7] = (i[1] - minY) / (maxY - minY)
		i[8] = (i[2] - minZ) / (maxZ - minZ)

	points = np.asarray(points)
	pointLabels = np.asarray(pointLabels)

	pointsDatabase.append(points)
	labelsDatabase.append(pointLabels)

	if len(pointsDatabase) == batchSize:
		trainModel()
		del pointsDatabase[0]
		del labelsDatabase[0]

def initModel():

	global sess, ops, saver

	# with tf.Graph().as_default():
	with tf.device('/gpu:'+str(gpuDevice)):
		pointclouds_pl, labels_pl = placeholder_inputs(batchSize, radarPoints)
		is_training_pl = tf.placeholder(tf.bool, shape=())

		batch = tf.Variable(0)
		bn_decay = get_bn_decay(batch)
		tf.summary.scalar('bn_decay', bn_decay)

		pred = get_model(pointclouds_pl, is_training_pl, bn_decay=bn_decay)
		loss = get_loss(pred, labels_pl)
		tf.summary.scalar('loss', loss)

		correct = tf.equal(tf.argmax(pred, 2), tf.to_int64(labels_pl))
		accuracy = tf.reduce_sum(tf.cast(correct, tf.float32)) / float(batchSize*radarPoints)
		tf.summary.scalar('accuracy', accuracy)

		learning_rate = get_learning_rate(batch)
		tf.summary.scalar('learning_rate', learning_rate)
		optimizer = tf.train.AdamOptimizer(learning_rate)
		train_op = optimizer.minimize(loss, global_step=batch)
		
		saver = tf.train.Saver()

	config = tf.ConfigProto()
	config.gpu_options.allow_growth = True
	config.allow_soft_placement = True
	config.log_device_placement = True
	sess = tf.Session(config=config)

	merged = tf.summary.merge_all()

	init = tf.global_variables_initializer()
	sess.run(init, {is_training_pl: True})

	ops = {'pointclouds_pl': pointclouds_pl,
		'labels_pl': labels_pl,
		'is_training_pl': is_training_pl,
		'pred': pred,
		'loss': loss,
		'train_op': train_op,
		'merged': merged,
		'step': batch}

def get_learning_rate(batch):
	learning_rate = tf.train.exponential_decay(
		0.001,
		batch * batchSize,
		300000,
		0.5,
		staircase=True)
	learning_rate = tf.maximum(learning_rate, 0.00001)
	return learning_rate

def get_bn_decay(batch):
	bn_momentum = tf.train.exponential_decay(
		0.5,
		batch*batchSize,
		300000,
		0.5,
		staircase=True)
	bn_decay = tf.minimum(0.99, 1 - 0.99)
	return bn_decay

def train_one_epoch(sess, ops):

	global radarPoints

	pass
 #    current_data, current_label, _ = provider.shuffle_data(train_data[:,0:radarPoints,:], train_label) 
    
 #    file_size = current_data.shape[0]
 #    num_batches = file_size // BATCH_SIZE
    
 #    total_correct = 0
 #    total_seen = 0
 #    loss_sum = 0

	# for batch_idx in range(num_batches):
	# 	start_idx = batch_idx * BATCH_SIZE
	# 	end_idx = (batch_idx+1) * BATCH_SIZE

	# 	feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
	# 		ops['labels_pl']: current_label[start_idx:end_idx],
	# 		ops['is_training_pl']: is_training,}
	# 	summary, step, _, loss_val, pred_val = sess.run([ops['merged'], ops['step'], ops['train_op'], ops['loss'], ops['pred']],
	# 		feed_dict=feed_dict)
	# 	train_writer.add_summary(summary, step)
	# 	pred_val = np.argmax(pred_val, 2)
	# 	correct = np.sum(pred_val == current_label[start_idx:end_idx])
	# 	total_correct += correct
	# 	total_seen += (BATCH_SIZE*NUM_POINT)
	# 	loss_sum += loss_val

	# 	try:
	# 		print('Loss: %f' % (loss_sum / float(num_batches)))
	# 		print('Accuracy: %f' % (total_correct / float(total_seen)))
	# 	except:
	# 		pass


def closeNode():

	global sess, saver

	print("Closing Node")
	saver.save(sess, os.path.join("onlineModel", "model.ckpt"))

if __name__ == "__main__":

	initModel()

	rospy.init_node('deepRadar', anonymous=True)
	rospy.on_shutdown(closeNode)
	print("Node initialised")

	tfListener = rosTF.TransformListener()

	rospy.Subscriber(radarPublisher, PointCloud2, radarCallback)
	if laserPublisherType == "pointcloud":
		rospy.Subscriber(laserPublisher, PointCloud2, laserCallback)
	elif laserPublisherType == "laserscan":
		rospy.Subscriber(laserPublisher, LaserScan, laserCallback)
	elif laserPublisherType == "leg_detector":
		rospy.Subscriber(laserPublisher, msgTemplate.Marker, legDetectorCallback)

	inputBoundsPublisher = rospy.Publisher('/deep_radar/input/input_bounds', msgTemplate.Marker, queue_size=0)
	positionsPublisher = rospy.Publisher('/deep_radar/input/input_position', msgTemplate.Marker, queue_size=0)
	positivePointsPublisher = rospy.Publisher('/deep_radar/input/positive_radar_points', msgTemplate.Marker, queue_size=0)
	negativePointsPublisher = rospy.Publisher('/deep_radar/input/negative_radar_points', msgTemplate.Marker, queue_size=0)

	rospy.spin()
