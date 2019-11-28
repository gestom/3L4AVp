#!/usr/bin/env python
import rospy
import tensorflow as tf
import tf as tfTrans
import numpy
import math
import os
import numpy as np
import h5py
import sys

# import tf2_ros
# import tf2_py as tf2

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import std_msgs.msg as std_msgs

sys.path.insert(0, 'nn-utils')

from model import *
import indoor3d_util

import argparse
import socket
import provider
import tf_util

import shutil

shutil.rmtree("tmp")
os.mkdir("tmp")
os.mkdir("tmp/velocity")

laserPublisher = "/visualization_marker"
laserPublisherType = "leg_detector"
radarPublisher = "/radar/RScan"

#restrict laser search to inside these boundaries (px = positive x, etc)
# inputBounds = {"px": 1.0, "py": 1.0, "pz": 2, "nx": -1.0, "ny": -1.0, "nz": -1}
inputBounds = {"px": 0.4, "py": 0.4, "pz": 1.0, "nx": -0.4, "ny": -0.4, "nz": -0.3}
inputBounds = {"px": 0.5, "py": 0.5, "pz": 1.0, "nx": -0.5, "ny": -0.5, "nz": -0.3}

minRadarClusterSize = 29
maxTimeSinceLaser = rospy.Duration(0, 250000000) #secs, nanosecs

tfListener = None
learning = True

meanPositionBuffer = (None, None, None, None)

inputBoundsPublisher = None
radarRepeatPublisher = None

pointsDataset = []
labelsDataset = []

print("Waiting for data...")

def legDetectorCallback(msg):
	global meanPositionBuffer

	if msg.ns != "PEOPLE":
		return

	frame_id = msg.header.frame_id

	meanX = float(msg.pose.position.x)
	meanY = float(msg.pose.position.y)
	meanZ = float(msg.pose.position.z)

	meanPositionBuffer = (meanX, meanY, meanZ, msg.header.stamp)

	msg = msgTemplate.Marker()
	msg.header.frame_id = frame_id
	msg.type = 5
	msg.action = 0

	msg.scale.x = 0.05
	msg.scale.y = 0.05
	msg.scale.z = 0.05

	msg.color.a = 1.0
	msg.color.r = 0.0
	msg.color.g = 0.0
	msg.color.b = 1.0

	localBounds = {}
	for key in inputBounds:
		if "x" in key:
			localBounds[key] = meanX + inputBounds[key]
		if "y" in key:
			localBounds[key] = meanY + inputBounds[key]
		if "z" in key:
			localBounds[key] = meanZ + inputBounds[key]

	msg.points = generateBoxPoints(localBounds)

	inputBoundsPublisher.publish(msg)

def radarCallback(msg):

	if meanPositionBuffer[0] == None:
		print "Aborting, not enough laser data yet"
		return

	#make sure position buffer is recent
	now = rospy.get_rostime()
	if now - maxTimeSinceLaser > meanPositionBuffer[3]:
		print "Aborting, laser position is too old"
		return

	#perform tf transform
	transform = None
	try:
		# transform = listener.lookupTransform('base_radar_link', 'laser', rospy.Time(0))
		transform = tfListener.lookupTransform('laser', 'base_radar_link', rospy.Time(0))
	except:
		print("Aborting, unable to get tf transformation")
		return

	translation = transform[0]
	euler = tfTrans.transformations.euler_from_quaternion(transform[1])

	transformationMatrix = numpy.array([
		[1, 0, 0, translation[0]], 
		[0, 1, 0, translation[1]], 
		[0, 0, 1, translation[2]], 
		[0, 0, 0, 1]])

	localBounds = {}
	for key in inputBounds:
		if "x" in key:
			localBounds[key] = meanPositionBuffer[0] + inputBounds[key]
		if "y" in key:
			localBounds[key] = meanPositionBuffer[1] + inputBounds[key]
		if "z" in key:
			localBounds[key] = meanPositionBuffer[2] + inputBounds[key]

	points = []
	pointLabels = []
	allAvailablePoints = []

	point_generator = pc2.read_points(msg)

	for point in point_generator:

		if math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2]):
			print("Warning: NaN in radar point cloud")
			return

		if math.isnan(point[3]) or math.isnan(point[4]) or math.isnan(point[5]):
			print("Warning: NaN in radar point cloud")
			return

		insideBB = False

		mulPoint = numpy.append(point[0:3], 1.0)
		mulpoint = np.matmul(transformationMatrix, mulPoint)

		if localBounds["nx"] < mulpoint[0] and mulpoint[0] < localBounds["px"]:
			if localBounds["ny"] < mulpoint[1] and mulpoint[1] < localBounds["py"]:
				if localBounds["nz"] < mulpoint[2] and mulpoint[2] < localBounds["pz"]:
					insideBB = True

		if insideBB:
			allAvailablePoints.append(Point(x = mulpoint[0], y = mulpoint[1], z = mulpoint[2]))

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

	m.header.frame_id = "laser";
	m.header.stamp = rospy.Time.now();

	m.type = 7
	m.action = 0

	m.scale.x = 0.2
	m.scale.y = 0.2
	m.scale.z = 0.2

	m.color.a = 1.0
	m.color.r = 0.0
	m.color.g = 1.0
	m.color.b = 0.5

	m.points = allAvailablePoints

	radarRepeatPublisher.publish(m)

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

	pointsDataset.append(points)
	labelsDataset.append(pointLabels)

	writeFile()

def writeFile():

	global pointsDataset, labelsDataset

	minPoints = len(min(pointsDataset, key=lambda i: len(i)))
	maxPoints = len(max(pointsDataset, key=lambda i: len(i)))

	lens = {}
	for i in xrange(minPoints, maxPoints + 1):
		lens[int(i)] = 0
	for i in pointsDataset:
		lens[len(i)] = lens[len(i)] + 1
	print lens

	print(minPoints)

	print("Cluster size range: %i-%i" % (minPoints, maxPoints))
	print("Truncating all to the minimum size for constant tensor shape")

	path = "tmp"

	tmppointsDataset = pointsDataset
	tmplabelsDataset = labelsDataset

	for i in xrange(0, len(pointsDataset)):
		tmppointsDataset[i] = tmppointsDataset[i][:minPoints]
		tmplabelsDataset[i] = tmplabelsDataset[i][:minPoints]

	tmppointsDataset = np.asarray(tmppointsDataset)
	tmplabelsDataset = np.asarray(tmplabelsDataset)

	try:
		dataFile = os.path.join(path, "velocity", "data.h5")
		f = h5py.File(dataFile, 'w')
		label = f.create_dataset("label", data=tmplabelsDataset)
		data = f.create_dataset("data", data=tmppointsDataset)
		f.close()

		with open(os.path.join(path, "velocity", "class_names.txt"), "w") as f:
			f.write("noise\nperson\n")
	except:
		print("skipping file write")

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

def stopLearningCallback(msg):
	global learning
	learning = not msg.data

def myhook():
	print("Closing node")

if __name__ == '__main__':

	rospy.init_node('onlineLearning', anonymous=True)
	rospy.on_shutdown(exitNode)

	tfListener = tfTrans.TransformListener()

	rospy.Subscriber(radarPublisher, PointCloud2, radarCallback)
	rospy.Subscriber(laserPublisher, msgTemplate.Marker, legDetectorCallback)
	rospy.Subscriber("radar_detector_ol/training_status", std_msgs.Bool, stopLearningCallback)


	inputBoundsPublisher = rospy.Publisher('/deep_radar/input/input_bounds', msgTemplate.Marker, queue_size=0)
	radarRepeatPublisher = rospy.Publisher('/deep_radar/input/radar_repeat', msgTemplate.Marker, queue_size=0)

	rospy.spin()
