#!/usr/bin/env python
import rospy
import tf
import numpy
import math
import os
import numpy as np
import h5py
import sys

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point

# laserPublisher = "/velodyne_points"
# laserPublisherType = "pointcloud"
# laserPublisher = "/scan"
# laserPublisherType = "laserscan"
laserPublisher = "/visualization_marker"
laserPublisherType = "leg_detector"
radarPublisher = "/radar/RScan/aligned"

#restrict laser search to inside these boundaries (px = positive x, etc)
#use it to remove ground, walls etc
#dronehumandataset-person
# searchBoundaries = {"px": 10, "py": -0.5, "pz": 10, "nx": 1, "ny": -6, "nz": -0.2}
#dronehumandataset-drone
# searchBoundaries = {"px": 10, "py": 6, "pz": 10, "nx": 1, "ny": -0.5, "nz": -0.2}
#dronedataset
# searchBoundaries = {"px": 10, "py": 5, "pz": 10, "nx": 1, "ny": -6, "nz": -0.2}
#move-no-smoke-3
#searchBoundaries = {"px": 3.6, "py": 1.2, "pz": 1.8, "nx": 0, "ny": -1.2, "nz": -0.4}
#ds/2019-07-17-13-45-23.bag
searchBoundaries = {"px": 3.6, "py": 1.2, "pz": 1.8, "nx": 0, "ny": -1.2, "nz": -0.4}

#restrict laser search to inside these boundaries (px = positive x, etc)
#big
# inputBounds = {"px": 1.0, "py": 1.0, "pz": 2, "nx": -1.0, "ny": -1.0, "nz": -1}
#tight
inputBounds = {"px": 0.4, "py": 0.4, "pz": 1.0, "nx": -0.4, "ny": -0.4, "nz": -0.3}
inputBounds = {"px": 0.5, "py": 0.5, "pz": 1.0, "nx": -0.5, "ny": -0.5, "nz": -0.3}

minLaserPoints = 1
minRadarClusterSize = 29
maxTimeSinceLaser = rospy.Duration(0, 250000000) #secs, nanosecs

lp = lg.LaserProjection()
tfListener = None

meanPositionBuffer = (None, None, None, None)

searchAreaPublisher = None
inputBoundsPublisher = None
meanPublisher = None
radarRepeatPublisher = None

pointsDataset = []
labelsDataset = []

datasets = []
for i in os.walk("datasets"):
	if i[0] == 'datasets':
		print i[1]
		datasets = i[1]
		break
dataset = raw_input("Enter dataset name: ")
if dataset in datasets:
		print "Dataset already exists!"
		sys.exit()
path = os.path.join("datasets", dataset)
if not os.path.exists(path): os.makedirs(path)
if not os.path.exists(os.path.join(path, "velocity")): os.makedirs(os.path.join(path, "velocity"))
if not os.path.exists(os.path.join(path, "no-velocity")): os.makedirs(os.path.join(path, "no-velocity"))

print("Waiting for data...")

def laserCallback(msg):

	global meanPositionBuffer

	frame_id = msg.header.frame_id

	if laserPublisherType == "laserscan":
		msg = lp.projectLaser(msg)

	point_generator = pc2.read_points(msg)

	totalPoints = 0
	pointsUsed = 0
	totalX, totalY, totalZ = 0, 0, 0

	for point in point_generator:

		if math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2]):
			print("Warning: NaN in laser point cloud")
			return

		totalPoints += 1

		if searchBoundaries["nx"] < point[0] and point[0] < searchBoundaries["px"]:
			if searchBoundaries["ny"] < point[1] and point[1] < searchBoundaries["py"]:
				if searchBoundaries["nz"] < point[2] and point[2] < searchBoundaries["pz"]:

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

	meanPositionBuffer = (meanX, meanY, meanZ, msg.header.stamp)

	print("Mean Laser Position: %f, %f, %f" % (meanX, meanY, meanZ))
	print("Total Points Used: %i/%i" % (pointsUsed, totalPoints))

	#publish mean position
	msg = msgTemplate.Marker()
	msg.header.frame_id = frame_id
	msg.type = 2
	msg.action = 0

	msg.scale.x = 0.2
	msg.scale.y = 0.2
	msg.scale.z = 0.01

	msg.color.a = 1.0
	msg.color.r = 1.0
	msg.color.g = 0.0
	msg.color.b = 1.0

	msg.pose.position = Point(meanX, meanY, meanZ)

	meanPublisher.publish(msg)

	#publish search area
	msg = msgTemplate.Marker()
	msg.header.frame_id = frame_id
	msg.type = 5
	msg.action = 0

	msg.scale.x = 0.05
	msg.scale.y = 0.05
	msg.scale.z = 0.05

	msg.color.a = 1.0
	msg.color.r = 1.0
	msg.color.g = 1.0
	msg.color.b = 0.0

	msg.points = generateBoxPoints(searchBoundaries)

	searchAreaPublisher.publish(msg)

	#publish bounds area
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

def legDetectorCallback(msg):
	global meanPositionBuffer

	if msg.ns != "PEOPLE":
		return

	frame_id = msg.header.frame_id

	meanX = float(msg.pose.position.x)
	meanY = float(msg.pose.position.y)
	meanZ = float(msg.pose.position.z)

	meanPositionBuffer = (meanX, meanY, meanZ, msg.header.stamp)

	print("Mean Laser Position: %f, %f, %f" % (meanX, meanY, meanZ))

	#publish mean position
	msg = msgTemplate.Marker()
	msg.header.frame_id = frame_id
	msg.type = 2
	msg.action = 0

	msg.scale.x = 0.2
	msg.scale.y = 0.2
	msg.scale.z = 0.01

	msg.color.a = 1.0
	msg.color.r = 1.0
	msg.color.g = 0.0
	msg.color.b = 1.0

	msg.pose.position = Point(meanX, meanY, meanZ)

	meanPublisher.publish(msg)

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
		transform = listener.lookupTransform('laser', 'base_radar_link', rospy.Time(0))
	except:
		print("Aborting, unable to get tf transformation")
		return

	translation = transform[0]
	euler = tf.transformations.euler_from_quaternion(transform[1])

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

def writeData():

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

	for i in xrange(0, len(pointsDataset)):
		pointsDataset[i] = pointsDataset[i][:minPoints]
		labelsDataset[i] = labelsDataset[i][:minPoints]

	pointsDataset = np.asarray(pointsDataset)
	labelsDataset = np.asarray(labelsDataset)

	dataFile = os.path.join(path, "velocity", "data.h5")
	f = h5py.File(dataFile, 'w')
	label = f.create_dataset("label", data=labelsDataset)
	data = f.create_dataset("data", data=pointsDataset)
	f.close()

	with open(os.path.join(path, "velocity", "class_names.txt"), "w") as f:
		f.write("noise\nperson\n")

	#strip out velocity
	for i in xrange(0, len(pointsDataset)):
		for j in xrange(0, len(pointsDataset[i])):
			pointsDataset[i][j][3] = 1
			pointsDataset[i][j][4] = 1
			pointsDataset[i][j][5] = 1

	dataFile = os.path.join(path, "no-velocity", "data.h5")
	f = h5py.File(dataFile, 'w')
	label = f.create_dataset("label", data=labelsDataset)
	data = f.create_dataset("data", data=pointsDataset)
	f.close()

	with open(os.path.join(path, "no-velocity", "class_names.txt"), "w") as f:
		f.write("noise\nperson\n")

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

if __name__ == '__main__':

	rospy.on_shutdown(writeData)
	rospy.init_node('radarAlignment', anonymous=True)

	tfListener = listener = tf.TransformListener()

	rospy.Subscriber(radarPublisher, PointCloud2, radarCallback)
	if laserPublisherType == "pointcloud":
		rospy.Subscriber(laserPublisher, PointCloud2, laserCallback)
	elif laserPublisherType == "laserscan":
		rospy.Subscriber(laserPublisher, LaserScan, laserCallback)
	elif laserPublisherType == "leg_detector":
		rospy.Subscriber(laserPublisher, msgTemplate.Marker, legDetectorCallback)

	searchAreaPublisher = rospy.Publisher('/deep_radar/input/search_area_publisher', msgTemplate.Marker, queue_size=0)
	inputBoundsPublisher = rospy.Publisher('/deep_radar/input/input_bounds', msgTemplate.Marker, queue_size=0)
	meanPublisher = rospy.Publisher('/deep_radar/input/estimated_position', msgTemplate.Marker, queue_size=0)
	radarRepeatPublisher = rospy.Publisher('/deep_radar/input/radar_repeat', msgTemplate.Marker, queue_size=0)

	rospy.spin()
