#!/usr/bin/env python
import numpy as np
from sklearn.cluster import KMeans
import cv2
import os
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math

from sklearn.cluster import DBSCAN
import numpy as np

import random

clusterDistance = 1 
minClusterPoints = 4

visPublisher = None
publisher = None

def callback(msg):

	if msg.id != 0:

                originalMsg = msg

		buf = []
		for i in msg.points:
			buf.append([i.x, i.y])

		if len(buf) == 0:
			return

		# print("buf:")
		# print buf
		clusterer = DBSCAN(eps=0.4, min_samples=5).fit(buf)
		clusterNames = list(set(clusterer.labels_))
		clusters = {}

		if len(clusterNames) == 0:
			return

		for i in clusterNames:
			clusters[i] = []

		for i in xrange(0, len(msg.points)):
			clusters[clusterer.labels_[i]].append(msg.points[i])

		print "Found %i clusters" % (len(clusters))

		positions = []

		for key in clusters:

			cluster = clusters[key]

			if key == "-1":
				continue

			if len(cluster) < 5:
				continue

			msg = msgTemplate.Marker()

			msg.header.frame_id = "base_radar_link"
			msg.header.stamp = originalMsg.header.stamp

			msg.ns = "clusters"
			msg.id = i
			msg.type = 2
			msg.action = 0

			msg.scale.x = 0.4
			msg.scale.y = 0.4
			msg.scale.z = 0.05

			msg.color.a = 1.0
		 	msg.color.r = 1.0
		 	msg.color.g = 0.0
		 	msg.color.b = 1.0

			msg.lifetime = rospy.Duration.from_sec(1)

			totalX = 0
			totalY = 0

			for j in cluster:
				totalX += j.x
				totalY += j.y
			meanY = totalY / len(cluster)
			meanX = totalX / len(cluster)

			positions.append([meanX, meanY])

			msg.pose.position.x = meanX
			msg.pose.position.y = meanY
			msg.pose.position.z = 0.1

			msg.pose.orientation.w = 1

			visPublisher.publish(msg)

		msg = PoseArray()

		msg.header.frame_id = "base_radar_link";
		msg.header.stamp = originalMsg.header.stamp

		msg.poses = []

		for i in positions:

			pose = Pose()
			pose.position.x = i[0]
			pose.position.y = i[1]
			pose.position.z = 0.1

			pose.orientation.w = 1

			msg.poses.append(pose)

		publisher.publish(msg)

if __name__ == '__main__':
	print("Starting clustering")
	rospy.init_node('clustering')
	visPublisher = rospy.Publisher('/deep_radar/out/clusteringVis', msgTemplate.Marker, queue_size=0)
	publisher = rospy.Publisher('/deep_radar/out/clustering', PoseArray, queue_size=0)
	rospy.Subscriber("/deep_radar/out/points", msgTemplate.Marker, callback)
	rospy.spin()
