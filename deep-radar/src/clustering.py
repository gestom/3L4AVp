import numpy as np
from sklearn.cluster import KMeans
import cv2
import os
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point
import math

import random

clusterDistance = 1 
minClusterPoints = 4

publisher = None

def callback(msg):

	if msg.id != 0:

		clusters = []
		toSearch = msg.points

		while len(toSearch):

			pointIndex = random.randint(0, len(toSearch) - 1)
			point = toSearch[pointIndex]
			del toSearch[pointIndex]
			newCluster = [point]
			toDelete = []

			for i in xrange(0, len(toSearch)):

				distance = math.sqrt((toSearch[i].x ** 2) + (toSearch[i].y ** 2))

				if distance > clusterDistance:
					continue

				newCluster.append(toSearch[i])
				toDelete.append(i)

			toDelete = list(reversed(toDelete))

			if len(newCluster) >= minClusterPoints:
				clusters.append(newCluster)
				for i in toDelete:
					del toSearch[i]

		print "Found %i clusters" % (len(clusters))

		for i in xrange(0, len(clusters)):

			msg = msgTemplate.Marker()

			msg.header.frame_id = "base_radar_link";
			msg.header.stamp = rospy.Time.now();

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
			for j in clusters[i]:
				totalX += j.x
				totalY += j.y
			print totalX, totalY
			meanX = totalX / len(clusters[i])
			meanY = totalY / len(clusters[i])
			print meanX, meanY

			msg.pose.position.x = meanX
			msg.pose.position.y = meanY
			msg.pose.position.z = 0
			# msg.points = [Point(x=meanX, y=meanY, z=0)]

			publisher.publish(msg)

if __name__ == '__main__':
	print("Starting clustering")
	rospy.init_node('clustering')
	publisher = rospy.Publisher('/deep_radar/out/clustering', msgTemplate.Marker, queue_size=0)
	rospy.Subscriber("/deep_radar/out/points", msgTemplate.Marker, callback)
	rospy.spin()
