#!/usr/bin/env python
import random
import rospy
import math
import os
import h5py
import sys
import socket

import numpy as np
import tf as rostf
import tensorflow as tf

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from visualization_msgs import msg as msgTemplate
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

pointsPublisher = None

def radarCallback(msg):
	global pointsPublisher
	
	points = []

	point_generator = pc2.read_points(msg)
	for point in point_generator:		
		points.append(point)

	#publish points

	msg = msgTemplate.Marker()

	msg.header.frame_id = "base_radar_link";
	msg.header.stamp = rospy.Time.now();

	msg.ns = "points"
	msg.id = 1
	msg.type = 7
	msg.action = 0

	msg.scale.x = 0.1
	msg.scale.y = 0.1
	msg.scale.z = 0.1

	msg.color.a = 1.0
	msg.color.r = 1.0
	msg.color.g = 1.0
	msg.color.b = 1.0

	msg.lifetime = rospy.Duration.from_sec(1)

	msg.points = []
	for j in points:
		msg.points.append(Point(x = j[0], y = j[1], z = j[2]))

	pointsPublisher.publish(msg)


if __name__ == '__main__':

	rospy.init_node('clusteringPass', anonymous=True)
	rospy.Subscriber("/radar/RScan", PointCloud2, radarCallback)
	pointsPublisher = rospy.Publisher('/deep_radar/out/clusterExpPoints', msgTemplate.Marker, queue_size=0)
	rospy.spin()
