#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from visualization_msgs import msg as msgTemplate

acceptedRadius = 0.5
confusionMatrix = {"tp": 0, "tn": 0, "fp": 0, "fn": 0}

def gtCallback(msg):

	gtX = 
	gtY = 

	

def cnnCallback(msg):

	pass

	# print("=====")
	# for i in msg.poses:
	# 	print(i.position)

def svmCallback(msg):

	pass

def listener():
	rospy.init_node('confusion')
	rospy.Subscriber("/deep_radar/out/clustering", PoseArray, cnnCallback)
	rospy.Subscriber("/deep_radar/out/clustering", PoseArray, svmCallback)
	rospy.Subscriber("/camera/ground_truth", msgTemplate.Marker, gtCallback)
	rospy.spin()

if __name__ == '__main__': 

	listener()
