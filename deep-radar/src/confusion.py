#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
# from geometry_msgs.msg import Pose
from visualization_msgs import msg as msgTemplate

acceptedRadius = 0.5
cnnConfMat = {"tp": 0, "tn": 0, "fp": 0, "fn": 0}
svmConfMat = {"tp": 0, "tn": 0, "fp": 0, "fn": 0}
cnnPos = {"x": None, "y": None, "t": None}
svmPos = {"x": None, "y": None, "t": None}
totalCNN = 0
totalSVM = 0
tfListener = None

def gtCallback(msg):
	global cnnPos, svmPos, totalSVM, totalCNN

	#todo check time since pos (if cnn.t - current time < threshold)

	# now = rospy.get_rostime()
	# if now - maxTimeSinceLaser > meanPositionBuffer[3]:
	# 	print "Aborting, laser position is too old"
	# 	return

	# print msg

	pass

	# #cnn
	# diffX = abs(msg.position.x - cnnPos["x"])
	# diffY = abs(msg.position.y - cnnPos["y"])
	# hyp = ((diffX ** 2) + (diffY ** 2)) ** 0.5
	
	# #svm
	# diffX = abs(msg.position.x - svmPos["x"])
	# diffY = abs(msg.position.y - svmPos["y"])
	# hyp2 = ((diffX ** 2) + (diffY ** 2)) ** 0.5

	# totalCNN += hyp
	# totalSVM += hyp2

	# # if hyp < hyp2:
	# # 	totalCNN += 1
	# # elif hyp > hyp2:
	# # 	totalSVM += 1

	# print("{:5.3f}".format(hyp) + " - " + "{:5.3f}".format(hyp2) + " - " + "{:6.4f}".format(hyp2 - float(hyp)) + " - " + str(totalCNN) + ":" + str(totalSVM))
	# # print(msg.position)

def cnnCallback(msg):
	global cnnPos, tfListener

	transform = None
	try:
		transform = tfListener.lookupTransform('camera_depth_optical_frame', 'base_radar_link', rospy.Time(0))
	except:
		print("Aborting, unable to get tf transformation")
		return

    trans_mat = tf.transformations.translation_matrix(transform[0])
    rot_mat = tf.transformations.quaternion_matrix(transform[1])

	print(transform)

	#todo transform points
	cnnPos["x"] = msg.poses[0].position.x
	cnnPos["y"] = msg.poses[0].position.y
	t = str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs)
	cnnPos["t"] = t

def svmCallback(msg):
	global svmPos, tfListener

	#todo transform points
	svmPos["x"] = msg.poses[0].position.x
	svmPos["y"] = msg.poses[0].position.y
	t = str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs)
	svmPos["t"] = t

def listener():

	global tfListener

	rospy.init_node('confusion')
	tfListener = tf.TransformListener()
	rospy.Subscriber("/deep_radar/out/clustering", PoseArray, cnnCallback)
	rospy.Subscriber("/radar_detector_ol/poses", PoseArray, svmCallback)
	rospy.Subscriber("/person/ground_truth", PoseArray, gtCallback)
	rospy.spin()

if __name__ == '__main__': 

	listener()
