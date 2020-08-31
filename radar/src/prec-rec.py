#!/usr/bin/env python
import rospy
import sys
from radar.msg import radar_fusion
from geometry_msgs.msg import PoseWithCovariance
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import PoseArray

latestGT = []

results = {}
threshs = []
x = 0
while x <= 1.01:
    threshs.append(x)
    x += 0.001
for i in threshs:
    results[i] = {"tp": 0, "fp": 0, "fn": 0}

def GTcallback(msg):
    global latestGT
    latestGT = msg.poses

def euclid(x, y, xx, yy):
    return ((x - xx)**2 + (y - yy)**2)**0.5

def Dcallback(msg):
    global latestGT, results

    minProb = 999999999
    maxProb = -999999999
    for point in msg.points:
        if point.z < minProb:
            minProb = point.z
        if point.z > maxProb:
            maxProb = point.z

    #normalised points
    nPoints = []
    for point in msg.points:
        #normProb = (point.z - minProb) / (maxProb - minProb)
        normProb = (point.z + 5) / 10
        if normProb < 0:
            normProb = 0
        elif normProb > 1:
            normProb = 1
        nPoints.append([point.x, point.y, normProb])

    tps = 0
    fps = 0
    fns = 0

#    for thresh in threshs:
#        for gt in latestGT:
#            gtp = gt.position
#            fn = True
#            for point in nPoints:
#                d = euclid(point[0], point[1], gtp.x, gtp.y)
#                if d < 0.5:
#                    if point[2] > thresh:
#                        fn = False
#            if fn == True:
#                results[thresh]["fn"] += 1

    for thresh in threshs:
        for point in nPoints:
            if point[2] > thresh:
                insideClust = False
                for gt in latestGT:
                    gtp = gt.position
                    d = euclid(point[0], point[1], gtp.x, gtp.y)
                    if d < 0.5:
                        insideClust = True
                if insideClust:
                    results[thresh]["tp"] += 1
                else:
                    results[thresh]["fp"] += 1
            else:
                insideClust = False
                for gt in latestGT:
                    gtp = gt.position
                    d = euclid(point[0], point[1], gtp.x, gtp.y)
                    if d < 0.5:
                        insideClust = True
                if insideClust:
                    results[thresh]["fn"] += 1

def listener():
    global results
    rospy.init_node('prec-rec')
    #rospy.Subscriber("/radar_detector_ol/markers_prob", msgTemplate.Marker, Dcallback)
    rospy.Subscriber("/deep_radar/out/points_raw", msgTemplate.Marker, Dcallback)
    rospy.Subscriber("/person/ground_truth", PoseArray, GTcallback)
    rospy.spin()
    results = sorted(results.items(), key=lambda x:x[0])
    for i in results:
        print(i)

if __name__ == '__main__':
    listener()
