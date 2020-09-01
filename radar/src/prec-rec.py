#!/usr/bin/env python
import rospy
import sys
from radar.msg import radar_fusion
from geometry_msgs.msg import PoseWithCovariance
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import PoseArray

latestGT = []

resultsD = {}
resultsS = {}
threshs = []
x = 0
while x <= 1.01:
    threshs.append(x)
    x += 0.001
for i in threshs:
    resultsD[i] = {"tp": 0, "fp": 0, "fn": 0}
    resultsS[i] = {"tp": 0, "fp": 0, "fn": 0}

def GTcallback(msg):
    global latestGT
    latestGT = msg.poses

def euclid(x, y, xx, yy):
    return ((x - xx)**2 + (y - yy)**2)**0.5

def Dcallback(msg):
    global latestGT, resultsD

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
                    resultsD[thresh]["tp"] += 1
                else:
                    resultsD[thresh]["fp"] += 1
            else:
                insideClust = False
                for gt in latestGT:
                    gtp = gt.position
                    d = euclid(point[0], point[1], gtp.x, gtp.y)
                    if d < 0.5:
                        insideClust = True
                if insideClust:
                    resultsD[thresh]["fn"] += 1

def Scallback(msg):
    global latestGT, resultsS

    #normalised points
    nPoints = []
    for point in msg.points:
        normProb = point.z
        if normProb < 0:
            normProb = 0
        elif normProb > 1:
            normProb = 1
        nPoints.append([point.x, point.y, normProb])

    tps = 0
    fps = 0
    fns = 0

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
                    resultsS[thresh]["tp"] += 1
                else:
                    resultsS[thresh]["fp"] += 1
            else:
                insideClust = False
                for gt in latestGT:
                    gtp = gt.position
                    d = euclid(point[0], point[1], gtp.x, gtp.y)
                    if d < 0.5:
                        insideClust = True
                if insideClust:
                    resultsS[thresh]["fn"] += 1

def listener():
    global resultsD, resultsS
    rospy.init_node('precrec')
    rospy.Subscriber("/radar_detector_ol/markers_prob", msgTemplate.Marker, Scallback)
    rospy.Subscriber("/deep_radar/out/points_raw", msgTemplate.Marker, Dcallback)
    rospy.Subscriber("/person/ground_truth", PoseArray, GTcallback)
    rospy.spin()
    resultsD = sorted(resultsD.items(), key=lambda x:x[0])
    resultsS = sorted(resultsS.items(), key=lambda x:x[0])

    with open("/home/george/tmp/d.txt", "w") as f:
        for i in resultsD:
            f.write(str(i) + "\n")

    with open("/home/george/tmp/s.txt", "w") as f:
        for i in resultsS:
            f.write(str(i) + "\n")

if __name__ == '__main__':
    listener()
