#!/usr/bin/env python
import rospy
import sys
from radar.msg import radar_fusion
from geometry_msgs.msg import PoseWithCovariance

publisher = None

lastLeg = None
lastSVM = None
lastPN = None

def swap(msg, gt, prev, debug):

    if len(gt) == 1:

        #experiment 1 or 2

        if len(msg) < 2:
            return msg

        else:
            #swap to minimise vs gt

            closestCluster = 999999.9
            bestArrIdx = -1

            for arrIdx in range(len(msg)):
                x1 = msg[arrIdx].pose.position.x
                y1 = msg[arrIdx].pose.position.y
                x2 = gt[0].pose.position.x
                y2 = gt[0].pose.position.y
                distance = ((x1-x2)**2 + (y1-y2)**2)**0.5
                if distance < closestCluster:
                    closestCluster = distance
                    bestArrIdx = arrIdx

            if bestArrIdx == 0:
                return msg
            else:
                tmp = msg[0]
                msg[0] = msg[bestArrIdx]
                msg[bestArrIdx] = tmp
                return msg

    else:
        
        #experiment 3

        #swap order to min dist to gt
        if len(msg) == 0:
            return msg
        elif len(msg) == 1:

            if len(prev) == 1:
                return msg

            closestCluster = 999999.9
            bestGTIdx = -1

            for arrIdx in range(len(gt)):
                x1 = msg[0].pose.position.x
                y1 = msg[0].pose.position.y
                x2 = gt[arrIdx].pose.position.x
                y2 = gt[arrIdx].pose.position.y
                distance = ((x1-x2)**2 + (y1-y2)**2)**0.5
                if distance < closestCluster:
                    closestCluster = distance
                    bestGTIdx = arrIdx

            if bestGTIdx == 0:
                msg.append(prev[1])
                return msg
            else:
                msg.append(msg[0])
                msg[0] = prev[0]
                return msg
        else:
 
            bestCluster = 999999.9
            bestArrIdx = -1
            bestGTIdx = -1
            print("===========================")
            print("Start:")
            print(debug)
            for arrIdx in range(len(msg)):
                for gtIdx in range(len(gt)):
                    x1 = msg[arrIdx].pose.position.x
                    y1 = msg[arrIdx].pose.position.y
                    x2 = gt[gtIdx].pose.position.x
                    y2 = gt[gtIdx].pose.position.y
                    distance = ((x1-x2)**2 + (y1-y2)**2)**0.5
                    print(gtIdx, arrIdx, distance)
                    if distance < bestCluster:
                        bestCluster = distance
                        bestArrIdx = arrIdx
                        bestGTIdx = gtIdx

            tmp = msg[bestGTIdx]
            msg[bestGTIdx] = msg[bestArrIdx]
            msg[bestArrIdx] = tmp

            print("Best:")
            print(bestGTIdx, bestArrIdx, bestCluster)

            secondBestCluster = 999999.9
            secondBestArrIdx = -1
            secondBestGTIdx = -1

            for arrIdx in range(len(msg)):
                if arrIdx == bestGTIdx:
                    continue
                for gtIdx in range(len(gt)):
                    if gtIdx == bestGTIdx:
                        continue
                    x1 = msg[arrIdx].pose.position.x
                    y1 = msg[arrIdx].pose.position.y
                    x2 = gt[gtIdx].pose.position.x
                    y2 = gt[gtIdx].pose.position.y
                    distance = ((x1-x2)**2 + (y1-y2)**2)**0.5
                    if distance < secondBestCluster:
                        secondBestCluster = distance
                        secondBestArrIdx = arrIdx
                        secondBestGTIdx = gtIdx

            print(msg)
            tmp = msg[secondBestGTIdx]
            msg[secondBestGTIdx] = msg[secondBestArrIdx]
            msg[secondBestArrIdx] = tmp

            print(msg)

            return msg

def callback(msg):
    global publisher, lastLeg, lastSVM, lastPN

    msg.leg = swap(msg.leg, msg.gt, lastLeg, "las")
    msg.rad = swap(msg.rad, msg.gt, lastSVM, "svm")
    msg.deep = swap(msg.deep, msg.gt, lastPN, "deep")

    lastLeg = msg.leg
    lastSVM = msg.rad
    lastPN = msg.deep

    publisher.publish(msg)

def listener():
    global publisher
    rospy.init_node('remuxer')
    rospy.Subscriber("/evaluator_mux", radar_fusion, callback)
    publisher = rospy.Publisher("/evaluator_remux", radar_fusion, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
