#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

publisher = None

lastPoses = [[-9999, -9999], [-9999, -9999]]

def callback(msg):
    global lastPoses, publisher

    print("msg recvd %i" % (len(msg.poses)))

    out = PoseArray()
    out.header.frame_id = "base_radar_link"
    out.header.stamp = rospy.Time.now()
    out.poses = []

    if len(msg.poses) == 0:
        publisher.publish(out)
        return

    entities = 1
    if len(msg.poses) == 2:
        entities = 2
    elif lastPoses[-1][-1] != -9999:
        entities = 2

    for i in range(entities):
        p = Pose()
        p.position.x = lastPoses[i][0]
        p.position.y = lastPoses[i][1]
        p.position.z = 0.1
        p.orientation.w = 1
        out.poses.append(p)

    closestIdx = -1
    closestDist = 99999
    closestTarget = -1

    for pose in range(len(msg.poses)):
        for target in range(len(lastPoses)):
            dx = msg.poses[pose].position.x - lastPoses[target][0]
            dy = msg.poses[pose].position.y - lastPoses[target][1]
            dist = (dx*dx+dy*dy)**0.5
            if dist < closestDist:
                closestDist = dist
                closestIdx = pose
                closestTarget = target

    print(closestDist, closestIdx, closestTarget)

    out.poses[closestTarget].position.x = msg.poses[closestIdx].position.x
    out.poses[closestTarget].position.y = msg.poses[closestIdx].position.y
    lastPoses[closestTarget][0] = msg.poses[closestIdx].position.x
    lastPoses[closestTarget][1] = msg.poses[closestIdx].position.y

    if len(msg.poses) == 1:
       publisher.publish(out) 
       return

    print("before")
    print(out.poses)

    NclosestIdx = -1
    NclosestDist = 9999
    NclosestTarget = -1

    for pose in range(len(msg.poses)):
        print(pose, closestIdx)
        if pose == closestIdx:
            print("contd")
            continue
        for target in range(len(lastPoses)):
            if target == closestTarget:
                continue
            dx = msg.poses[pose].position.x - lastPoses[target][0]
            dy = msg.poses[pose].position.y - lastPoses[target][1]
            dist = (dx*dx+dy*dy)**0.5
            if dist <= NclosestDist:
                NclosestDist = dist
                NclosestIdx = pose
                NclosestTarget = target

    print(closestDist, closestIdx, closestTarget)
    out.poses[NclosestTarget].position.x = msg.poses[NclosestIdx].position.x
    out.poses[NclosestTarget].position.y = msg.poses[NclosestIdx].position.y
    lastPoses[NclosestTarget][0] = msg.poses[NclosestIdx].position.x
    lastPoses[NclosestTarget][1] = msg.poses[NclosestIdx].position.y

    print("after")
    print(out.poses)

    publisher.publish(out)

def listener():
    global publisher
    rospy.init_node('ptt')
    rospy.Subscriber("/deep_radar/out/clustering", PoseArray, callback)
    publisher = rospy.Publisher('/ptt', PoseArray, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
