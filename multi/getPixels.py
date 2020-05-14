#!/usr/bin/env python

import rosbag
import sys
import cv2
import numpy as np

from cv_bridge import CvBridge

bridge = CvBridge()

fn = "/home/george/radar/rosbags/output2.bag"
pixelfn = "images/0-10.txt"

counter = 0
interval = 10

data = []
with open(pixelfn, "r") as f:
    data = f.read()
data = data.split("\n")
data = filter(None, data)
print(data)

newData = []

for topic, msg, t in rosbag.Bag(fn).read_messages():
    if "/camera/depth/image_rect_raw" in topic:

        if counter % interval != 0:
            counter += 1
            continue

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        img = cv_image[:,:].copy()

        for i in range(len(cv_image)):
            for j in range(len(cv_image[i])):
                img[i][j] = img[i][j]

        for line in data:
            if str(t) in line:
                line = line.split(" ")
                ax = line[1]
                ay = line[2]
                bx = line[3]
                by = line[4]

                if "a" in ax or "a" in bx:
                    continue
                ax = int(ax)
                ay = int(ay)
                bx = int(bx)
                by = int(by)
                #print(ax, ay, bx, by)

                az = img[ay][ax]
                bz = img[by][bx]

                print(az, bz)

                newData.append((str(t), str(ax), str(ay), str(az), str(bx), str(by), str(bz)))

                break

        counter += 1

with open("3d.txt", "w") as f:
    for i in newData:
        f.write(" ".join(i) + "\n")
