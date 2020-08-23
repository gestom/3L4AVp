#!/usr/bin/env python

import rosbag
import sys
import cv2
import numpy as np

from cv_bridge import CvBridge

bridge = CvBridge()

fn = "/home/george/radar/rosbags/output2.bag"

counter = 0
interval = 10

for topic, msg, t in rosbag.Bag(fn).read_messages():
    if "/camera/depth/image_rect_raw" in topic:

        if counter % interval != 0:
            counter += 1
            continue

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        img = cv_image[:,:].copy()

        for i in range(len(cv_image)):
            for j in range(len(cv_image[i])):
                img[i][j] = img[i][j] / 20.

        cv2.imwrite("images/0-10/" + str(t) + ".jpg", img) 
        counter += 1
