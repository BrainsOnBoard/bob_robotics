#!/usr/bin/env python3
from bob_robotics import navigation
import cv2
import sys

dbPath = sys.argv[1]
img = cv2.imread(dbPath + '/image_00002.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

pm = navigation.PerfectMemory(*img.shape)
pm.train_route(dbPath)

print("angle: %g°" % pm.get_heading(img))
