#!/usr/bin/env python3
from bob_robotics import navigation
import cv2
import sys

dbPath = sys.argv[1]
img = cv2.imread(dbPath + '/image_00002.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

algo = navigation.PerfectMemory(90, 45)
algo.train_route(dbPath)

print("angle: %g°" % algo.get_heading(img))
