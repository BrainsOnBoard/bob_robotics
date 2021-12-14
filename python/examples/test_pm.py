#!/usr/bin/env python3
from bob_robotics import navigation
import cv2
import sys

dbPath = sys.argv[1]
img = cv2.imread(dbPath + '/image_00100.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print('Loaded image')

algo = navigation.PerfectMemory((180, 45))
algo.train_route(dbPath)
print('Training complete')

print("angle: %g°" % algo.get_heading(img))
