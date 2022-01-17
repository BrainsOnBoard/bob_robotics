#!/usr/bin/env python3
from bob_robotics import navigation
import cv2
import sys
import matplotlib.pyplot as plt
import numpy as np

dbPath = sys.argv[1]
img = cv2.imread(dbPath + '/image_00100.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print('Loaded image')

algo = navigation.InfoMax((90, 10))
algo.train_route(dbPath)

heading, minval, ridf = algo.get_ridf_data(img)
print("angle: %g°" % np.rad2deg(heading))

plt.plot(ridf)
plt.show()
