#!/usr/bin/env python3
from bob_robotics.navigation import InfoMax
import cv2
import sys
import matplotlib.pyplot as plt
import numpy as np

IM_SIZE=(90, 10)

dbPath = sys.argv[1]
img = cv2.imread(dbPath + '/image_00100.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Can also specify num_hidden here...
weights, seed = InfoMax.generate_initial_weights(IM_SIZE, seed=42)

# If the weights arg is omitted, random weights are used
algo = InfoMax(IM_SIZE, weights=weights)
algo.train_route(dbPath)

heading, minval, ridf = algo.get_ridf_data(img)
print("angle: %g°" % np.rad2deg(heading))

plt.plot(ridf)
plt.show()
