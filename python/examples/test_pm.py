#!/usr/bin/env python3
from bob_robotics import navigation
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys

dbPath = sys.argv[1]
img = cv2.imread(dbPath + '/image_00100.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print('Loaded image')

algo = navigation.PerfectMemory((180, 45))
algo.train_route(dbPath)
print('Training complete')

t0 = time()
ang = algo.get_heading(img)
print(time() - t0)

# Alternatively you can use algo.get_heading(img) to just getting the heading,
# which should be faster
(heading, best_snap, best_min, diffs) = algo.get_ridf_data(img)

print('Best snapshot: %d at %g° (diff: %g)' % (best_snap, np.rad2deg(heading), best_min))

xs = np.linspace(0, 360, diffs.shape[1])
ridf = diffs[best_snap, :]
plt.plot(xs, ridf / 255.0)
plt.show()
