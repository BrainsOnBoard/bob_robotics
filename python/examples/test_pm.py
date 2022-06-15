#!/usr/bin/env python3
import bob_robotics.navigation as bobnav
from bob_robotics.navigation import imgproc as ip
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys
from time import perf_counter

IM_SIZE = (45, 180)
PREPROC = ip.resize(*IM_SIZE)

# dbPath = sys.argv[1]
dbPath = "/home/alex/code/datasets/ant_world/routes/ant1_route1"
img = cv2.imread(dbPath + '/image_00100.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print('Loaded image')

db = bobnav.Database(dbPath)
train_entries = db.read_image_entries(range(20), preprocess=PREPROC)
test_entry = db.read_image_entries(100, preprocess=PREPROC)

algo = bobnav.PerfectMemory(IM_SIZE)
algo.train(train_entries)
print('Training complete')

t0 = perf_counter()
# (heading, best_snap, best_min, diffs) = algo.ridf(img)
df = algo.ridf(img)
print(perf_counter() - t0)

print('Best snapshot: %d at %g° (diff: %g)' % (df.best_snap, np.rad2deg(df.estimated_heading), df.min))

diffs = df.differences
xs = np.linspace(0, 360, diffs.shape[1])
ridf = diffs[best_snap, :]
plt.plot(xs, ridf / 255.0)
plt.show()
