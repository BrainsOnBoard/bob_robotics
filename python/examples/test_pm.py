#!/usr/bin/env python3
# -*- coding: utf8 -*-
import bob_robotics.navigation as bobnav
from bob_robotics.navigation import imgproc as ip
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys
from time import perf_counter

PREPROC = ip.resize(45, 180)

db = bobnav.Database(sys.argv[1])
img = db.read_images(100, preprocess=PREPROC)

train_idx = range(0, len(db), 10)  # Use every 10th image
train_images = db.read_images(train_idx, preprocess=PREPROC)
algo = bobnav.PerfectMemory(train_images=train_images)
print('Training complete')

t0 = perf_counter()
df = algo.ridf(img)
print(perf_counter() - t0)

print('Best snapshot: %d at %g° (diff: %g)' % (df.best_snap, np.rad2deg(df.estimated_dheading), df.minval))

diffs = df.differences
xs = np.linspace(0, 360, diffs.shape[1])
ridf = diffs[df.best_snap, :]
plt.plot(xs, ridf / 255.0)
plt.show()
