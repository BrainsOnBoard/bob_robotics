import os, sys

# For loading image databases
sys.path.append(os.path.join(os.environ['BOB_ROBOTICS_PATH'], 'python'))
from BoBRobotics import image_database

database_path = '../../../tools/ant_world_db_creator/world5000_grid'
entries, metadata = image_database.load(database_path)
print('Database loaded')

grid_size = metadata['grid']['size']
grid_start = metadata['grid']['beginAtMM']
grid_separation = metadata['grid']['separationMM']

def mm2index(val, dim):
    return int((val + grid_start[dim]) // grid_separation[dim])

import cv2, numpy as np
ims = []
yis = list(range(mm2index(4000, 1), mm2index(6000, 1)))
xis = list(range(mm2index(4000, 0), mm2index(6000, 0)))
res = metadata['camera']['resolution']
ims = np.empty(([len(yis), len(xis), res[1], res[0]]))
for i in range(len(yis)):
    for j in range(len(xis)):
        im = cv2.imread(entries[(yis[i], xis[j], 0)]['filepath'])
        ims[i,j,:,:] = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
print('Images loaded')

refim = ims[len(yis) // 2, len(xis) // 2, :, :]
diffs = np.subtract(refim / 255, ims / 255)

xs = np.add(grid_start[0], np.multiply(xis, grid_separation[0]))
ys = np.add(grid_start[1], np.multiply(yis, grid_separation[1]))
ys, xs = np.meshgrid(ys, xs)

import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')

mean_diffs = np.mean(np.mean(diffs, axis=3), axis=2)
ax.plot_surface(xs, ys, mean_diffs)