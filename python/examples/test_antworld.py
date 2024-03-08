#!/usr/bin/env python3
import cv2
from bob_robotics import antworld

# Old Seville data (lower res, but loads faster)
# worldpath = antworld.bob_robotics_path + "/resources/antworld/world5000_gray.bin"
# z = 0.01 # m

# New Seville data
worldpath = antworld.bob_robotics_path + "/resources/antworld/seville_vegetation_downsampled.obj"
z = 1.5 # m (for some reason the ground is at ~1.5m for this world)

agent = antworld.Agent(720, 150, cubemap_size=512, near_clip=0.1)
(xlim, ylim, zlim) = agent.load_world(worldpath)
xstart = xlim[0] + (xlim[1] - xlim[0]) / 2.0
y = ylim[0] + (ylim[1] - ylim[0]) / 2.0

print("starting at (%f, %f, %f)" % (xstart, y, z))

fogit = [{"mode": "disabled"},
         {"mode": "linear", "start": 0.0, "end": 100.0, "colour": (0.75, 0.75, 0.75, 1.0)},
         {"mode": "exp", "density": 1.0, "colour": (0.75, 0.75, 0.75, 1.0)},
         {"mode": "exp", "density": 0.5, "colour": (0.75, 0.75, 0.75, 1.0)},
         {"mode": "exp2", "density": 1.0, "colour": (0.75, 0.75, 0.75, 1.0)},
         {"mode": "exp2", "density": 0.5, "colour": (0.75, 0.75, 0.75, 1.0)}]

for x, f in enumerate(fogit):
    agent.set_position(xstart + (x * 0.5), y, z)
    agent.set_fog(**f)
    im = agent.read_frame()
    filename = "antworld%i.png" % x
    print("Saving image as %s..." % filename)
    cv2.imwrite(filename, im)
