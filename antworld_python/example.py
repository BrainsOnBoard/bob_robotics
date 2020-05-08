#!/usr/bin/env python3
import antworld, cv2

# Old Seville data (lower res, but loads faster)
# worldpath = antworld.bob_robotics_path + "/resources/antworld/world5000_gray.bin"
# z = 0.01 # m

# New Seville data
worldpath = antworld.bob_robotics_path + "/resources/antworld/seville_vegetation_downsampled.obj"
z = 1.5 # m (for some reason the "ground is at ~1.5m for this world")

agent = antworld.Agent(720, 150)
(xlim, ylim, zlim) = agent.load_world(worldpath)
xstart = xlim[0] + (xlim[1] - xlim[0]) / 2.0
y = ylim[0] + (ylim[1] - ylim[0]) / 2.0

print("starting at (%f, %f, %f)" % (xstart, y, z))

for x in range(3):
    agent.set_position(x + xstart, y, z)
    im = agent.read_frame_greyscale()
    cv2.imshow("Ant world (OpenCV)", im)
    cv2.waitKey(500)
