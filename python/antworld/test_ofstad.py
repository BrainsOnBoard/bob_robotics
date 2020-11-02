#!/usr/bin/env python3
import antworld, cv2

# All coordinates are in metres

# Path to Ofstad arena -- replace with correct path
worldpath = antworld.bob_robotics_path + "/resources/ofstad/ofstad.obj"
z = 0

agent = antworld.Agent(720, 150)
agent.set_attitude(90, 0, 0) # yaw, pitch, roll in degrees
(xlim, ylim, zlim) = agent.load_world(worldpath)
xstart = xlim[0] + (xlim[1] - xlim[0]) / 2.0
y = ylim[0] + (ylim[1] - ylim[0]) / 2.0
step = 0.05 # m

print("starting at (%f, %f, %f)" % (xstart, y, z))

for x in range(3):
    agent.set_position(x * step + xstart, y, z)
    im = agent.read_frame()
    filename = "ofstad%i.png" % x
    print("Saving image as %s..." % filename)
    cv2.imwrite(filename, im)
