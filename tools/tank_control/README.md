## Generic tool for controlling tank robots over the network
This tool opens a network connection for remote control on a tank robot. In
addition, it also checks for the presence of a joystick, in which case this can
also be used to control the robot, and, if a camera is present, this will also
be streamed over the network. If building for the Mindstorms robot, it will
additionally check for an IMU. As elsewhere, one can set the type of tank robot
to be used with the ``TANK_TYPE`` variable, e.g.:
```sh
mkdir build
cd build
cmake -DTANK_TYPE=EV3 ..
```

The idea is that we can remotely control robots without writing separate
programs depending on which peripherals we are using.
