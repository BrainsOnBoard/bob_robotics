## Robot test programs
This folder contains a basic example for connecting to an Arduino robot (Nor-bot :-)), controlling it with a joystick and viewing the camera stream. Whether the stream is unwrapped can be toggled by pressing U. Press escape to exit the program with an inglorious flurry of uncaught exceptions ;-)

### Building on Linux
The example programs can be built on Linux with a simple `make`. If you want to build the robot-side program without I2C (e.g. if you're testing it on a desktop machine), build with:
```sh
NO_I2C_ROBOT=1 make
```

### Building on Windows
The programs are available in Visual Studio (2017) projects. Simply clone the `GeNN_Robotics` repository, then open `GeNN_Robotics.sln` in Visual Studio.

### Running
The program to be run on the robot is invoked like so:
```sh
./robot
```

The program for the computer is run on Linux as:
```sh
./computer $ROBOT_IP_ADDRESS
```
The commandline argument is optional.

You can run the Windows version through Visual Studio.
