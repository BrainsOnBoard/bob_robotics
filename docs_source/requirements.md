# Build requirements
This file lists the build requirements for different BoB robotics modules and sample projects.

## Compiler
On Linux, the earliest supported version of gcc is 5.x. For Windows, Visual Studio 2015 or newer is required.

## Environment variables
The ``BOB_ROBOTICS_PATH`` environment variable is used to help programs built with BoB robotics modules to find where this repository is on your machine. (Currently this is only needed so that the panoramic unwrapping code can find [the unwrapping parameters for different types of camera](../imgproc/unwrapparams).)

On Linux and Mac, you can have this variable automatically set by adding it to your .profile file as follows:
```sh
echo export BOB_ROBOTICS_PATH=/path/to/bob_robotics >> $HOME/.profile
```

On Windows, you can set environment variables by typing 'environment variables' into your start menu and following the instructions.

## GeNN
You must use version 3.X or later of GeNN. Stable releases of GeNN can be downloaded from https://github.com/genn-team/genn/releases. However some models in this repository may require the lastest version from the Github repository: https://github.com/genn-team/genn. It should also be passed to genn-buildmodel using the -i command line argument when building networks using the models in the [genn\_models](../genn_models) folder i.e. ``genn-buildmodel.sh -i $BOB_ROBOTICS_PATH`` on \*nix or ``genn-buildmodel.bat -i %BOB_ROBOTICS_PATH%`` on Windows.

## OpenCV
OpenCV v3+ is required for most BoB robotics projects. Note that the ``libopencv`` package in Ubuntu 16.04 is only version 2.4, so you will have to build from source or find a working OpenCV v3 PPA. Also note that the OpenCV package in Ubuntu is not compiled with CUDA support (this is required for [ant\_world](../ant_world)), so if you want this feature you will have to [build it from source](https://docs.opencv.org/3.4/d7/d9f/tutorial\_linux\_install.html).

On Ubuntu 18.04, you can install OpenCV with:
```sh
sudo apt install libopencv-dev
```

Installation instructions for NVIDIA Jetson TX1 can be found [here](https://devtalk.nvidia.com/default/topic/965134/opencv-3-1-compilation-on-tx1-lets-collect-the-quot-definitive-quot-cmake-settings-).

## OpenGL
[Libantworld](../libantworld) and the [ant\_world\_test](../examples/ant_world_test) example require OpenGL, GLEW and SFML.

These can be installed on Ubuntu with:
```sh
sudo apt install libglew-dev libsfml-dev
```

## Linux headers
Header files for the currently running kernel are required for the [I2C interface code](https://github.com/BrainsOnBoard/bob_robotics/blob/master/common/i2c_interface.h) and the [See3CAM\_CU40 module](https://github.com/BrainsOnBoard/bob_robotics/blob/master/video/see3cam_cu40.h).

These can be installed on Ubuntu with:
```sh
sudo apt install linux-headers-$(uname -r)
```

## libi2c
The I2C interface requires Linux headers (as mentioned above) and for Ubuntu 18.04 ``libi2c`` is additionally required (but not Ubuntu 16.04, for some reason).

Install with:
```sh
sudo apt install libi2c-dev
```

## Parrot's SDK (ARSDK)
This module requires that Parrot's SDK is installed and that the ``ARSDK_ROOT`` environment variable points to the folder where it was downloaded.

There are build instructions below (adapted from [Parrot's website](http://developer.parrot.com/docs/SDK3)).

```sh
# install dependencies
sudo apt install repo git build-essential autoconf libtool python python3 libavahi-client-dev libavcodec-dev libavformat-dev libswscale-dev libavutil-dev zlib1g-dev

# download SDK sources
mkdir arsdk
cd arsdk
repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git -m release.xml
repo sync

# build SDK
./build.sh -p arsdk-native -t build-sdk -j

# add ARSDK_ROOT environment variable to your .profile
echo export ARSDK_ROOT=$PWD >> ~/.profile
source ~/.profile
```
