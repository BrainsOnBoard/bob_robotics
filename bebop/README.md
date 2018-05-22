### Bebop module
This folder contains the code for both controlling Bebop 2 drones (i.e. sending move commands) and streaming the video feed from it. These two things are split into two files to allow for compiling just the controller code by itself (and thus not having to install the extra dependencies for the video code). Some of the video code is taken from [the Bebop ROS module](https://github.com/AutonomyLab/bebop_autonomy).

One can build the module as follows:
```sh
# controller code only
make bebop.o

# video code only
make video.o

# both controller and video code
make
```

The idea is that this makefile can be called by the makefile for a project that requires the module, i.e.:
```Makefile
bebop_module:
	make -C $(GENN_ROBOTICS_PATH)/bebop
```

## Building Parrot's SDK
This module requires that Parrot's SDK is installed and that the ARSDK_ROOT environment variable points to the folder where it was downloaded.

These build instructions are partly taken from [Parrot's website](http://developer.parrot.com/docs/SDK3).

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

# add ARSDK_ROOT environment variable to your .bashrc
echo export ARSDK_ROOT=$PWD >> ~/.bashrc
source ~/.bashrc
```

