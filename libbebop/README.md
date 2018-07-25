## Default drone joystick controls
* A button: take off
* B button: land
* Left stick (vertical): up/down
* Right stick: backwards/forwards + left/right
* Left trigger: rotate anti-clockwise
* Right trigger: rotate clockwise

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
