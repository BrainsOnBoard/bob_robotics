[![Build Status](https://gen-ci.inf.sussex.ac.uk/buildStatus/icon?job=BoB%20robotics/bob_robotics/master)](https://gen-ci.inf.sussex.ac.uk/job/BoB%20robotics/job/bob_robotics/job/master/)
# BoB robotics
This repository contains code for interfacing with various robot platforms and other associated hardware, as well as code for running simulations and visualising data.

Fuller documentation is available at https://brainsonboard.github.io/bob_robotics/.

**Note** this repository is public so should only be used for utility code and example models - larger models should go in their own repositories and only be made public upon publication.

## Prerequistites
A full list of requirements for different projects can be found [in the documentation](https://github.com/BrainsOnBoard/bob_robotics/documentation/requirements.md).

The main requirements are as follows:
* GeNN 3.X - Stable releases of GeNN can be downloaded from https://github.com/genn-team/genn/releases. However some models in this repository may require the lastest version from the Github repository: https://github.com/genn-team/genn. It should also be passed to genn-buildmodel using the -i command line argument when building networks using the models in the [genn\_models](genn_models) folder i.e. ``genn-buildmodel.sh -i $BOB_ROBOTICS_PATH`` on \*nix or ``genn-buildmodel.bat -i %BOB_ROBOTICS_PATH%`` on Windows.
* Open CV 3.X - OpenCV releases can be downloaded from https://opencv.org. Installation instructions for Jetson TX1 can be found [here](https://devtalk.nvidia.com/default/topic/965134/opencv-3-1-compilation-on-tx1-lets-collect-the-quot-definitive-quot-cmake-settings-).

## Examples
Example projects covering most of the capabilities of BoB robotics can be found [in the examples folder](examples).
