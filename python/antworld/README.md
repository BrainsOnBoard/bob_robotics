# AntWorld Python module
This folder contains the source for a python module which wraps the basic
functionality of the BoB robotics [AntWorld module](https://brainsonboard.github.io/bob_robotics/namespaceBoBRobotics_1_1AntWorld.html).

Example usage is given in the file `test_antworld.py`.

# Building
Compilation requires [scikit-build](https://pypi.org/project/scikit-build).

Build as usual:
``sh
python setup.py bdist_wheel
``

**NOTE**: Currently this module needs to be packaged manually.
