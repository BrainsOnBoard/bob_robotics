import os
import random
import sys

from skbuild import setup

# The ranges library appears not to work with MSVC unless experimental C++20
# features are enabled. (Tested with Visual Studio 2019.)
cmake_args = []
if os.name == 'nt':
    cmake_args.append("-DCMAKE_CXX_STANDARD=20")

setup(
    name="bob_navigation",
    version="1.0",
    description="A python interface for the BoB robotics navigation module",
    author='Alex Dewar',
    license="GPLv2",
    packages=['bob_robotics.navigation'],
    package_data = { 'bob_robotics.navigation': ['*.dll'] },
    cmake_args = cmake_args)
