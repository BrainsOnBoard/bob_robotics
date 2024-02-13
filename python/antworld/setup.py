from skbuild import setup  # This line replaces 'from setuptools import setup'
import sys
import os.path

setup(
    name="bob_antworld",
    version="1.0",
    description="A python interface for the BoB robotics antworld module",
    author='Alex Dewar',
    license="GPLv2",
    packages=['bob_robotics.antworld'],
    package_data = { 'bob_robotics.antworld': ['*.dll'] }
)
