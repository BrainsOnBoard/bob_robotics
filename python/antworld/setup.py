from skbuild import setup  # This line replaces 'from setuptools import setup'
import sys
import os.path
sys.path.append(os.path.dirname(__file__) + "/..")

from build_common import get_git_version

setup(
    name="bob_antworld",
    version=get_git_version(),
    description="A python interface for the BoB robotics antworld module",
    author='Alex Dewar',
    license="GPLv2",
    packages=['bob_robotics.antworld'],
    package_data = { 'bob_robotics.antworld': ['*.dll'] }
)
