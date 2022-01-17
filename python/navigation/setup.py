from skbuild import setup  # This line replaces 'from setuptools import setup'
import sys
import os.path
sys.path.append(os.path.dirname(__file__) + "/..")

from build_common import get_git_version

setup(
    name="bob_navigation",
    version=get_git_version(),
    description="A python interface for the BoB robotics navigation module",
    author='Alex Dewar',
    license="GPLv2",
    packages=['bob_robotics.navigation'],
    package_data = { 'bob_robotics.navigation': ['*.dll'] }
)
