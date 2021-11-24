from skbuild import setup  # This line replaces 'from setuptools import setup'

setup(
    name="bob_navigation",
    version="0.0.1",
    description="A python interface for the BoB robotics navigation module",
    author='Alex Dewar',
    license="GPLv2",
    packages=['bob_robotics.navigation'],
    package_data = { 'bob_robotics.navigation': ['*.dll'] }
)
