from skbuild import setup  # This line replaces 'from setuptools import setup'

setup(
    name="bob_antworld",
    version="0.0.1",
    description="A python interface for the BoB robotics antworld module",
    author='Alex Dewar',
    license="GPLv2",
    packages=['bob_robotics.antworld'],
    package_data = { 'bob_robotics.antworld': ['*.dll'] }
)
