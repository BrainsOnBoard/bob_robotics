#!/bin/sh

set -e

# Make virtual environment
rm -rf virtualenv
python -m venv virtualenv
. virtualenv/bin/activate

# Install necessary packages, including BoB navigation module
pip install wheel numpy scikit-build numpy pandas pytest
cd "$(dirname $0)"
python setup.py build install

# Run tests
pytest -v --junitxml=../../pytest_test_results.xml
