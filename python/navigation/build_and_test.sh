#!/bin/sh

set -e

# We use poetry for managing dependencies
python -m pip install poetry
poetry install

# Build package
poetry build

# Run tests
poetry run pytest -v --junitxml=../../pytest_test_results.xml
