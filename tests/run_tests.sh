#!/bin/sh
# Build and run all unit tests

make && ./tests
exit $?
