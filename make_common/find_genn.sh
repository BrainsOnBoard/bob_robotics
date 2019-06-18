#!/bin/bash
# Find GeNN buildmodel command
GENN_BUILDMODEL=`which genn-buildmodel.sh`

# Get the parent of it's parent directory - the genn directory
GENN_PATH=$(dirname "$(dirname "$GENN_BUILDMODEL")")

# Echo it
echo $GENN_PATH
