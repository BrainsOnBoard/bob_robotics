#!/bin/sh
# A script to check whether this machine is running
# Ubuntu 16.04 or older.

# Check for lsb_release program
command -v lsb_release > /dev/null 2>&1 || exit 1

# Check we're running on Ubuntu
if [ ! $(lsb_release -is) = Ubuntu ]; then exit 1; fi

# Check we're running on Ubuntu 16.04 or earlier
if [ $(lsb_release -rs | sed 's/\.//') -gt 1604 ]; then exit 1; fi
