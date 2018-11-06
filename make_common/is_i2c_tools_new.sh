#!/bin/bash

# Functions for comparing program versions, after: https://stackoverflow.com/questions/4023830/how-to-compare-two-strings-in-dot-separated-version-format-in-bash
verlte() {
    [  "$1" = "`echo -e \"$1\n$2\" | sort -V | head -n1`" ]
}

verlt() {
    [ "$1" = "$2" ] && return 1 || verlte $1 $2
}

# Give an error code if i2c-tools version <4.0
i2cver=$(i2cdetect -V |& awk '{ print $3 }')
if verlt $i2cver 4.0; then exit 1; fi
