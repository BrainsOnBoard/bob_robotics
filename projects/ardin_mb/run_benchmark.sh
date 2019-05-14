#!/bin/bash

rm -f test.txt
for r in $(dirname "$0")/../../resources/antworld/ant*_route*.bin; do
    echo "$r($n)" >> test.txt
    ./ardin_mb $r 2>> test.txt
done
