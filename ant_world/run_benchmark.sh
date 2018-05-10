#!/bin/bash

rm -f test.txt
for r in ../libantworld/ant*_route*.bin; do
    echo "$r($n)" >> test.txt
    ./ant_world $r 2>> test.txt
done