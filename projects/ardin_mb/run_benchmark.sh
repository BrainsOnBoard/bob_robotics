#!/bin/bash

rm -f test.txt
for r in ../../libantworld/ant*_route*.bin; do
    echo "$r($n)" >> test.txt
    ./ardin_mb --auto-test $r 2>> test.txt
done
