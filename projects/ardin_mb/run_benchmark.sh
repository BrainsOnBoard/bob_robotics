#!/bin/bash
#resolutions=("36,9,3,3" "36,10,4,2" "35,10,5,5")
resolutions=("36,9,3,3")
#orientations=("2" "3" "4")
#jitters=("1" "2" "4")
orientations=("3")
jitters=("0")

for r in ${resolutions[@]}; do
    for o in ${orientations[@]}; do
        for j in ${jitters[@]}; do
            IFS=',' read -ra r2 <<< "${r[@]}"
            
            fn="${r2[0]}x${r2[1]}_${r2[2]}x${r2[3]}x${o}_${j}cm.txt"
            rm -f $fn
            
            for f in ../../libantworld/ant*_route*.bin; do
                ./ardin_mb $f ${r2[0]} ${r2[1]} ${r2[2]} ${r2[3]} $o $j 2>> $fn
            done
        done
    done
done

#for x in 0 1 2 3 4 5 6 7 8 9; do
#rm -f test.txt
#
#    echo "$r($n)" >> test.txt
#    ./ardin_mb $r 2>> test.txt
#
