#!/bin/bash

for kcToGGNWeight in $(seq 0.003 0.0002 0.004); do
    for ggnToKCWeight in $(seq -5 0.5 -5); do
        for ggnToKCVmid in $(seq -56 2 -56); do
            for ggnToKCVSlope in $(seq 1 0.25 1); do
                logFile=ggn_sweep/${kcToGGNWeight}_${ggnToKCWeight}_${ggnToKCVmid}_${ggnToKCVSlope}.csv
                echo $logFile
                ./ardin_mb --log $logFile --quit-after-train --kc-to-ggn-weight $kcToGGNWeight --ggn-to-kc-weight $ggnToKCWeight --ggn-to-kc-vmid $ggnToKCVmid --ggn-to-kc-vslope $ggnToKCVSlope $BOB_ROBOTICS_PATH/libantworld/ant1_route1.bin
            done
        done
    done
done
