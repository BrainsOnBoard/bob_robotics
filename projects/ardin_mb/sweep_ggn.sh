#!/bin/bash

# Create output directory
mkdir -p ggn_sweep

# Sweep
for kcToGGNWeight in $(seq 0.016 0.001 0.020); do
    for ggnToKCWeight in $(seq -4 0.5 -4); do
        for ggnToKCVmid in $(seq -54.2 0.1 -54.2); do
            for ggnToKCVSlope in $(seq 1 0.25 1); do
                logFile=ggn_sweep/${kcToGGNWeight}_${ggnToKCWeight}_${ggnToKCVmid}_${ggnToKCVSlope}.csv
                echo $logFile
                ./ardin_mb --log $logFile --quit-after-train --kc-to-ggn-weight $kcToGGNWeight --ggn-to-kc-weight $ggnToKCWeight --ggn-to-kc-vmid $ggnToKCVmid --ggn-to-kc-vslope $ggnToKCVSlope $BOB_ROBOTICS_PATH/libantworld/ant1_route1.bin
            done
        done
    done
done
