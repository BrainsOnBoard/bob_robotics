import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.signal import medfilt

FILTER_SIZE = 3

def normalise180(ths):
    if np.isscalar(ths):
        return normalise180([ths])[0]

    ths = np.array(ths) % 360
    ths[ths > 180] -= 360
    return ths

def processData(ths):
    ths = medfilt(ths, FILTER_SIZE)
    return normalise180(ths - np.median(ths))

df = pd.read_csv(os.path.join(os.path.dirname(__file__), 'data_ramp.csv'))

t = df['Time'] / 1000
vicon = np.array([df['ViconAng1'], df['ViconAng2'], df['ViconAng3']])
imu = np.array([df['ImuAng1'], df['ImuAng2'], df['ImuAng3']])

# Pitch axis goes in the other direction for the Vicon system!
vicon[1,:] = -vicon[1,:]

_, axes = plt.subplots(3, figsize=[20, 10])
for i, ax in enumerate(axes):
    ax.plot(t, processData(imu[i,:]), t, processData(vicon[i,:]))

    ax.set_xlim(0, np.array(t)[-1])
    ax.set_ylim(-180, 180)
    ax.set_yticks(range(-180, 181, 45))

plt.show()
