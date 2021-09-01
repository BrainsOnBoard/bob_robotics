#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import os
import sys

df = pd.read_csv(os.path.join(sys.argv[1], "database_entries.csv"), na_values="nan")

# strip whitespace from column headers
df = df.rename(columns=lambda x: x.strip())

print('Duration: %g s' % (df['Timestamp [ms]'][df.index[-1]] / 1000.0))

x = df['X [mm]'].astype(float).notna() / 1000.0
y = df['Y [mm]'].astype(float).notna() / 1000.0

print('Range [m]: (%g, %g)' % (np.ptp(x), np.ptp(y)))

plt.plot(x, y, 'x-')
plt.title(sys.argv[1])
plt.show()
