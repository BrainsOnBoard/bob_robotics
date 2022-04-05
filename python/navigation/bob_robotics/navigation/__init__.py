from ._navigation import *
from pandas import DataFrame
import numpy as np


class Database(DatabaseInternal):
    def __init__(self, path):
        super().__init__(path)

        # Convert from a list of dicts
        df = DataFrame.from_records(self.get_entries())
        self.entries = df

        self.position = np.array(df[['x', 'y', 'z']])
        self.x = self.position[:, 0]
        self.y = self.position[:, 1]
        self.z = self.position[:, 2]
        self.heading = np.array(df['yaw'])
        self.filepath = df['filepath']

        # Calculate cumulative distance for each point on route
        elem_dists = np.hypot(np.diff(self.x), np.diff(self.y))
        self.distance = np.nancumsum([0, *elem_dists])
