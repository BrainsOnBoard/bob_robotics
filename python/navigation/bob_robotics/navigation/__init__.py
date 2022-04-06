import os.path
from collections.abc import Iterable
from warnings import warn

import numpy as np
from pandas import DataFrame

from ._navigation import *


def _apply_functions(im, funs):
    if funs is None:
        return im
    if isinstance(funs, Iterable):
        for fun in funs:
            im = _apply_functions(im, fun)
        return im
    return funs(im)

def _interpolate_nan_entries(ts, position):
    def is_idx_nan(idx):
        return np.isnan(position[idx, 0])

    # FIXME
    assert not is_idx_nan(0)

    i = j = 0
    while i < len(position):
        j = i + 1
        while j < len(position) and is_idx_nan(j):
            j += 1

        # If there are trailing NaN entries, fill them with the contents of the
        # final valid entry
        if j == len(position):
            position[i + 1:, :] = position[i, :]
            break

        t_range = ts[j] - ts[i]
        for k in range(i + 1, j):
            t_prop = (ts[k] - ts[i]) / t_range
            assert t_prop > 0 and t_prop < 1
            position[k, :] = position[i, :] + t_prop * (position[j, :] - position[i, :])

        i = j

    # Sanity check
    assert not np.any(np.isnan(position))

def _to_float(im):
    # Normalise values
    info = np.iinfo(im.dtype)
    return im.astype(float) / info.max

class Database(DatabaseInternal):
    def __init__(self, path, limits_metres=None, interpolate_xy=False):
        super().__init__(path)

        not_unwrapped = self.needs_unwrapping()
        if not_unwrapped is None:
            warn("Not known whether or not database is unwrapped")
        elif not_unwrapped:
            warn("!!!!! This database has not been unwrapped. Analysis may not make sense! !!!!!")

        # Convert from a list of dicts
        df = DataFrame.from_records(self.get_entries())
        self.entries = df
        self.position = np.array(df[['x', 'y', 'z']])

        if interpolate_xy:
            _interpolate_nan_entries(df['Timestamp [ms]'].astype(float), self.position)

        # Calculate distance along route for each position
        distance = self._calculate_cumdist()

        # User has specified limits
        if limits_metres is not None:
            assert len(limits_metres) == 2
            assert limits_metres[0] >= 0
            assert limits_metres[0] < limits_metres[1]
            if limits_metres[1] > distance[-1]:
                warn(f'{os.path.basename(path)}: Limit of {limits_metres[1]} is greater than route length of {distance[1]}')

            sel = np.logical_and(distance >= limits_metres[0], distance < limits_metres[1])
            self.position = self.position[sel]
            distance = distance[sel]
            df = df[sel]

            # Also delete the relevant entries from the C++ object
            self._truncate(np.argwhere(sel))

        # Convenient aliases
        self.x = self.position[:, 0]
        self.y = self.position[:, 1]
        self.z = self.position[:, 2]
        self.heading = np.array(df['yaw'])
        self.distance = distance
        self.filepath = df['filepath']

    def _calculate_cumdist(self):
        # Calculate cumulative distance for each point on route
        elem_dists = np.hypot(np.diff(self.position[:, 0]), np.diff(self.position[:, 1]))
        return np.nancumsum([0, *elem_dists])


    def read_images(self, entries=None, preprocess=None, to_float=True,
                    greyscale=True):
        images = super().read_images(entries, greyscale)

        if to_float:
            # Convert all the images to floats before we use them
            preprocess = (preprocess, _to_float)

        if not preprocess:
            return images
        return [_apply_functions(im, preprocess) for im in images]
