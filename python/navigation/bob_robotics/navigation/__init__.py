import os.path
from collections.abc import Iterable
from warnings import warn

import numpy as np
from pandas import DataFrame

from ._navigation import *
from ._navigation import __version__  # ultimately set by build process
from .ca import *
from .caching import *
from .infomax import *
from .navigation import *
from .plotting import *



def _apply_functions(im, funs):
    if funs is None:
        return im
    if isinstance(funs, Iterable):
        for fun in funs:
            im = _apply_functions(im, fun)
        return im
    return funs(im)

def _calculate_cumdist(position):
    # Calculate cumulative distance for each point on route
    elem_dists = np.hypot(np.diff(position[:, 0]), np.diff(position[:, 1]))
    return np.nancumsum([0, *elem_dists])

def _get_headings_from_xy(x, y):
    headings = np.arctan2(np.diff(y), np.diff(x))
    headings = np.append(headings, [headings[-1]])
    return headings

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
        self.name = os.path.basename(path)

        not_unwrapped = self.needs_unwrapping()
        if not_unwrapped is None:
            warn("Not known whether or not database is unwrapped")
        elif not_unwrapped:
            warn("!!!!! This database has not been unwrapped. Analysis may not make sense! !!!!!")

        # Convert from a list of dicts
        df = DataFrame.from_records(self.get_entries())
        position = np.array(df[['x', 'y', 'z']])

        if interpolate_xy:
            _interpolate_nan_entries(df['Timestamp [ms]'].astype(float), position)
            df.x = position[:, 0]
            df.y = position[:, 1]

        # Calculate distance along route for each position
        distance = _calculate_cumdist(position)

        # User has specified limits
        if limits_metres is not None:
            assert len(limits_metres) == 2
            assert limits_metres[0] >= 0
            assert limits_metres[0] < limits_metres[1]
            if limits_metres[1] > distance[-1]:
                warn(f'Limit of {limits_metres[1]} is greater than route length of {distance[1]}')

            sel = np.logical_and(distance >= limits_metres[0], distance < limits_metres[1])
            position = position[sel]
            distance = distance[sel]
            df = df[sel]

        self.position = position  # NB: This can't be a column in a DataFrame
        self.entries = df
        self.entries['heading'] = _get_headings_from_xy(self.x, self.y)
        self.entries['distance'] = distance

        # Unfortunately, index values are discarded when a single element is
        # extracted from a DataFrame (the column names are then used as the
        # indexes), but we want to hang onto this so we know which database
        # entry e.g. a given data point corresponds to. So make our own extra
        # index column.
        self.entries['database_idx'] = self.entries.index

    def __getattr__(self, name):
        try:
            return object.__getattribute__(self, name)
        except AttributeError:
            return getattr(self.entries, name)

    def read_images(self, entries=None, preprocess=None, to_float=True,
                    greyscale=True):
        if entries is None:
            # ...then load all images
            entries = self.entries.index
        elif hasattr(entries, "iloc"):
            # ...then it's a pandas Series/DataFrame
            entries = entries.database_idx

        # Check if entries is a scalar. Unfortunately a DataFrame.index value
        # isn't an int so we can't just use isinstance() -- instead we try
        # casting.
        idx = None
        try:
            idx = int(entries)
        except TypeError:
            pass
        if idx is not None:
            # ...scalar value given
            return self.read_images([idx], preprocess, to_float, greyscale)[0]

        # Invoke C++ code
        images = super().read_images(entries, greyscale)

        if to_float:
            # Convert all the images to floats before we use them
            preprocess = (preprocess, _to_float)

        if not preprocess:
            return images
        return [_apply_functions(im, preprocess) for im in images]

    def read_image_entries(self, entries=None, preprocess=None, to_float=True,
                           greyscale=True):
        if entries is None:
            # ...then load all images
            entries = self.entries

        # We make a copy as pandas is funny about assigning to slices
        entries = entries.copy()

        # Check that we're not overwriting images
        assert not hasattr(entries, "image")

        entries["image"] = self.read_images(entries, preprocess, to_float, greyscale)

        return entries
