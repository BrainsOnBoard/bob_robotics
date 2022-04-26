import math
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

    def read_images(self, entries=None, preprocess=None, greyscale=True):
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
            return self.read_images([idx], preprocess, greyscale)[0]

        # Invoke C++ code
        images = super().read_images(entries, greyscale)

        if not preprocess:
            return images
        return [_apply_functions(im, preprocess) for im in images]

    def read_image_entries(self, entries=None, preprocess=None, greyscale=True):
        if entries is None:
            # ...then load all images
            entries = self.entries

        # We make a copy as pandas is funny about assigning to slices
        entries = entries.copy()

        # Check that we're not overwriting images
        assert not hasattr(entries, "image")

        entries["image"] = self.read_images(entries, preprocess, greyscale)

        return entries

    def calculate_distance(self, entry1, entry2):
        '''Euclidean distance between two database entries (in m)'''

        # CA bounds may be infinite so handle this
        if math.isinf(entry1) or math.isinf(entry2):
            return float('inf')

        return np.linalg.norm(np.array(self.position[entry1, 0:2]) - self.position[entry2, 0:2])

    def calculate_distances(self, ref_entry, entries):
        dists = []
        for i in entries:
            dist = self.calculate_distance(ref_entry, i)
            dists.append(dist if i >= ref_entry else -dist)
        return dists

    def calculate_heading_offset(self, distance_thresh):
        i = 0
        while self.distance[i] < distance_thresh:
            i += 1

        dpos = self.position[i, :] - self.position[0, :]
        return math.atan2(dpos[1], dpos[0])

    def entry_bounds(self, max_dist, start_entry):
        '''Get upper and lower bounds for frames > max_dist from start frame'''
        upper_entry = start_entry
        while self.calculate_distance(start_entry, upper_entry) < max_dist:
            upper_entry += 1
        lower_entry = start_entry
        while self.calculate_distance(start_entry, lower_entry) < max_dist:
            lower_entry -= 1
        return (lower_entry, upper_entry)

    def get_nearest_entries(self, x, y=None):
        if y is None:
            return self.entries.iloc[self.get_nearest_entries(x.x, x.y)]

        diff_x = np.atleast_2d(self.x) - np.atleast_2d(x).T
        diff_y = np.atleast_2d(self.y) - np.atleast_2d(y).T
        distances = np.hypot(diff_x, diff_y)
        nearest = np.argmin(distances, axis=1)
        assert len(nearest) == len(x)
        return nearest

    def load_test_frames(self, ref_entry, frame_dist, preprocess=None, fr_step=1):
        (lower, upper) = (ref_entry - frame_dist, ref_entry + frame_dist)
        entries = range(lower, upper+fr_step, fr_step)
        snap = self.read_images(ref_entry, preprocess)
        images = self.read_images(entries, preprocess)
        print("Testing frames %i to %i (n=%i)" %
              (lower, upper, len(images)))
        return (images, snap, entries)

    def plot_idfs_frames(self, ref_entry, frame_dist, preprocess=None, fr_step=1, ridf_step=1, filter_zeros=True):
        (images, snap, entries) = self.load_test_frames(
            ref_entry, frame_dist, preprocess, fr_step)

        idf_diffs = route_idf(images, snap)
        ridf_diffs = route_ridf(images, snap, ridf_step)
        return plot_route_idf(entries, idf_diffs, ridf_diffs,
                                 filter_zeros=filter_zeros)

    def plot_idfs(self, ax, ref_entry, max_dist, preprocess=None, fr_step=1, ridf_step=1, filter_zeros=True):
        (lower, upper) = self.entry_bounds(max_dist, ref_entry)
        entries = range(lower, upper+fr_step, fr_step)
        dists = self.calculate_distances(ref_entry, entries)

        # Load snapshot and test images
        snap = self.read_images(ref_entry, preprocess)
        images = self.read_images(entries, preprocess)
        print("Testing frames %i to %i (n=%i)" %
              (lower, upper, len(images)))

        # Show which part of route we're testing
        ax[1].plot(self.x, self.y, self.position[lower:upper, 0], self.position[lower:upper, 1],
                   self.position[ref_entry, 0], self.position[ref_entry, 1], 'ro')
        ax[1].set_xlabel("x (m)")
        ax[1].set_ylabel("y (m)")
        ax[1].axis("equal")

        # Plot (R)IDF diffs vs distance
        idf_diffs = route_idf(images, snap)
        ridf_diffs = route_ridf(images, snap, ridf_step)

        if filter_zeros:
            idf_diffs = zeros_to_nones(idf_diffs)
            ridf_diffs = zeros_to_nones(ridf_diffs)
        ax[0].plot(dists, idf_diffs, dists, ridf_diffs)
        ax[0].set_xlabel("Distance (m)")
        ax[0].set_xlim(-max_dist, max_dist)
        ax[0].set_xticks(range(-max_dist, max_dist))
        ax[0].set_ylabel("Mean image diff (px)")
        ax[0].set_ylim(0, 0.06)
