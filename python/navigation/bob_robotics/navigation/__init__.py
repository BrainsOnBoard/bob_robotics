from ._navigation import *
from pandas import DataFrame
import numpy as np
from collections.abc import Iterable
from warnings import warn


def _apply_functions(im, funs):
    if funs is None:
        return im
    if isinstance(funs, Iterable):
        for fun in funs:
            im = _apply_functions(im, fun)
        return im
    return funs(im)

def _to_float(im):
    # Normalise values
    info = np.iinfo(im.dtype)
    return im.astype(float) / info.max

class Database(DatabaseInternal):
    def __init__(self, path):
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
        self.x = self.position[:, 0]
        self.y = self.position[:, 1]
        self.z = self.position[:, 2]
        self.heading = np.array(df['yaw'])
        self.filepath = df['filepath']

        # Calculate cumulative distance for each point on route
        elem_dists = np.hypot(np.diff(self.x), np.diff(self.y))
        self.distance = np.nancumsum([0, *elem_dists])

    def read_images(self, entries=None, preprocess=None, to_float=True,
                    greyscale=True):
        images = super().read_images(entries, greyscale)

        if to_float:
            # Convert all the images to floats before we use them
            preprocess = (preprocess, _to_float)

        if not preprocess:
            return images
        return [_apply_functions(im, preprocess) for im in images]
