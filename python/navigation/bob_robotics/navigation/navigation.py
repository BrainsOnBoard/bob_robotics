import bob_robotics.navigation as bobnav
import numpy as np

from . import caching


# Yes, there is np.atleast_3d, but the problem is that it tacks the extra
# dimension onto the end, whereas we want it to be at the beginning (e.g. you
# get an array of dims [90, 360, 1] rather than [1, 90, 360])
def to_images_array(x):
    if hasattr(x, "to_list"):
        # Convert elements of DataFrame. The .to_numpy() method doesn't work
        # here because our data are multi-dimensional arrays.
        x = x.to_list()

    # Make sure x is a numpy array
    x = np.array(x)
    if x.ndim == 3:
        return x

    assert x.ndim == 2
    return np.array([x])


def mean_absdiff(x, y):
    """Return mean absolute difference between two images or sets of images."""
    x = to_images_array(x)
    y = to_images_array(y)

    # Either x or y can be 3D, but not both
    assert len(x) == 1 or len(y) == 1

    if len(x) > 1:
        x, y = y, x

    # Convert back to 2D
    x = np.squeeze(x, axis=0)

    pm = bobnav.PerfectMemory(x.shape[::-1])
    pm.train(x)
    return pm.test(y)


def ridf(images, snapshots, step=1):
    """Return an RIDF for one or more images vs one or more snapshots (as a vector)"""
    snapshots = to_images_array(snapshots)
    pm = bobnav.PerfectMemory(snapshots[0].shape[::-1])
    pm.train(snapshots)
    return np.array(pm.ridf(images, step=step).ridf.to_list())


def get_ridf_headings_no_cache(images, snapshots, step=1):
    """A version of get_ridf_headings() without on-disk caching of results.

    Parameters are the same as for get_ridf_headings().
    """
    snapshots = to_images_array(snapshots)
    pm = bobnav.PerfectMemory(snapshots[0].shape[::-1])
    pm.train(snapshots)
    return pm.ridf(images, step=step).estimated_heading


@caching.cache_result
def get_ridf_headings(images, snapshots, step=1):
    """Get a numpy array of headings computed from multiple images and snapshots

    Parameters
    ----------
    images
        List of test images
    snapshots
        List of training images
    step
        Step size for each rotation in pixels
    parallel
        Whether to run algorithm in parallel. If omitted, it will be run in
        parallel according to a heuristic (i.e. if there is enough work)

    Returns:
        Headings in radians.
    """
    return get_ridf_headings_no_cache(images, snapshots, step)


def route_idf(images, snap):
    return mean_absdiff(images, snap)


def normalise180(ths):
    if np.isscalar(ths):
        return normalise180([ths])[0]

    ths = np.array(ths) % 360
    ths[ths > 180] -= 360
    return ths


def ridf_to_degrees(diffs):
    assert diffs.ndim == 1 or diffs.ndim == 2
    bestcols = np.argmin(diffs, axis=-1)
    return 360.0 * bestcols / diffs.shape[-1]


def ridf_to_radians(diffs):
    return np.deg2rad(ridf_to_degrees(diffs))


def route_ridf(images, snap, step=1):
    return np.amin(ridf(images, snap, step=step), axis=1)


def route_ridf_errors(images, snap, step=1):
    diffs = ridf(images, snap, step=step)
    return np.abs(normalise180(ridf_to_degrees(diffs)))
