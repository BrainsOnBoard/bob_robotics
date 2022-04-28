from warnings import warn

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import medfilt


class CatchmentArea:
    def __init__(self, vals, process_fun, thresh_fun, goal_idx, medfilt_size):
        # Convert to numpy array
        vals = np.array(vals)

        if vals.ndim != 1 or len(vals) == 0:
            raise ValueError('Input values must be vector')

        # Do median filtering
        self.filtered_vals = medfilt(vals, medfilt_size)

        # If goal_idx not provided, then we estimate it. If there is a perfect
        # match then we can safely assume this is the intended goal (because it
        # it is a self-vs-self comparison) and otherwise just assume the best
        # matching val indicates the goal.
        if goal_idx is None:
            indices = np.where(vals == 0)[0]
            if indices.size:
                goal_idx = indices[0]
            else:
                warn('Could not find exact match, using best match as goal')
                goal_idx = np.argmin(self.filtered_vals)

        # Apply process_fun to values from left and right of goal
        left = self.filtered_vals[goal_idx::-1]
        if len(left) > 0:
            left = process_fun(left)
        right = self.filtered_vals[goal_idx:]
        if len(right) > 0:
            right = process_fun(right)

        def ca(vec):
            if not vec.size:  # Empty array
                return 0

            # Return index of first value in vec for which thresh_fun() returns true
            return next(i for i, val in enumerate(vec) if thresh_fun(val))

        # A StopIteration error is raised when there are no values for which
        # thresh_fun() == True. In this case the CA extends beyond the lower or
        # upper bounds of the input values and we indicate it by setting the bound
        # to +-inf.
        try:
            lower = goal_idx - ca(left)
        except StopIteration:
            lower = float('-inf')
        try:
            upper = goal_idx + ca(right)
        except StopIteration:
            upper = float('inf')

        self.bounds = (lower, upper)
        self.goal_idx = goal_idx
        self.vals = vals

    def size(self):
        return self.bounds[1] - self.bounds[0]

    def get_finite_bounds(self):
        return (max(0, self.bounds[0]), min(len(self.vals) - 1, self.bounds[1]))

    def plot(self, xs=None, filter_zeros=True, ymax=None, ax=None):
        if xs:
            # Check entries is sorted
            assert all(xs[i] <= xs[i + 1] for i in range(len(xs) - 1))
        else:
            xs = range(len(self.vals))

        if ax is None:
            _, ax = plt.subplots()

        # (Optionally) filter out perfect matches as they're essentially
        # spurious and they make graphs appear more V-shaped than they truly are
        if filter_zeros:
            zeros = self.vals == 0
            self.vals = np.array(self.vals)
            self.vals[zeros] = None
            self.filtered_vals[zeros] = None
            print(sum(zeros), 'zero values are not being shown')

        # Plot unfiltered values
        lines = ax.plot(xs, self.vals)

        # Clamp bounds as they may be +-inf
        lo = max(0, self.bounds[0])
        hi = min(len(xs) - 1, self.bounds[1])

        # Indicate the CA by plotting a red line over the top
        ax.plot(xs[lo:hi], self.vals[lo:hi], 'r')
        ax.set_xlim(xs[0], xs[-1])

        # Plot dotted lines over the top to show median-filtered values
        if self.filtered_vals is not None:
            ax.plot(xs, self.filtered_vals,
                    ':', color=lines[0].get_color())

            ax.plot(xs[lo:hi], self.filtered_vals[lo:hi], 'r:')

        # Show the goal as a dashed black line
        ymax = ymax or ax.get_ylim()[1]
        ax.plot([xs[self.goal_idx]] * 2, (0, ymax), 'k--')
        ax.set_ylim(0, ymax)

        return ax


def calculate_ca(idf, goal_idx=None, medfilt_size=1):
    '''
    Get catchment area for 1D IDF.

    Differences from Andy's implementation:
        - I'm treating IDFs like this: [1, 2, 0, 2, 1] as having a CA of 2
          rather than 0.
        - Cases where vector length > filter size cause an error
    '''
    return CatchmentArea(idf, np.diff, lambda x: x < 0, goal_idx, medfilt_size)


def calculate_rca(errs, thresh=45, goal_idx=None, medfilt_size=1):
    '''
    Get rotational catchment area:
        i.e., area over which errs < some_threshold

    Differences from Andy's implementation:
        - medfilt_size defaults to 1, not 3
    '''
    assert thresh >= 0

    return CatchmentArea(errs, lambda x: x[1:], lambda th: th >= thresh,
                         goal_idx, medfilt_size)
