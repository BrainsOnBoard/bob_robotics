import matplotlib.pyplot as plt
import numpy as np


def anglequiver(ax, x, y, theta, invert_y=False, **kwargs):
    u = np.cos(theta)
    v = np.sin(theta)
    if invert_y:
        v = -v
    return ax.quiver(x, y, u, v, angles='xy', **kwargs) #, scale_units='xy')


def zeros_to_nones(vals):
    zeros = 0
    ret = []
    for val in vals:
        if val == 0:
            ret.append(None)
            zeros += 1
        else:
            ret.append(val)

    if zeros > 0:
        print('%i zero values (perfect matches?) are not being shown' % zeros)

    return ret


def plot_route_idf(xs, *errs_args, ax=None, filter_zeros=True, xlabel='Frame',
                   labels=None, adjust_ylim=True):
    if not labels:
        labels = len(errs_args[0]) * [None]

    if ax is None:
        _, ax = plt.subplots()

    lines = []
    for errs, label in zip(errs_args, labels):
        if filter_zeros:
            errs = zeros_to_nones(errs)
        lines.append(ax.plot(xs, errs, label=label)[0])

    ax.set_xlabel(xlabel)
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylabel("Mean image diff (px)")
    if adjust_ylim:
        ax.set_ylim(bottom=0)

    if filter_zeros:
        for errs in errs_args:
            for entry, val in zip(xs, errs):
                if val == 0:
                    ax.plot((entry, entry), ax.get_ylim(), 'r:')

    if labels[0]:
        ax.legend()

    return lines


def plot_ridf(diffs, ax=None, im=None, adjust_ylim=True, show_minimum=False):
    assert diffs.ndim == 1

    # We want the plot to go from -180° to 180°, so we wrap around
    diffs = np.roll(diffs, round(len(diffs) / 2))
    diffs = np.append(diffs, diffs[0])

    if ax is None:
        _, ax = plt.subplots()
    else:
        ax.clear()

    xs = np.linspace(-180, 180, len(diffs))
    ax.plot(xs, diffs)
    ax.set_xlim(-180, 180)
    if adjust_ylim:
        ax.set_ylim(bottom=0)
    ax.set_xticks(range(-180, 181, 45))

    if show_minimum:
        idx = np.argmin(diffs)
        ax.plot([xs[idx]] * 2, ax.get_ylim(), 'k--', alpha=0.7)

    if im is not None:
        ext = (*ax.get_xlim(), *ax.get_ylim())
        ax.imshow(im, cmap='gray', zorder=0, extent=ext)
        ax.set_aspect((im.shape[0] / im.shape[1]) *
                      ((ext[1] - ext[0]) / (ext[3] - ext[2])))

    return ax
