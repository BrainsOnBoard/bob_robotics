from collections.abc import Iterable

import cv2
import numpy as np


def apply_functions(im, funs):
    if funs is None:
        return im
    if isinstance(funs, Iterable):
        for fun in funs:
            im = apply_functions(im, fun)
        return im
    return funs(im)


def resize(width, height):
    '''Return a function to resize image to given size'''
    return lambda im: cv2.resize(im, (width, height))


def histeq(im):
    '''Histogram equalisation: Requires uint8s as input'''
    return cv2.equalizeHist(im)


def to_float(im):
    # Normalise values
    info = np.iinfo(im.dtype)
    return im.astype(float) / info.max


def add_randomness(im, sigma):
    assert im.dtype == float
    im += sigma * np.random.standard_normal(im.shape)
    return np.clip(im, 0, 1)


def fill_holes(im):
    '''Fills in any black objects'''
    assert im.dtype == np.uint8

    # Find polygons
    contours, _ = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return
    contours = list(contours)

    # Find the biggest polygon...
    biggest_idx = np.argmax([cv2.contourArea(cnt) for cnt in contours])

    #...drop it from list and colour it white
    biggest_cnt = contours.pop(biggest_idx)
    cv2.drawContours(im, [biggest_cnt], 0, 255, -1)

    # Colour remaining polygons black
    for cnt in contours:
        cv2.drawContours(im, [cnt], 0, 0, -1)

    return im


def threshold_otsu(im):
    _, thresh = cv2.threshold(
        im, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return thresh


def ground_mask(im):
    # First, roughly threshold image
    im = threshold_otsu(im)

    # Set top row to black and bottom row to white before we fill
    im[-1, :] = 255
    im[0, :] = 0

    # Fill any black holes
    im = fill_holes(im)

    # Fill white holes, by inverting, filling then inverting again
    im = cv2.bitwise_not(im)
    im = fill_holes(im)
    return cv2.bitwise_not(im)


def remove_sky(im):
    '''Sets the sky to 0'''
    mask = ground_mask(im)
    im = cv2.bitwise_and(im, mask)
    return im


def remove_sky_and_histeq(im):
    mask = ground_mask(im)
    return cv2.bitwise_and(histeq(im), mask)


def mask(mask_filepath, greyscale=True):
    '''Generate a filter for mask at given filepath'''

    mask = cv2.imread(mask_filepath, cv2.IMREAD_GRAYSCALE)
    assert mask is not None
    assert mask.dtype == np.uint8
    mask = np.where(mask < 128, 0, 255).astype(np.uint8)
    if not greyscale:
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    return lambda im: cv2.bitwise_and(im, mask)
