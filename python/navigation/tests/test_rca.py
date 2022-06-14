import pytest
from bob_robotics.navigation.ca import *


def test_typical():
    errs = [45, 10, 0, 10, 45]
    ca = calculate_rca(errs)
    assert ca.goal_idx == 2
    assert ca.bounds == (1, 3)
    assert ca.size() == 2


def test_explicit_goal():
    errs = [45, 10, 0, 10, 45]
    ca = calculate_rca(errs, 45, 2)
    assert ca.bounds == (1, 3)


def test_medfilt_right():
    ca = calculate_rca([0, 46, 30, 30, 45, 45], medfilt_size=3)
    assert ca.goal_idx == 0
    assert ca.bounds == (0, 3)


def test_medfilt_left():
    ca = calculate_rca([45, 45, 30, 30, 46, 0], medfilt_size=3)
    assert ca.goal_idx == 5
    assert ca.bounds == (2, 5)


def test_medfilt_both():
    errs = [45, 45, 30, 30, 46, 0, 46, 30, 30, 45, 45]
    ca = calculate_rca(errs, medfilt_size=3)
    assert (ca.filtered_vals == [45, 45, 30, 30, 30, 46, 30, 30, 30, 45, 45]).all()
    assert ca.goal_idx == 5
    assert ca.bounds == (2, 8)

def test_medfilt_both_nonzero():
    errs = [45, 45, 30, 30, 46, 1, 46, 30, 30, 45, 45]

    # We expect a warning because there are no perfect matches
    with pytest.warns(UserWarning):
        ca = calculate_rca(errs, medfilt_size=3)

    assert (ca.filtered_vals == [45, 45, 30, 30, 30, 46, 30, 30, 30, 45, 45]).all()
    assert ca.goal_idx == 2
    assert ca.bounds == (2, 4)
