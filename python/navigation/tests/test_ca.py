from bob_robotics.navigation.ca import *


def test_typical():
    ca = calculate_ca([1, 2, 0, 2, 3, 1])
    assert ca.goal_idx == 2
    assert ca.bounds == (1, 4)
    assert ca.size() == 3


def test_explicit_goal():
    ca = calculate_ca([1, 2, 0, 2, 3, 1])
    assert ca.bounds == (1, 4)


def test_no_left():
    ca = calculate_ca([0, 2, 1])
    assert ca.goal_idx == 0
    assert ca.bounds == (0, 1)
    assert ca.size() == 1


def test_no_right():
    ca = calculate_ca([1, 2, 0])
    assert ca.goal_idx == 2
    assert ca.bounds == (1, 2)
    assert ca.size() == 1


def test_single():
    assert calculate_ca([0]).size() == 0


def test_infinite_left():
    ca = calculate_ca([1, 1, 0, 2, 1])
    assert ca.goal_idx == 2
    assert ca.bounds == (float('-inf'), 3)


def test_infinite_right():
    ca = calculate_ca([1, 2, 0, 1, 1])
    assert ca.goal_idx == 2
    assert ca.bounds == (1, float('inf'))


def test_medfilt_right():
    assert calculate_ca([0, 1, 2, 1, 3, 1], medfilt_size=3).size() == 3


def test_medfilt_left():
    assert calculate_ca([1, 3, 1, 2, 1, 0], medfilt_size=3).size() == 3


def test_medfilt_both():
    ca = calculate_ca([1, 3, 1, 2, 1, 0, 0, 0, 1, 2, 1, 3, 1], medfilt_size=3)
    assert (ca.filtered_vals == [1, 1, 2, 1, 1, 0, 0, 0, 1, 1, 2, 1, 1]).all()
    assert ca.goal_idx == 5
    assert ca.bounds == (2, 10)
    assert ca.size() == 8
