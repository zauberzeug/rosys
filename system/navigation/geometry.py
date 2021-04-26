import numpy as np


def projectPointOnDirection(point, yaw):

    return np.linalg.norm(point[:2]) * np.cos(yaw - np.arctan2(point[1], point[0]))


def dist1d(a, b, yaw):

    return projectPointOnDirection(b, yaw) - projectPointOnDirection(a, yaw)


def dist2d(a, b):

    return np.linalg.norm(np.array(b[:2]) - a[:2])


def angle(yaw1, yaw2):

    return ((yaw2 - yaw1) + np.pi) % (2 * np.pi) - np.pi


def abs_angle(yaw1, yaw2):

    return np.abs(angle(yaw1, yaw2))


def direction(pos1, pos2):

    return np.arctan2(pos2[1] - pos1[1], pos2[0] - pos1[0])


if __name__ == "__main__":

    assert np.isclose(angle(0.1, 0.4), 0.3)
    assert np.isclose(angle(0.8, 0.4), -0.4)
    assert np.isclose(abs_angle(0.8, 0.4), 0.4)
    assert np.isclose(abs_angle(0.8 + 2 * np.pi, 0.4), 0.4)
    assert np.isclose(abs_angle(0.8 - 4 * np.pi, 0.4), 0.4)
