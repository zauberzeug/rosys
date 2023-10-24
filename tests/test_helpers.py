from rosys import helpers


def test_ramp():
    # low, high --> low, high
    assert helpers.ramp(1, 2, 8, 20, 80) == 10
    assert helpers.ramp(2, 2, 8, 20, 80) == 20
    assert helpers.ramp(5, 2, 8, 20, 80) == 50
    assert helpers.ramp(8, 2, 8, 20, 80) == 80
    assert helpers.ramp(9, 2, 8, 20, 80) == 90
    assert helpers.ramp(1, 2, 8, 20, 80, clip=True) == 20
    assert helpers.ramp(2, 2, 8, 20, 80, clip=True) == 20
    assert helpers.ramp(5, 2, 8, 20, 80, clip=True) == 50
    assert helpers.ramp(8, 2, 8, 20, 80, clip=True) == 80
    assert helpers.ramp(9, 2, 8, 20, 80, clip=True) == 80

    # low, high --> high, low
    assert helpers.ramp(1, 2, 8, 80, 20) == 90
    assert helpers.ramp(2, 2, 8, 80, 20) == 80
    assert helpers.ramp(5, 2, 8, 80, 20) == 50
    assert helpers.ramp(8, 2, 8, 80, 20) == 20
    assert helpers.ramp(9, 2, 8, 80, 20) == 10
    assert helpers.ramp(1, 2, 8, 80, 20, clip=True) == 80
    assert helpers.ramp(2, 2, 8, 80, 20, clip=True) == 80
    assert helpers.ramp(5, 2, 8, 80, 20, clip=True) == 50
    assert helpers.ramp(8, 2, 8, 80, 20, clip=True) == 20
    assert helpers.ramp(9, 2, 8, 80, 20, clip=True) == 20

    # high, low --> low, high
    assert helpers.ramp(1, 8, 2, 20, 80) == 90
    assert helpers.ramp(2, 8, 2, 20, 80) == 80
    assert helpers.ramp(5, 8, 2, 20, 80) == 50
    assert helpers.ramp(8, 8, 2, 20, 80) == 20
    assert helpers.ramp(9, 8, 2, 20, 80) == 10
    assert helpers.ramp(1, 8, 2, 20, 80, clip=True) == 80
    assert helpers.ramp(2, 8, 2, 20, 80, clip=True) == 80
    assert helpers.ramp(5, 8, 2, 20, 80, clip=True) == 50
    assert helpers.ramp(8, 8, 2, 20, 80, clip=True) == 20
    assert helpers.ramp(9, 8, 2, 20, 80, clip=True) == 20

    # high, low --> high, low
    assert helpers.ramp(1, 8, 2, 80, 20) == 10
    assert helpers.ramp(2, 8, 2, 80, 20) == 20
    assert helpers.ramp(5, 8, 2, 80, 20) == 50
    assert helpers.ramp(8, 8, 2, 80, 20) == 80
    assert helpers.ramp(9, 8, 2, 80, 20) == 90
    assert helpers.ramp(1, 8, 2, 80, 20, clip=True) == 20
    assert helpers.ramp(2, 8, 2, 80, 20, clip=True) == 20
    assert helpers.ramp(5, 8, 2, 80, 20, clip=True) == 50
    assert helpers.ramp(8, 8, 2, 80, 20, clip=True) == 80
    assert helpers.ramp(9, 8, 2, 80, 20, clip=True) == 80
