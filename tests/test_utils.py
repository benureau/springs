import math

from springs import utils


def test_angle():
    assert utils.angle(( 0,  1), (0, 0), ( 1,  0)) ==   math.pi/2
    assert utils.angle(( 0,  3), (0, 0), ( 1,  0)) ==   math.pi/2
    assert utils.angle((-1,  0), (0, 0), ( 1,  0)) ==   math.pi
    assert utils.angle(( 1,  0), (0, 0), (-1,  0)) == - math.pi
    assert utils.angle(( 0, -1), (0, 0), ( 0,  1)) == - math.pi
    assert utils.angle(( 1,  0), (0, 0), (-1,  0)) == - math.pi
    assert utils.angle(( 1,  1), (0, 0), (-1, -1)) ==   math.pi


if __name__ == '__main__':
    test_angle()
