import math
import time
from collections.abc import Iterable


def distance(a, b):
    d_x, d_y = (a.x - b.x), (a.y - b.y)
    return math.sqrt(d_x*d_x + d_y*d_y)


def pos_rel(x, y, pos_a, pos_b):
    angle = math.atan2(float(pos_b.y - pos_a.y), float(pos_b.x - pos_a.x))
    x_origin = (pos_a.x + pos_b.x) / 2
    y_origin = (pos_a.y + pos_b.y) / 2
    cos, sin = math.cos(angle), math.sin(angle)
    return x * cos - y * sin + x_origin, x * sin + y * cos + y_origin

def polygon(n, center_x, center_y, size):
    """Return the position of the nodes of a regular polygon"""
    radius = size / math.sqrt( 2 * (1 - math.cos( 2 * math.pi / n )))

    x, y = 0, radius
    nodes = []
    cos, sin = math.cos(2*math.pi/n), math.sin(2*math.pi/n)
    for i in range(n):
        x, y = x * cos - y * sin , x * sin + y * cos
        nodes.append((x + center_x, y + center_y))
    return nodes

def listify(v, size):
    """If `v` is a list return it, else return a list of `v` of size `size`."""
    if isinstance(v, Iterable) and not isinstance(v, str):
        # assert len(v) == size
        return v
    else:
        return size * [v]



def uniquify(objects):
    """Given a list of objects, produce a new list where each object appears only once."""
    return list({id(e): e for e in objects}.values())

def angle(a, b, c):
    """Return the angle from three points, where b is the vertex of the angle.

    Angle is between -pi and pi. Angles in the *counterclockwise* direction are
    positive.
    """
    x_a, y_a = a
    x_b, y_b = b
    x_c, y_c = c
    return math.atan2(y_a - y_b, x_a - x_b) - math.atan2(y_c - y_b, x_c - x_b)



if __name__ == '__main__':
    for _ in range(10000000):
        angle((0.12, 0.23), (0.34, 0.45), (0.56, 0.67))
