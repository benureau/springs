"""A simple springy robot that moves"""

import math
import random

import springs



# FIXME: triangle collisions are defined for only two edges right now (was purposeful).

random.seed(0)
space = springs.create_space(dt=0.01, gravity=0.0, engine="cpp", restitution_threshold=0.0)

x_0, y_0 = 450.0, 500.0
speed = 0.1

K = 100
for i in range(K):
    angle = (i/K + 0.6) * 2 * math.pi
    x = 300.0 * math.cos(angle)
    y = 300.0 * math.sin(angle)
    node = space.add_node(x_0 + x, y_0 + y, mass=1.0, friction=0.5)
    node.v_x, node.v_y = - x * speed, - y * speed

# node = space.add_node(x_0 + -7, y_0 + 55.0, mass=1.0)
# node.v_x, node.v_y = 7 * speed, - 55 * speed

#triangle = space.add_triangle(x_0-200, y_0-150, x_0+200, y_0-50, x_0, y_0 + 150.0, 1.0)

rect = space.add_rect(x_0 - 100.0, x_0 + 150.0, y_0 - 100.0, y_0 + 50.0, 1.0)

# def update(space):
#     print(node.v_x, node.v_y)
# space.add_update_function(update)


if __name__ == '__main__':
    springs.create_display(space, width=1000, height=900, zoom=0.7, spf=0.04,
                           paused=False).run()
