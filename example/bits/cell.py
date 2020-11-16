"""A simple springy robot that moves"""

import math
import random

import springs


random.seed(0)
space = springs.create_space(dt=0.001, gravity=0.0, engine="cpp")

j1 = space.add_node(1000.0, 500.0, mass=1.0)
j2 = space.add_node(1000.0, 600.0, mass=1.0)
j3 = space.add_node( 900.0, 550.0, mass=1.0)
j4 = space.add_node(1150.0, 550.0, mass=1.0, fixed=False)

m = space.add_link(j1, j2, stiffness=5000, damping_ratio=1.0)
space.add_link(j1, j3, stiffness=50000)
space.add_link(j2, j3, stiffness=50000)
space.add_link(j1, j4, stiffness=50000)
space.add_link(j2, j4, stiffness=50000)

tick_period = round(2.0/space.dt)

def update(space):
    m.contract(1 + 0.5 * math.sin(2 * math.pi * space.t / 4))
    if space.ticks % tick_period == 0:
        j4.fixed = not j4.fixed
space.add_update_function(update)


if __name__ == '__main__':
    springs.create_display(space, width=1440, height=900, zoom=0.75, spf=0.05,
                           paused=False).run()
