"""A simple link"""

import random

import springs


random.seed(0)

engine = 'cython'
space = springs.create_space(dt=0.0001, gravity=-100.0, engine='cpp')

def set_spring(x, length, mass, stiffness):
    n_a = space.add_node(x, 200, mass=1.0, fixed=True)
    n_b = space.add_node(x + length, 200, mass=mass)
    return space.add_spring(n_a, n_b, stiffness, damping_ratio=1.0, actuated=True)

m = set_spring(250, length=100.0, mass=1.0, stiffness=10.0)
m.contract(0.5)


if __name__ == '__main__':
    springs.create_display(space, width=500, height=400).run()
