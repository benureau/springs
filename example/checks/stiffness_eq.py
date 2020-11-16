"""Verifying that linear changes in stiffness induce linear changes in force."""

import random
import springs

"""
4000 0.09782510911613833
12000 0.058890191484544045
500000 0.01957107322474618
1000000 0.017500000339420733
"""

ENGINE = 'cpp'

def create_link(space, x, mass, stiffness, damping_ratio=1.0, y=440):
    n_a = space.add_node(x, y + 100, mass=mass, fixed=False)
    space.add_rect(-1000, 1000, 50, 100, 0)
    n_b = space.add_node(x, y, mass=mass)
    # print(stiffness)
    m = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m.stiffness = stiffness
    return n_b

def displacement(stiffness, mass=1.0, dt=0.005, n_substep=5, gravity=-500, steps=10000, engine='cpp'):
    space = springs.create_space(dt=dt, n_substep=n_substep, gravity=gravity, engine=engine)
    node = create_link(space, 0, mass, stiffness, y=0)
    for _ in range(steps):
        space.step()
    return -node.y

if __name__ == '__main__':
    ds = []
    for stiffness in [4000, 12000, 500000, 1000000]:
    # for stiffness in [5111, 8490, 25547, 28571]:
        d = displacement(stiffness)
        ds.append(d)
        print(stiffness, d)

    print([0.026116380034543717, 0.00962726598162749, 0.0012657998468039954, 0.0011023686575934934])
    d_4000 = displacement(4000)
    for d in [0.026116380034543717, 0.00962726598162749, 0.0012657998468039954, 0.0011023686575934934]:
        print('{} for {:g}'.format(d_4000/d * 4000, d))
