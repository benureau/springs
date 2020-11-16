"""Verifying that linear changes in stiffness induce linear changes in force."""

import math
import random
import springs


ENGINE = 'cpp'
MASS = 1.0
DT = 0.005


def create_link(space, x, mass, stiffness, damping_ratio=1.0, y=440):
    n_a = space.add_node(x, y + 100, mass=mass, fixed=True)
    n_b = space.add_node(x, y, mass=mass)
    # print(stiffness)
    m = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m.stiffness = stiffness
    return n_b

def displacement(stiffness, mass=MASS, dt=DT, steps=10000, engine=ENGINE):
    space = springs.create_space(dt=dt, n_substep=5, gravity=-500.0, engine=engine)
    node = create_link(space, 0, mass, stiffness, y=0)
    for _ in range(steps):
        space.step()
    return -node.y

def real_stiffness(stiffness, mass=MASS, dt=DT, ref_stiffness=4000):
    target_stiffness = stiffness
    bound_low, bound_high = None, None
    ref_ratio = ref_stiffness * displacement(ref_stiffness, mass=mass, dt=dt)
    obs_ratio = target_stiffness * displacement(stiffness, mass=mass, dt=dt)

    if ref_ratio < obs_ratio:
        bound_low = stiffness
        while bound_high is None:
            stiffness *= 10
            obs_ratio = target_stiffness * displacement(stiffness, mass=mass, dt=dt)
            if ref_ratio > obs_ratio:
                bound_high = stiffness
    else:
        bound_high = stiffness
        while bound_low is None:
            stiffness /= 10
            obs_ratio = target_stiffness * displacement(stiffness, mass=mass, dt=dt)
            if ref_ratio < obs_ratio:
                bound_low = stiffness

    while bound_high - bound_low > 0.01:
        # print(stiff_high - stiff_low)
        bound_middle = (bound_high-bound_low) / 2 + bound_low
        ratio = displacement(bound_middle) * target_stiffness # * mass_gamma(mass, mass, target_stiffness)
        if ratio < ref_ratio:
            bound_high = bound_middle
        else:
            bound_low  = bound_middle

    return (bound_high - bound_low) / 2 + bound_low


if __name__ == '__main__':
    random.seed(0)
    print('engine: {}  mass: {}'.format(ENGINE, MASS))

    # ref_ratio = 100 * observed_stiffness(100)
    # print(ref_ratio)
    for i, stiffness in enumerate([10, 100, 2000, 20000]):
        print('{}: {:.2f}'.format(stiffness, real_stiffness(stiffness, mass=MASS, dt=DT, ref_stiffness=4000)))

    # ref_ratio = 100 * observed_stiffness(100)
    # print(ref_ratio)
    # for i, stiffness in enumerate([10, 100, 2000]):
    #     print('{}: {:.2f}'.format(stiffness, ref_ratio / observed_stiffness(stiffness)))
