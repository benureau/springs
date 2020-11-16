"""Verifying that linear changes in stiffness induce linear changes in force."""

import math
import random

import numpy as np

import springs

ENGINE = 'cpp'
MASS = 0.1
DT = 0.005


def create_link(space, x, mass, stiffness, damping_ratio=1.0, y=440):
    n_a = space.add_node(x, y + 100, mass=mass, fixed=True)
    n_b = space.add_node(x, y, mass=mass)
    m = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m.stiffness = stiffness
    return n_b, m

def mass_gamma(m_a, m_b, stiffness):

    # print('m_a {} m_b {} stiffness {}'.format(m_a, m_b, stiffness))
    inv_mass = (1/m_a + 1/m_b)
    # print('inv_mass {}'.format(inv_mass))
    # print('stiffness mass {}'.format(stiffness * inv_mass))
    omega = math.sqrt(stiffness * inv_mass)
    # print('omega {}'.format(omega))
    damping = 2.0 / inv_mass * 1.0 * omega
    gamma = 1.0 / ( DT * (damping + DT * stiffness) + 0.0000001)
    # print('damping {} gamma {}'.format(damping, gamma))
    inv_mass += gamma
    return gamma/inv_mass

def find_stiffness(mass, target_stiffness, stiff_ref=4000, engine='cpp'):
    #print('mass: {} target_stiffness: {}'.format(mass, target_stiffness))

    def displacement(stiffness, mass=mass):
        space = springs.create_space(dt=DT, gravity=-500.0, engine=engine, n_substep=5)
        y = 500
        node, link = create_link(space, 20, mass, stiffness, y=440)
        for _ in range(100000):
            space.step()
        return 440-node.y

    def force(stiffness, mass):
        space = springs.create_space(dt=DT, gravity=-500.0, engine=engine, n_substep=5)
        y = 500
        node, link = create_link(space, 20, mass, stiffness, y=440)
        for _ in range(100000):
            space.step()
        return link.force

    ref_ratio = displacement(stiff_ref, 1.0) * stiff_ref # * mass_gamma(mass, mass, stiff_ref)

    stiff_low, stiff_high = float('-inf'), float('inf')

    ratio = displacement(target_stiffness) * target_stiffness # * mass_gamma(mass, mass, target_stiffness)
    if ratio <= ref_ratio:
        stiff_high =       target_stiffness
        stiff_low  = 0.001 * target_stiffness
        assert displacement(stiff_low) * target_stiffness > ref_ratio # * mass_gamma(mass, mass, target_stiffness) > ref_ratio
    else:
        stiff_high = 1000.0 * target_stiffness
        stiff_low  = target_stiffness
        assert displacement(stiff_high) * target_stiffness < ref_ratio # * mass_gamma(mass, mass, target_stiffness)

    while stiff_high - stiff_low > 0.01:
        # print(stiff_high - stiff_low)
        stiff_middle = (stiff_high-stiff_low) / 2 + stiff_low
        ratio = displacement(stiff_middle) * target_stiffness # * mass_gamma(mass, mass, target_stiffness)
        if ratio < ref_ratio:
            stiff_high = stiff_middle
        else:
            stiff_low  = stiff_middle

    virtual_stiffness = (stiff_high-stiff_low) / 2 + stiff_low
    # print('mass_gamma ref {}', target_stiffness * mass_gamma(mass, mass, stiff_ref))
    # print('mass_gamma {:.2f}'.format(target_stiffness),  mass_gamma(mass, mass, target_stiffness))
    #print('mass_gamma {:.2f}'.format(virtual_stiffness), mass_gamma(mass, mass, virtual_stiffness))
    print('mass: {:6.3f} stiff: {:.1f}: T {:10.2f},'.format(mass, target_stiffness, virtual_stiffness))
    # print(' {:.1f}: R {:.2f},'.format(target_stiffness, virtual_stiffness * mass_gamma(mass, mass, virtual_stiffness)))
    # print('force: target {}, real: {}'.format(force(target_stiffness, mass),
    #                                           force(virtual_stiffness, mass)))

    return virtual_stiffness
#
#
if __name__ == '__main__':
    masses = 1 * np.array([0.064, 0.125, 0.216, 0.343, 0.512, 0.729, 1.0,
                            1.331, 1.728, 2.197, 2.744, 4.096, 5.832, 8.0])
    stiffnesses = [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000,
                   11000, 12000, 13000, 14000, 16000, 18000, 20000]
    ys = []

    for mass in masses:
        ys.append(find_stiffness(mass, 20000, stiff_ref=20000.0))
    # for stiff in stiffnesses:
    #     ys.append(find_stiffness(MASS, stiff, stiff_ref=4000.0))

    # import numpy
    # import numpy.polynomial.polynomial as poly
    # print(poly.Polynomial.fit(xs, ys, 4))
