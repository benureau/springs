"""Verifying that linear changes in stiffness induce linear changes in force."""

import math
import random
import springs


ENGINE = 'cpp'
MASS = 1.0
DT = 0.0001

def create_link(space, x, mass, stiffness, damping_ratio=1.0, y=440):
    n_a = space.add_node(x, y + 100, mass=mass, fixed=True)
    n_b = space.add_node(x, y, mass=mass)
    m = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m.stiffness = stiffness
    return n_b

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

if __name__ == '__main__':
    random.seed(0)
    print(ENGINE)
    space = springs.create_space(dt=DT, gravity=-10.0, engine=ENGINE)

    # n_ref = create_link(20, MASS, 4000)
    nodes = []
    # [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000, 20000]

    # for i, stiffness in enumerate([1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000,
    #                                9000, 10000, 11000, 12000, 13000, 14000, 20000]):
    for i, stiffness in enumerate([1000]):
        # inv_mass = 1/(1/1.0 + 1/MASS)
        # omega = math.sqrt(stiffness * inv_mass)
        # damping = 2 / inv_mass * 1.0 * omega
        # gamma = 1/ ( DT * (damping + DT * stiffness) + 0.0000001)
        # inv_mass += gamma
        # real_stiffness = gamma/inv_mass * stiffness

        print(mass_gamma(MASS, 1.0, stiffness))
        n = create_link(space, 20*(i+2), MASS, stiffness)
        nodes.append((stiffness, n))

    for _ in range(500):
        space.step()
    print(mass_gamma(MASS, 1.0, stiffness))

    # print('{}: {:.4f}'.format(4000, (440 - n_ref.y) * 4000, ))
    for stiffness, node in nodes:
        print('{}: {:.4f}'.format(stiffness, (440 - node.y) * stiffness))
    #springs.create_display(space, width=1000, height=1000).run()
