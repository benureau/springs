"""Verifying that linear changes in stiffness induce linear changes in force."""

import random
import springs


ENGINE = 'cpp'

def create_link(x, mass, stiffness, damping_ratio=1.0):
    n_a = space.add_node(x, 1010, mass=1.0, fixed=True)
    n_b = space.add_node(x, 1000, mass=mass)
    m_1 = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m_1.stiffness = stiffness
    n_c = space.add_node(x+10, 1010, mass=1.0, fixed=True)
    n_d = space.add_node(x+10, 1000, mass=mass)
    m_2 = space.add_spring(n_c, n_d, stiffness, damping_ratio=damping_ratio)

if __name__ == '__main__':
    random.seed(2)
    print(ENGINE)
    space = springs.create_space(dt=0.001, gravity=-100.0, engine=ENGINE, n_substep=1)

    for i in range(10):
        create_link(100*(i+0.5)     , 1.0*(i+1), 1.0)
        create_link(100*(i+0.5) + 30, 1.0      , 1.0/(i+1))
        create_link(100*(i+0.5) + 60, 1.0/(i+1), 1.0/(i+1)/(i+1))

    for i in range(10, 50):
        create_link(900 + 30*i, random.uniform(0.1, 2.0), random.uniform(0.1, 2.0), random.uniform(0.0, 1.5))


    springs.create_display(space, width=2500, height=1500, spf=0.05).run()
