"""Verifying that linear changes in stiffness induce linear changes in force."""

import math
import random
import springs


ENGINE = 'cpp'
MASS = 0.1
DT = 0.005

def create_link(space, x, mass, stiffness, damping_ratio=0.0, y=200):
    n_a = space.add_node(x, y + 10, mass=mass, fixed=True)
    n_b = space.add_node(x, y, mass=mass)
    m = space.add_spring(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    return n_b

if __name__ == '__main__':
    random.seed(0)
    print(ENGINE)
    space = springs.create_space(dt=DT, gravity=-100.0, engine=ENGINE, n_substep=1)

    nodes = []
    for i, stiffness in enumerate([100000, 2, 3, 4, 5, 6, 7, 8]):
        n = create_link(space, 20*(i+2), MASS, stiffness)
        nodes.append((stiffness, n))

    # springs.create_display(space, width=1000, height=1000).run()

    unstable_links = set()
    for t in range(500):
        space.step()
        for link in space.links:
            d = link.length()
            if d == float('inf'):
                print(t, 'spring length is infinite')
            elif math.isnan(d):
                print(t, 'spring length is nan')
            elif link.max_length > 10 * link.relax_length:
                unstable_links.add(link)
                print(t, 'unstable spring')
