import math
import springs


ENGINE = 'cpp'
DT = 0.005

def create_link(space, x, mass, stiffness, damping_ratio=1.0, y=440):
    n_a = space.add_node(x, y + 100, mass=mass, fixed=True)
    n_b = space.add_node(x, y, mass=mass)
    m = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m.stiffness = stiffness
    return n_b

def displacement(stiffness, mass):
    space = springs.create_space(dt=DT, gravity=-500.0, engine=ENGINE)
    y = 500
    node = create_link(space, 20, mass, stiffness, y=y)
    for _ in range(100000):
        space.step()
    return (y - node.y)

for mass in [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0]:
    print(displacement(1000, mass)/mass)
