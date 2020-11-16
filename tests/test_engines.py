import math
import random

import springs


def test_spring():

    spaces = [springs.create_space(dt=0.0001, gravity=-100.0, engine=engine)
              for engine in ['cpp', 'cython']]

    def close_position(a, b, threshold=0.5):
        #print(abs(b.x - a.x), abs(b.y - a.y))
        return abs(b.x - a.x) < threshold and abs(b.y - a.y) < threshold

    def create_link(space, mass, stiffness, damping_ratio):
        a = space.add_node(100, 200, 1.0, fixed=True)
        b = space.add_node(100, 100, mass=mass)
        spring = space.add_link(a, b, stiffness, damping_ratio=damping_ratio, actuated=True)
        return b

    nodes = []
    for space in spaces:
        space_nodes = []
        space_nodes.append(create_link(space, 1.0, 1.0, 0.5))
        space_nodes.append(create_link(space, 2.0, 0.5, 0.5))
        space_nodes.append(create_link(space, 0.5, 2.0, 0.5))
        nodes.append(space_nodes)

    for _ in range(1000):
        for i, space in enumerate(spaces):
            space.step()
            assert close_position(nodes[i][0], nodes[i][1])
            assert close_position(nodes[i][1], nodes[i][2])
            if i > 0:
                assert close_position(nodes[i-1][0], nodes[i][0])
                assert close_position(nodes[i-1][1], nodes[i][1])
                assert close_position(nodes[i-1][2], nodes[i][2])


def test_stiffness():
    for engine in ['cpp', 'cython']:
        space = springs.create_space(dt=0.0001, gravity=100.0, engine=engine)
        a = space.add_node(100, 200, mass=1.0, fixed=True)
        b = space.add_node(100, 100, mass=1.0)
        link = space.add_link(a, b, 1.0, 1.0, False)
        for i in range(100):
            mass = random.uniform(0.01, 100.0)
            stiffness = random.uniform(0.01, 100.0)
            b.mass = mass
            link.stiffness = stiffness
            assert abs(link.stiffness - stiffness) < 1e-6


def test_restitution():
    for engine in ['cython', 'cpp']:
        def check_restitution(speed, restitution, dt=0.01, threshold=0.0):
            space = springs.create_space(dt=dt, gravity=0.0, engine=engine,
                                         restitution_threshold=threshold)
            space.add_rect(-10000, 10000, -100, 0, restitution=restitution)

            n = space.add_node(0, 1, mass=1.0)
            n.v_y = -speed

            for _ in range(int(math.ceil(1/speed/dt)) + 10):
                space.step()
                #assert n.y >= 0
            if abs(speed) < threshold:
                assert n.v_y == 0 and n.y == 0
            else:
                assert n.y > 0
                assert abs(n.v_y - speed * restitution) < 1e-5

        random.seed(0)
        for _ in range(100):
            speed = random.uniform(0, 10.0)
            restitution = random.uniform(0, 10.0)
            check_restitution(speed, restitution, dt=0.01)

        for _ in range(100):
            speed = random.uniform(0, 10.0)
            restitution = random.uniform(0, 10.0)
            check_restitution(speed, restitution, dt=0.01, threshold=speed+1)


if __name__ == '__main__':
    test_spring()
    test_stiffness()
    test_restitution()
