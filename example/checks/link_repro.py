"""A simple script to track the trajectory of a soft link"""

import random

import springs


for engine in ['box2d', 'cpp']:
    print('\n{} -----------------------\n'.format(engine))
    space = springs.create_space(dt=0.05, gravity=-10.0, engine=engine)

    a = space.add_node(100, 100, 10.0, fixed=True)
    b = space.add_node(100,   0, 10.0, fixed=False)
    m = space.add_link(a, b, stiffness=1.0, damping_ratio=1.0)

    def update(space):
        pass

    space.add_update_function(update)

    if __name__ == '__main__':
        # m.contract(0.5)
        for t in range(10):
            space.step()
            print(' ---- {}'.format(t))
