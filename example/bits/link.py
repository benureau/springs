"""A simple script to track the trajectory of a soft link"""

import random
import argparse

import springs


parser = argparse.ArgumentParser()
parser.add_argument('engine', help="physics engine to use", default='box2d')
args = parser.parse_args()


random.seed(0)
space = springs.create_space(dt=0.0001, gravity=0.0, engine=args.engine)

a = space.add_node(100, 100, 1.0, fixed=False)
b = space.add_node(100,   0, 1.0, fixed=False)
m = space.add_link(a, b, stiffness=10.0, damping_ratio=1.0)

def update(space):
    pass

space.add_update_function(update)

if __name__ == '__main__':
    m.contract(0.5)

    springs.create_display(space, width=900, height=300, autotranslate=False,
                           origin_y=-100, paused=False).run()
    # for t in range(5):
    #     space.step()
    #     print(' ---- {}'.format(t))
