import numpy as np
import sys
import math
import random

from springs import creatures
import springs

GRAVITY    = (-0.1, -100.0)
FLOOR      = True
MASS       = 1.0
FRICTION   = 0.9
GROWTH     = False
BIRTH_SIZE = 1.0
MUSCLE_N_GROUP = 2
HEADLESS   = False


def create_space(engine):
    random.seed(1)  # reproducible results

    # creating the physic engine, `Space`.
    space = springs.create_space(dt=0.001, gravity=GRAVITY, engine=engine, n_substep=1)
    if FLOOR:
        space.add_rect(-10000, 10000, -100, 100, 0.9)

    # morphology of the starfish
    K = 8
    arm_dims = 5 * [[(40, 30) for i in range(K)] + [(40, 0)]]

    materials = {
        'node_center' : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_tip'    : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True},
        'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

    starfish  = creatures.DevStarfish(space, arm_dims, (1000, 500), 30, materials=materials,
                                      muscle_n_groups=MUSCLE_N_GROUP, height_dev_factor=BIRTH_SIZE,
                                      section_cls=springs.Section,
                                      # section_cls=springs.CentralBoneSection,
                                     )
    space.add_entity(starfish)

    speeds = np.array([random.uniform(0.2, 0.5) for _ in range(len(starfish.muscle_interface))])
    # controller
    def starfish_controller(starfish, space):
        if GROWTH:
            dev_size = min(1.0, BIRTH_SIZE + space.t/60)
            starfish.change_height_dev_factor(dev_size)

        current_length = 0.1 * np.sin(speeds * space.t)
        starfish.muscle_interface.actuate(current_length)

    starfish.add_controller(starfish_controller)

    return space


if __name__ == '__main__':
#    spaces = [create_space('cython')]
    import argparse
    parser = argparse.ArgumentParser(description='Compare engine behavior')
    parser.add_argument('engine', default='both')
    args = parser.parse_args()

    if args.engine == 'both':
        engines = ['cpp', 'cython']
        spaces = [create_space('cpp'), create_space('cython')]

    elif args.engine == 'cpp':
        engines = ['cpp']
        spaces = [create_space('cpp')]

    elif args.engine == 'cython':
        engines = ['cython']
        spaces = [create_space('cython')]


    if HEADLESS:
        for i in range(100):
            for engine, space in zip(engines, spaces):
                print('\n{}:{}'.format(engine, i))
                space.step()
        # for t in range(5000):
        #     for space in spaces:
        #         space.step()

    else:
        springs.create_display(spaces, width=1440, height=900, zoom=0.75, spf=0.05,
                               paused=False).run()
