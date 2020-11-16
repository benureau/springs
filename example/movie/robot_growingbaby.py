import numpy as np
import sys
import math
import random

from springs import creatures
import springs

FLOOR   = False
MUSCLE_N_GROUP = 2
BIRTH_SIZE = 0.25
GROWTH = True


random.seed(1)  # reproducible results

# creating the physic engine, `Space`.
space = springs.create_space(dt=0.001, gravity=0.0, engine="cython")
if FLOOR:
    space.add_rect(-10000, 10000, -100, 100, 0.9)

# morphology of the starfish
K = 8
arm_dims = 6 * [[(40, 30) for i in range(K)] + [(40, 0)]]

materials = {
    'node_center' : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
    'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
    'node_tip'    : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
    'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_side'    : {'stiffness':   3000.0, 'damping_ratio': 1.0, 'actuated':  True},
    'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

speeds = None
starfish  = creatures.DevStarfish(space, arm_dims, (960, 540), 30,
                                  materials=materials, muscle_n_groups=MUSCLE_N_GROUP,
                                  # section_cls=springs.CentralBoneSection,
                                  section_cls=springs.Section, height_dev_factor=BIRTH_SIZE)
space.add_entity(starfish)

# controller
if speeds is None:
    speeds = np.array([random.uniform(0.2, 0.5) for _ in range(len(starfish.muscle_interface))])
def starfish_controller(starfish, space):
    if GROWTH:
        dev_size = min(1.0, BIRTH_SIZE + max(0, space.t-10)/120)
        starfish.change_height_dev_factor(dev_size)

    current_length = 0.1 * np.sin(speeds * space.t)
    starfish.muscle_interface.actuate(current_length)

starfish.add_controller(starfish_controller)


if __name__ == '__main__':
    springs.create_display(space, width=1920, height=1080, zoom=1.0,
                           duration=60, spf=0.04,
                           screenshot_folder='/Users/fcyb/Data/Screens/201911_movie/growing/growing',
                           paused=False, hud=False).run()
