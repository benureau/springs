"""A simple script demonstrating the tentacle class"""

import math
import random

import springs
from springs import creatures


K = 8
SIZE = 70.0
BEND = 0.25
SECTION = springs.Section

random.seed(0)
space = springs.create_space(dt=0.001, gravity=0.0, engine='cython')

base_L = space.add_node(1920/2-35, 1080/3-35, fixed=True)
base_R = space.add_node(1920/2+35, 1080/3-35, fixed=True)
space.add_link(base_L, base_R)

heights = (K) * [SIZE]
widths  = K * [SIZE]

materials = {
    'node_center' : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
    'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
    'node_tip'    : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
    'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_side'    : {'stiffness':    4000.0, 'damping_ratio': 1.0, 'actuated':  True},
    'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

tentacle = creatures.Tentacle(space, (base_L, base_R), heights, widths, materials=materials,
                              section_cls=SECTION)

def update(space):
    if space.t < 2 * 8 * math.pi:
        for m in tentacle.left_muscles:
            m.contract(1.00 + BEND * (math.sin(space.t/8)))
        for m in tentacle.right_muscles:
            m.contract(1.00 - BEND * (math.sin(space.t/8)))
    else:
        for m in tentacle.left_muscles:
            m.contract(1.00)
        for m in tentacle.right_muscles:
            m.contract(1.00)

    # for m in tentacle.left_muscles[:2]:
    #     m.contract(1.00 + BEND * (math.sin(space.t/4)))
    # for m in tentacle.right_muscles[:2]:
    #     m.contract(1.00 - BEND * (math.sin(space.t/4)))
    # for m in tentacle.left_muscles[2:4]:
    #     m.contract(1.00 - BEND * (math.sin(space.t/3)))
    # for m in tentacle.right_muscles[2:4]:
    #     m.contract(1.00 + BEND * (math.sin(space.t/3)))
    # for m in tentacle.left_muscles[4:6]:
    #     m.contract(1.00 + BEND * (math.sin(space.t/2)))
    # for m in tentacle.right_muscles[4:6]:
    #     m.contract(1.00 - BEND * (math.sin(space.t/2)))
    # for m in tentacle.left_muscles[6:]:
    #     m.contract(1.00 - BEND * (math.sin(space.t/1)))
    # for m in tentacle.right_muscles[6:]:
    #     m.contract(1.00 + BEND * (math.sin(space.t/1)))


space.add_update_function(update)
springs.create_display(space, width=1920, height=1080, zoom=1.0, hud=False,
                       duration=60, spf=0.04,
                       screenshot_folder='/Users/fcyb/Data/Screens/201911_movie/tentacle/tentacle'
                      ).run()
