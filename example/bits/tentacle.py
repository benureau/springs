"""A simple script demonstrating the tentacle class"""

import math
import random
import argparse

import springs
from springs import creatures



# parser = argparse.ArgumentParser()
# parser.add_argument('engine', help="physics engine to use", default='box2d')
# args = parser.parse_args()

K = 8
MASS = 0.5
SIZE = 80.0
BEND = 2.0
DT = 0.005

SECTION = springs.CentralBoneSection  # FIXME: instability?
# SECTION = springs.Section

def update(space):
    tentacle = space.entities[0]
    for m in tentacle.left_muscles:
        m.contract(1.00 + BEND * (math.sin(space.t/4)))
    for m in tentacle.right_muscles:
        m.contract(1.00 - BEND * (math.sin(space.t/4)))
    tip = tentacle.tip.new_nodes[0]


random.seed(0)
spaces = []
#for engine in ['cpp']:
for i, engine in enumerate(['cpp', 'cython', 'box2d']):
    space = springs.create_space(dt=DT, gravity=0.0, engine=engine)

    x_0, y_0 = 400, 500
    jr = space.add_node(x_0, y_0 + 5*i         , 1, fixed=True)
    jl = space.add_node(x_0, y_0 + 5*i + SIZE  , 1, fixed=True)
    base = (jl, jr)
    if SECTION == springs.CentralBoneSection:
        jm = space.add_node(x_0, y_0 + SIZE/2, 1, fixed=True)
        base = (jl, jm, jr)

    heights = (K + 1) * [SIZE]
    widths  = K * [SIZE]

    materials = {
        'node_center' : {'mass': MASS, 'friction': 0.5, 'fixed': False},
        'node_section': {'mass': MASS, 'friction': 0.5, 'fixed': False},
        'node_tip'    : {'mass': MASS, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  10000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   5000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   5000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   1200.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    100.0, 'damping_ratio': 3.0, 'actuated':  True},
        'link_center'      : {'stiffness':  50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_tip'         : {'stiffness':  50000.0, 'damping_ratio': 1.0, 'actuated': False}}

    tentacle = creatures.Tentacle(space, base, heights, widths, materials=materials,
                                  section_cls=SECTION)

    space.add_entity(tentacle)
    space.add_update_function(update)
    spaces.append(space)

if __name__ == '__main__':
    # for dt in range(2):
    #     space.step()

    springs.create_display(spaces, width=1000, height=1000, zoom=0.8, spf=0.025).run()
