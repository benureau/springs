import numpy as np
import sys
import math
import random

import springs


GRAVITY = 0.0
FLOOR   = True


random.seed(0)  # reproducible results
space = springs.create_space(dt=0.01, gravity=-10.0, engine="cpp")

# morphology of the starfish
size = 0.5
arm_dims = 25 * [[(50, 30), (45, 30), (40, 30), (40, 25)]+ [(30, 0)]]  # + 9 * [(35, 20)]

materials = {'section_diag'    : ('link', {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False}),
             'section_big_diag': ('link', {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False}),
             'section_center'  : ('link', {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False}),
             'section_width'   : ('link', {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False}),
             'section_side'    : ('link', {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True}),
             'tip'             : ('link', {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}),
             'center'          : ('link', {'stiffness':  5000.0, 'damping_ratio': 1.0, 'actuated': False})}

starfish = springs.creatures.Centipede(space, arm_dims, (1200, 600), 40,
                                       materials=materials, muscle_n_groups=4)
# creating the muscle interface
starfish.create_muscle_interface(n_group=4, muscle_cls=springs.creatures.motors.SectionOneMuscle)
space.add_entity(starfish)
if FLOOR:
    space.add_rect(-10000, 10000, -100, 100, restitution=0.9)

# controller
speeds = np.array([random.uniform(0.4, 1.0) for _ in range(len(starfish.muscle_interface))])
def starfish_controller(starfish, space):
    # for i in range(len(current_length)//2):
    #     current_length[2*i + 1] *= 2
    #     current_length[2*i + 1] += 1.0
    current_length = 0.1 * np.sin(speeds * space.t)
    starfish.muscle_interface.actuate(current_length)

starfish.add_controller(starfish_controller)


if __name__ == '__main__':
    springs.create_display(space, width=2000, height=300, zoom=0.5, spf=0.1).run()
