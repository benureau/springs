import numpy as np
import sys
import math
import random

from springs import creatures
import springs

GRAVITY = (0.0, -100.0)
FLOOR   = True
NODE_MASS = 1.0
BIRTH_SIZE = 1.0
MUSCLE_N_GROUP = 2
GROWTH = False
HEADLESS = False


#random.seed(1)  # reproducible results


# creating the physic engine, `Space`.
space = springs.create_space(dt=0.001, gravity=GRAVITY, engine='cpp')
if FLOOR:
    space.add_rect(-10000, 10000, -100, 100, 0.5)
    # rough floor
    # for i in range(-500, 500):
    #     space.add_rect(20*i - 1, 20*(i+1) + 1, -500, random.uniform(79, 80), 0.5)

# morphology of the starfish
K = 8
arm_dims = 5 * [[(40, 30) for i in range(K)] + [(40, 0)]]
materials = {
    'node_center' : {'mass': NODE_MASS, 'friction': 0.5, 'fixed': False},
    'node_section': {'mass': NODE_MASS, 'friction': 0.5, 'fixed': False},
    'node_tip'    : {'mass': NODE_MASS, 'friction': 1.0, 'fixed': False},
    'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True},
    'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
    'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}


sensor_cfg = {'sensors': {'touch': 'muscle_group_sides', 'angle': 'muscle_group',
                          'angular_velocity': True, 'center_angle': True}}


starfish  = creatures.DevStarfish(space, arm_dims, (0, 500), 30, materials=materials,
                                  # section_cls=springs.CentralBoneSection,
                                  section_cls=springs.Section,
                                  muscle_n_groups=MUSCLE_N_GROUP, height_dev_factor=BIRTH_SIZE,
                                  sensor_cfg=sensor_cfg)
min_y = min(node.y for node in starfish.nodes)
starfish.translate(0, -min_y + 100)
space.add_entity(starfish)

# controller
class Network:

    def __init__(self, dims):
        self.dims = dims
        self.layer_weights = [(np.random.randn(y, x), np.random.randn(y, 1))
                               for x, y in zip(self.dims[:-1], self.dims[1:])]

    def forward(self, X):
        """Compute the network's output"""
        for W, B in self.layer_weights:
            # print(X.shape, W.shape, B.shape)
            # print(np.dot(W, X).shape)
            X = np.tanh(np.dot(W, X) + B)
        return X

print('{} sensors'.format(len(space.sensor_values())))
print('{} outputs'.format(len(starfish.muscle_interface)))
nn = Network([len(space.sensor_values()), 20, 20, len(starfish.muscle_interface)])

# speeds = np.array([random.uniform(-1.0, 1.0) for _ in range(len(starfish.muscle_interface))])
def starfish_controller(starfish, space):

    if GROWTH:
        dev_size = min(1.0, BIRTH_SIZE + space.t/200)
        starfish.change_height_dev_factor(dev_size)

    inputs = np.array(space.sensor_values())
    inputs = np.expand_dims(inputs, axis=1)
    current_length = 0.1 * nn.forward(inputs)
    starfish.muscle_interface.actuate(current_length)

starfish.add_controller(starfish_controller)


if __name__ == '__main__':

    if HEADLESS:
        for t in range(5000):
            space.step()
    else:
        springs.create_display(space, width=1440, height=900, zoom=0.75, spf=0.05,
                               paused=False).run()

    # import line_profiler
    #profile = line_profiler.LineProfiler(springs.engine.cython.cython.Link.substep)
    # profile = line_profiler.LineProfiler(springs.engine.cython.cython.Space.step)
    # profile.runcall(bla)
    # profile.print_stats()
