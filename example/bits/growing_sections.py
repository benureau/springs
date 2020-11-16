"""A simple script demonstrating how section can grow in height.

Four sections start with different birth size, but end up with the same adult form.
"""

import math
import random

import springs

random.seed(0)
space = springs.create_space(dt=0.01, gravity=0.0, engine='cpp')

sections = []
K = 5
for i, height in enumerate([0.4, 0.7, 1.0]):
    base_L = space.add_node(200*i + 100, 0, fixed=True)
    base_R = space.add_node(200*i + 200, 0, fixed=True)
    space.add_link(base_L, base_R)  # cosmetic

    sections.append(springs.creatures.Section(space, (base_L, base_R), 100*height, 100))


if __name__ == '__main__':

    def update(space):
        pass
        for section in sections:
            section.height = min(100, section.height + 0.02)
        # print(section.link_map['diag_RL'].length())
    space.add_update_function(update)

    springs.create_display(space, width=900, height=300, autotranslate=False,
                           origin_y=-100, paused=False).run()
