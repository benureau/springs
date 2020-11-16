"""A simple script demonstrating how section can grow in height.

Four sections start with different birth size, but end up with the same adult form.
"""

import math
import random

import springs




random.seed(0)
space = springs.create_space(dt=0.01, gravity=0.0, engine='cython')

# base_L = space.add_node(1920/4-35, 1080/4-35, fixed=True)
# base_R = space.add_node(1920/4+35, 1080/4-35, fixed=True)
# space.add_link(base_L, base_R)
#
# top_L = space.add_node(1920/4-35, 1080/4+35, fixed=False)
# top_R = space.add_node(1920/4+35, 1080/4+35, fixed=False)
# space.add_link(top_L, top_R)
#
#
# space.add_link(top_L, base_R)
# space.add_link(base_L, top_R)

# section = springs.creatures.Section(space, (base_L, base_R), 70, 70)


if __name__ == '__main__':
    x = 0
    base_L, base_R, top_L, top_R = None, None, None, None

    def update(space):
        global base_L, base_R, top_L, top_R

        if space.ticks == 2:
            base_L = space.add_node(1920/4-35, 1080/4-35, fixed=True)
            base_R = space.add_node(1920/4+35, 1080/4-35, fixed=True)
            space.add_link(base_L, base_R)
        if space.ticks == 4:
            top_L = space.add_node(1920/4-35, 1080/4+35, fixed=False)
            top_R = space.add_node(1920/4+35, 1080/4+35, fixed=False)
            space.add_link(top_L, top_R)
        elif space.ticks == 6:
            space.add_link(top_L, base_R)
        elif space.ticks == 8:
            space.add_link(base_L, top_R)

    space.add_update_function(update)
    springs.create_display(space, width=1920, height=1080, zoom=2.0, hud=False,
                           duration=0.1,
                           screenshot_folder='/Users/fcyb/Data/Screens/201911_movie/section_one/section_one').run()
