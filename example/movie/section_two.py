"""
To encode into a movie, in the screenshot folder:
ffmpeg -r 50 -f image2 -s 1920x1080 -i shot%05d.png -vcodec libx265 -preset medium -tag:v hvc1 -crf 25 -pix_fmt yuv420p section_two.mp4
"""

import math
import random

import springs




random.seed(0)
space = springs.create_space(dt=0.01, gravity=0.0, engine='cython')

base_L = space.add_node(1920/4-35, 1080/4-35, fixed=True)
base_R = space.add_node(1920/4+35, 1080/4-35, fixed=True)
space.add_link(base_L, base_R)
#
# top_L = space.add_node(1920/4-35, 1080/4+35, fixed=False)
# top_R = space.add_node(1920/4+35, 1080/4+35, fixed=False)
# space.add_link(top_L, top_R)
#
#
# space.add_link(top_L, base_R)
# space.add_link(base_L, top_R)

#section = springs.creatures.Section(space, (base_L, base_R), 70*0.25, 70)
section = springs.creatures.Section(space, (base_L, base_R), 70*0.5, 70)


if __name__ == '__main__':
    x = 0

    def update(space):
        global x
        BEND = 0.4
        if x < 1.5 * math.pi:
            speed = 0.25
        elif x < 4.5 * math.pi:
            speed = 0.5
        else:
            speed = 1.0
        x += 0.01 * speed

        section.muscles[0].contract(1.00 - BEND * (math.sin(x)))
        section.muscles[1].contract(1.00 + BEND * (math.sin(x)))

    space.add_update_function(update)
    springs.create_display(space, width=1920, height=1080, zoom=2.0, hud=False,
                           duration=60, spf=0.04,
                           screenshot_folder='/Users/fcyb/Data/Screens/201911_movie/section_two/section_two'
                          ).run()
