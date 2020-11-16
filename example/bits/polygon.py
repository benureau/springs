import springs


def create_polygon(space, n, center_x, center_y, size):
    nodes = springs.utils.polygon(n, center_x, center_y, size)
    joints = [space.add_node(x, y) for x, y in nodes]
    for i, joint in enumerate(joints):
        space.add_link(joints[i-1], joint)



if __name__ == '__main__':
    space = springs.create_space(dt=0.001, engine="cpp")

    create_polygon(space,  3,  200, 200, 100)
    create_polygon(space,  4,  500, 200, 100)
    create_polygon(space,  5,  800, 200, 100)
    create_polygon(space,  6, 1100, 200, 100)
    create_polygon(space,  7,  200, 500, 100)
    create_polygon(space,  8,  500, 500, 100)
    create_polygon(space,  9,  800, 500, 100)
    create_polygon(space, 10, 1100, 500, 100)

    springs.create_display(space, width=1400, height=900, hud=False, paused=True, zoom=1.0).run()
