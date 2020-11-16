import springs



def create_link(space, mass, stiffness, damping_ratio=1.0, x=0, y=0):
    n_a = space.add_node(x, y + 100, mass=mass, fixed=False)
    space.add_rect(-1000, 1000, y + 50, y + 99, 0)
    n_b = space.add_node(x, y, mass=mass)
    # print(stiffness)
    m = space.add_link(n_a, n_b, stiffness, damping_ratio=damping_ratio)
    m.stiffness = stiffness
    return n_b

def stiffness_response(stiffness, mass=1.0, dt=0.001, n_substep=5, gravity=-500, engine='cpp'):
    return space


if __name__ == '__main__':
    spaces = []
    for i, engine in enumerate(['box2d', 'cpp']):
        space = springs.create_space(dt=0.005, n_substep=5, gravity=-500, engine=engine)
        n = create_link(space, 1.0, 4.0, damping_ratio=1.0, x=300 + 200*i, y=100)
        # space.add_rect(-1000, 1000, 0, 100, 0)
        spaces.append(space)


    springs.create_display(spaces, width=1000, height=1000, zoom=1.0, spf=0.025).run()
