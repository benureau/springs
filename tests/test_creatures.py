import springs


def test_starfish_creation():

    space = springs.create_space(dt=0.001, gravity=-100.0, engine='cpp')

    arm_dims = 5 * [[(40, 30) for i in range(8)] + [(40, 0)]]
    materials = {
        'node_center' : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_tip'    : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True},
        'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

    starfish  = springs.creatures.DevStarfish(space, arm_dims, (1000, 500), 30, materials=materials,
                                              section_cls=springs.Section,
                                              # section_cls=springs.CentralBoneSection,
                                              muscle_n_groups=3, height_dev_factor=0.75)

    space = springs.create_space(dt=0.001, gravity=-100.0, engine="cpp")
    for dt in range(100):
        space.step()

# def test_centipede_creation():
#     pass



if __name__ == '__main__':
    test_starfish_creation()
