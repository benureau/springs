import springs
from springs.creatures.tentacles import _even_divide, Tentacle


def test_even_divide():
    assert _even_divide([1, 2, 3, 4, 5], 3) == [[1, 2], [3, 4], [5]]
    assert _even_divide([1, 2], 2) == [[1], [2]]
    assert _even_divide([1, 2], 1) == [[1, 2]]

    for i in range(1, 10):
        assert _even_divide([], i) == []

def test_tentacle():
    space = springs.create_space(dt=0.01, gravity=-100.0, engine='cython')
    jl = space.add_node(640, 100, fixed=True)
    jm = space.add_node(650, 100, fixed=True)
    jr = space.add_node(660, 100, fixed=True)

    heights = [40.0, 40.0, 40.0, 40.0]
    widths  = [20.0, 20.0, 20.0, 20.0]

    tentacle_1 = Tentacle(space, (jl, jr), heights, widths)
    assert len(tentacle_1.muscles) == 8
    tentacle_2 = Tentacle(space, (jl, jm, jr), heights, widths,
                          section_cls=springs.CentralBoneSection)
    assert len(tentacle_2.muscles) == 8


if __name__ == '__main__':
    test_even_divide()
    test_tentacle()
