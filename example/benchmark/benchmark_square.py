"""A simple script demonstrating the Squares class"""

import numpy as np
import random

import springs

from benchmark import benchmark_space


def create_space(engine):
    np.random.seed(1)
    space = springs.create_space(dt=0.001, gravity=-100.0, engine=engine)
    space.add_rect(-10000, 10000, -100, 100, restitution=0.5)

    shape = (10, 3)

    def create_square(x, stif):
        square_size      = [[75.0]]
        square_stiffness = [[stif]]
        amplitudes       = [  1.0 ]
        phases           = [  0.0 ]

        square = springs.creatures.FusedSquares(space, square_size, square_stiffness, origin=(x, 120))

        def update(space):
            m_signal = [amp*np.sin(space.t + phase) for amp, phase in zip(amplitudes, phases)]
            square.actuate(m_signal)

        space.add_update_function(update)
        return square

    SEED_OFFSET = 0

    for i in range(11):
        np.random.seed(SEED_OFFSET + i)
        square = create_square(125*i + 50, i/10)
        # print(square.square_matrix[0][0].amp_factor)

    return space

def benchmark_squares(engine, n=20000, duration=None):
    space = create_space(engine)
    benchmark_space(space, engine, n=n, duration=duration)
    return space


if __name__ == '__main__':
    space = benchmark_squares('cpp', duration=5.0)
    springs.create_display(space, width=1440, height=900, zoom=0.75, spf=0.05,
                           paused=False).run()
