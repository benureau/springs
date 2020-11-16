"""A simple script demonstrating the Squares class"""

import numpy as np
import random

import springs


np.random.seed(1)
space = springs.create_space(dt=0.01, gravity=-10.0, engine="cpp")
space.add_rect(-10000, 10000, -100, 100, restitution=0.9)

shape = (10, 3)

def create_square(x):
    square_size      = np.random.uniform(15, 30, size=shape)
    square_stiffness = np.random.uniform(0.0, 1.0, size=shape)
    amplitudes       = np.random.uniform(-1.0, 1.0, size=shape[0]*shape[1])
    phases           = np.random.uniform(-np.pi, np.pi, size=shape[0]*shape[1])

    square = springs.creatures.FusedSquares(space, square_size, square_stiffness, origin=(x, 120),
                                            amp_limit=0.5, amp_factor=0.5, α_freq=20.0, β_freq=10.0, γ_stif=4)

    def update(space):
        m_signal = [amp*np.sin(space.t + phase) for amp, phase in zip(amplitudes, phases)]
        square.actuate(m_signal)

    space.add_update_function(update)

SEED_OFFSET = 20

for i in range(7):
    np.random.seed(SEED_OFFSET + i)
    create_square(400*i + 100)

if __name__ == '__main__':
    springs.create_display(space, width=2000, height=300, zoom=0.5, spf=0.1).run()
