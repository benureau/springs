import time
import math
import random

import springs


def benchmark(engine, n=20000, duration=None):
    random.seed(0)  # reproducible results

    # creating the physic engine, `Space`.
    space = springs.create_space(dt=0.001, gravity=-100.0, n_substep=1, engine=engine)

    # morphology of the starfish
    materials = {
        'node_center' : {'mass': 0.1, 'friction': 0.5, 'fixed': False},
        'node_section': {'mass': 0.1, 'friction': 0.5, 'fixed': False},
        'node_tip'    : {'mass': 0.1, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True},
        'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}
    size = 0.5
    arm_dims = 6 * [[(50, 30), (45, 30), (40, 30), (40, 25)] + 2 * [(35, 20)] + [(30, 0)]]
    starfish = springs.creatures.Starfish(space, arm_dims, (720, 500), 40,
                                          muscle_n_groups=4)
    starfish.create_muscle_interface(n_group=4, muscle_cls=springs.creatures.motors.SectionOneMuscle)
    space.add_entity(starfish)
    # space.add_rect(-10000, 10000, -10000, 100, restitution=0.5)
    for i in range(-500, 500):
        space.add_rect(20*i - 1, 20*(i+1) + 1, 100, random.uniform(170, 190), 0.5)

    # controller
    tick_period = round(0.05 / space.dt)

    speeds = [random.uniform(0.2, 0.6) for _ in range(len(starfish.muscle_interface))]
    def starfish_controller(starfish, space):
        if space.ticks % tick_period == 0:
            current_length = [0.1 * math.sin(speed * space.t) for speed in speeds]
            starfish.muscle_interface.actuate(current_length)
    starfish.add_controller(starfish_controller)

    return benchmark_space(space, engine, n=n, duration=duration)


def benchmark_space(space, engine, n=20000, duration=None):
    start = time.time()
    count = 0
    if duration is None:
        for _ in range(n):
            space.step()
            count += 1
    else:
        while time.time() - start < duration:
            space.step()
            count += 1
    duration = time.time() - start
    perf = count/duration
    speedup = count * space.dt / duration
    print('{}: {} step/s (averaged over {:.2f} seconds, speedup {:.1f}x)'.format(engine, round(perf), duration, speedup))
    return perf, duration


if __name__ == '__main__':
    benchmark('cpp', duration=5.0)
    benchmark('cython', duration=5.0)
