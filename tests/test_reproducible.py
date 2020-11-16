import os
import random
import argparse

import numpy as np

import springs


active_engines = ['cpp', 'cython']

def test_cython_cpp_repro(args=None):
    def create_trace(engine, strides=100, node_strides=13):
        random.seed(1)  # reproducible results

        space = springs.create_space(dt=0.01, gravity=(-1.0, -100.0), engine=engine)
        space.add_rect(-10000, 10000, 0, 100, 0.9)

        arm_dims = 5 * [[(40, 30) for i in range(8)] + [(40, 0)]]
        materials = {
            'node_center' : {'mass': 2.0, 'friction': 0.5, 'fixed': False},
            'node_section': {'mass': 2.0, 'friction': 0.5, 'fixed': False},
            'node_tip'    : {'mass': 2.0, 'friction': 0.5, 'fixed': False},
            'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
            'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
            'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
            'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
            'link_sec_side'    : {'stiffness':    5000.0, 'damping_ratio': 1.0, 'actuated':  True},
            'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
            'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

        starfish  = springs.creatures.DevStarfish(space, arm_dims, (1000, 500), 30,
                        muscle_n_groups=2, section_cls=springs.Section, height_dev_factor=1.0,
                        materials=materials)
        space.add_entity(starfish)

        speeds = np.array([random.uniform(0.2, 0.5) for _ in range(len(starfish.muscle_interface))])
        def starfish_controller(starfish, space):
            current_length = 0.1 * np.sin(speeds * space.t)
            starfish.muscle_interface.actuate(current_length)

        starfish.add_controller(starfish_controller)

        trace = []
        def update(space):
            trace.append([(node.x, node.y, node.v_x, node.v_y) for node in space.nodes][::node_strides])
        space.add_update_function(update)

        for t in range(1000):
            space.step()
        trace = trace[::strides]
        return np.array(trace)

    def compare_traces(name_a, trace_a, name_b, trace_b):
        match = True
        for i, (t_i_a, t_i_b) in enumerate(zip(trace_a, trace_b)):
            if match and not np.array_equal(t_i_a, t_i_b):
            # if not np.allclose(t_i_cpp, t_i_cython):
                max_diff = np.max(np.abs(np.array(t_i_a) - np.array(t_i_b)))
                print('{} and {} traces differs at step {} (max diff: {})'.format(name_a, name_b, i, max_diff))
                match = False
                # break

        if not match:
            max_diff = np.max(np.abs(t_i_a - t_i_b))
            print(' -> final diff at step {}: {}'.format(i, max_diff))

        if match:
            print('{} and {} traces match'.format(name_a, name_b))
        else:
            print('{} and {} traces DO NOT match'.format(name_a, name_b))
        return match

    traces_match = True
    traces = {}

    # trace_cython = create_trace('cython')
    for engine in active_engines:
        traces[engine] = create_trace(engine)
    # traces_match = traces_match and compare_traces('cython', trace_cython, 'cpp', trace_cpp)

    path = os.path.join(os.path.dirname(__file__), 'engine_trace.npy')
    if args is not None and args.replace:
        print("replacing saved trace by the current trace")
        np.save(path, traces[active_engines[0]])
    else:
        old_trace = np.load(path)
        # for engine, trace in zip(['cpp', 'cython'], [trace_cpp, trace_cython]):
        for engine in active_engines:
            trace = traces[engine]
            if trace.shape == old_trace.shape:
                trace_match = compare_traces(engine, trace, 'stored', old_trace)
                traces_match = traces_match and trace_match
            else:
                print('current {} trace shape does not match saved trace shape ({} != {})'.format(
                      engine, trace.shape, old_trace.shape))
                traces_match = False
    assert traces_match


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check (or create) a position trace.')
    parser.add_argument('--replace', action='store_true', default=False)
    args = parser.parse_args()

    test_cython_cpp_repro(args=args)
