from .. import utils


def create_space(dt, gravity=(0, 0), n_substep=5, engine='cpp', **kwargs):
    if engine.lower() == 'cpp':
        from .cpp import Space as Space
        space_cls = Space
    elif engine.lower() == 'cython':
        from .cython import Space as CythonSpace
        space_cls = CythonSpace
    elif engine.lower() == 'box2d':
        from .box2d import Box2DSpace
        space_cls = Box2DSpace

    else:
        raise ValueError('unknown engine "{}"'.format(engine))
    return space_cls(dt, gravity=gravity, n_substep=n_substep, **kwargs)
