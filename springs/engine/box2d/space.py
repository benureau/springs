import math

import Box2D as b2


def distance(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)



class Node:

    def __init__(self, space, x, y, mass=1.0, friction=0.0, fixed=False):
        self.space, self.friction, self.fixed = space, friction, fixed
        self.mass = 0.0 if fixed else mass
        self.inv_mass = 0.0 if self.mass == 0.0 else 1/self.mass

        f = space.b2world.CreateStaticBody if fixed else space.b2world.CreateDynamicBody
        self.node = f(position=(x, y), allowSleep=False, fixedRotation=True)
        radius  = 0.0001
        density = mass / (math.pi * radius * radius)
        circle = self.node.CreateCircleFixture(radius=radius, density=density,
                                               friction=friction, categoryBits=0x0002, maskBits=0x0004)

    @property
    def position(self):
        return self.node.position

    @property
    def velocity(self):
        return self.node.velocity

    @property
    def x(self):
        return self.node.position.x

    @property
    def y(self):
        return self.node.position.y

    @property
    def v_x(self):
        return self.node.velocity.x

    @property
    def v_y(self):
        return self.node.velocity.y

    def translate(self, dx, dy):
        self.node.position[0] += dx
        self.node.position[1] += dx


class Link:
    """Passive link between two bodies.

    Depending on the parameter values, it can simulate flexibe springs or rigid bones, and it can
    be actuated, by modifying the expand factor, relative to a set relax length.

    :param space:          the space the link is to be added to.
    :param body_a:         first body the link it attached to.
    :param body_b:         the other body the link it attached to.
    :param stiffness:      stiffness of the link.
    :param damping_ratio:  damping ratio of the link.
    :param actuated:       if True, the expand_factor can be changed from its 1.0 neutral value.
    """
    def __init__(self, space, node_a, node_b, stiffness=100, damping_ratio=1, actuated=False):
        self.node_a, self.node_b = node_a, node_b
        self.b2link = space.b2world.CreateDistanceJoint(bodyA=node_a.node, bodyB=node_b.node,
                                                        localAnchorA=(0, 0), localAnchorB=(0, 0))
        self.stiffness = stiffness
        self.b2link.dampingRatio = damping_ratio
        self.actuated = actuated

        self.expand_factor = 1.0
        self._relax_length = distance(node_a.position, node_b.position)

    # motor behavior

    @property
    def relax_length(self):
        return self.b2link.length

    @relax_length.setter
    def relax_length(self, value):
        self._relax_length = value
        self.contract(self.expand_factor)

    def contract(self, expand_factor):
        self.expand_factor = expand_factor
        self.b2link.length = self.expand_factor * self._relax_length

    def relax(self):
        self.contract(1.0)

    # spring behavior

    @property
    def stiffness(self):
        return self.b2link.k

    @stiffness.setter
    def stiffness(self, val):
        inv_mass = self.node_a.inv_mass + self.node_b.inv_mass
        self.b2link.frequency = math.sqrt(val * inv_mass) / 6.28318530718

    @property
    def frequency(self):
        return self.b2link.frequency

    @frequency.setter
    def frequency(self, val):
        self.b2link.frequency = val

    @property
    def damping_ratio(self):
        return self.b2link.dampingRatio

    @damping_ratio.setter
    def damping_ratio(self, val):
        self.b2link.damping_ratio = val

class Rect:

    def __init__(self, space, xL, xR, yB, yT, restitution):
        assert xL < xR and yB < yT
        self.xL, self.xR, self.yB, self.yT = xL, xR, yB, yT
        self.width, self.height = self.xR - self.xL, self.yT - self.yB
        self.restitution = restitution

        shape = b2.b2PolygonShape(box=(self.width, self.height))
        fixture_def = b2.b2FixtureDef(shape=shape, restitution=self.restitution, categoryBits=0x0004, maskBits=0x0002)
        body = space.b2world.CreateStaticBody(position=(self.xL, self.yB))
                                              #0.5 * (self.xL + self.xR),
                                                #        0.5 * (self.yB + self.yT)))
        body.CreateFixture(fixture_def)


class Box2DSpace:

    def __init__(self, dt=0.01, n_substep=5, gravity=(0.0, 0.0), restitution_threshold=1.0):
        self.dt = dt
        self.n_substep = n_substep
        try:
            self.gravity_x, self.gravity_y = gravity
        except TypeError:
            self.gravity_x, self.gravity_y = 0, gravity

        self.t, self.ticks  = 0, 0

        self.nodes, self.links, self.rects, self.triangles = [], [], [], []
        self.entities, self.update_functions = [], []
        self.sensors = []

        self.restitution_threshold = restitution_threshold  # WARNING: not used yet.
        self.b2world = b2.b2World((self.gravity_x, self.gravity_y))

    @property
    def gravity(self):
        return self.gravity_x, self.gravity_y

    def add_node(self, x, y, mass=1.0, friction=0.5, fixed=False):
        node = Node(self, x, y, mass=mass, friction=friction, fixed=fixed)
        self.nodes.append(node)
        return node

    def add_rect(self, xL, xR, yB, yT, restitution):
        rect = Rect(self, xL, xR, yB, yT, restitution)
        self.rects.append(rect)
        return rect

    def add_link(self, node_a, node_b, stiffness=10000.0, damping_ratio=1.0, actuated=False):
        link = Link(self, node_a, node_b, stiffness=stiffness, damping_ratio=damping_ratio,
                          actuated=actuated)
        self.links.append(link)
        return link

    def add_entity(self, entity):
        self.entities.append(entity)

    def add_update_function(self, update_function):
        """Add a function to be run each step. Take the space as argument."""
        self.update_functions.append(update_function)

    def step(self):
        for update_function in self.update_functions:
            update_function(self)
        self.b2world.Step(self.dt, self.n_substep, 1)
        self.t += self.dt

        for sensor in self.sensors:
            sensor.update()

        for entity in self.entities:
            entity.update(self.t)
