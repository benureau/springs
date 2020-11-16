# cython: language_level=3, boundscheck=False
# distutils: language = c++
# cython: infer_types=True

from libcpp cimport bool
from libcpp.vector cimport vector


# objects
from space cimport Node   as CppNode
from space cimport Link   as CppLink
from space cimport Spring as CppSpring
from space cimport Space  as CppSpace
# sensors
from space cimport Sensor                as CppSensor
from space cimport AngleSensor           as CppAngleSensor
from space cimport AngularVelocitySensor as CppAngularVelocitySensor
from space cimport TouchSensor           as CppTouchSensor


# cdef extern from "math.h":
#     double sqrt(double m)

# cpdef double distance(Node a, Node b):
#     d_x, d_y = (a.x - b.x), (a.y - b.y)
#     return sqrt(d_x*d_x + d_y*d_y)


cdef class Node:
    """Just wrapping. The Node is constructed through Space.add_node"""

    cdef CppNode *c_node

    @property
    def x(self):
        return self.c_node.x

    @property
    def y(self):
        return self.c_node.y

    @property
    def v_x(self):
        return self.c_node.v_x

    @v_x.setter
    def v_x(self, double value):
        self.c_node.v_x = value

    @property
    def v_y(self):
        return self.c_node.v_y

    @v_y.setter
    def v_y(self, double value):
        self.c_node.v_y = value

    @property
    def position(self):
        return self.c_node.x, self.c_node.y

    @property
    def mass(self):
        return self.c_node.mass()

    @mass.setter
    def mass(self, double value):
        self.c_node.set_mass(value)

    @property
    def friction(self):
        return self.c_node.friction

    @friction.setter
    def friction(self, double value):
        self.c_node.friction = value

    @property
    def fixed(self):
        return self.c_node.fixed()

    @fixed.setter
    def fixed(self, bool value):
        self.c_node.set_fixed(value)

    cpdef translate(self, double dx, double dy):
        self.c_node.translate(dx, dy)



cdef class Link:

    cdef CppLink *c_link
    cdef readonly Node node_a, node_b

    def __cinit__(self, Node node_a, Node node_b):
        self.node_a = node_a
        self.node_b = node_b

    def length(self):
        return self.c_link.length()

    # motor behavior

    @property
    def actuated(self):
        return self.c_link.actuated

    @actuated.setter
    def actuated(self, bool value):
        self.c_link.actuated = value

    @property
    def relax_length(self):
        return self.c_link.relax_length

    @relax_length.setter
    def relax_length(self, value):
        self.c_link.relax_length = value

    @property
    def expand_factor(self):
        return self.c_link.expand_factor

    def contract(self, value):
        self.c_link.contract(value)

    def relax(self):
        self.c_link.relax()

    # spring behavior

    @property
    def stiffness(self):
        return self.c_link.stiffness()

    @stiffness.setter
    def stiffness(self, value):
        self.c_link.set_stiffness(value)

    @property
    def frequency(self):
        return self.c_link.frequency()

    @frequency.setter
    def frequency(self, value):
        self.c_link.set_frequency(value)

    @property
    def damping_ratio(self):
        return self.c_link.damping_ratio()

    @damping_ratio.setter
    def damping_ratio(self, value):
        self.c_link.set_damping_ratio(value)

    @property
    def max_impulse(self):
        return self.c_link.max_impulse

    @property
    def max_length(self):
        return self.c_link.max_length

    @property
    def force(self):
        return self.c_link.force()


cdef class Spring:

    cdef CppSpring *c_spring
    cdef readonly Node node_a, node_b

    def __cinit__(self, Node node_a, Node node_b):
        self.node_a = node_a
        self.node_b = node_b

    def length(self):
        return self.c_spring.length()

    # motor behavior

    @property
    def actuated(self):
        return self.c_spring.actuated

    @actuated.setter
    def actuated(self, bool value):
        self.c_spring.actuated = value

    @property
    def relax_length(self):
        return self.c_spring.relax_length

    @relax_length.setter
    def relax_length(self, value):
        self.c_spring.relax_length = value

    @property
    def expand_factor(self):
        return self.c_spring.expand_factor

    def contract(self, value):
        self.c_spring.contract(value)

    def relax(self):
        self.c_spring.relax()

    # spring behavior

    @property
    def stiffness(self):
        return self.c_spring.stiffness()

    @stiffness.setter
    def stiffness(self, value):
        self.c_spring.set_stiffness(value)

    @property
    def frequency(self):
        return self.c_spring.frequency()

    @frequency.setter
    def frequency(self, value):
        self.c_spring.set_frequency(value)

    @property
    def damping_ratio(self):
        return self.c_spring.damping_ratio()

    @damping_ratio.setter
    def damping_ratio(self, value):
        self.c_spring.set_damping_ratio(value)

    @property
    def max_impulse(self):
        return self.c_spring.max_impulse

    @property
    def max_length(self):
        return self.c_spring.max_length



cdef class Rect:
    """Hollow class for drawing purposes"""

    cdef readonly xL, xR, yB, yT, width, height, restitution

    def __cinit__(self, double xL, double xR, double yB, double yT, double restitution):
        self.xL, self.xR, self.yB, self.yT = xL, xR, yB, yT
        self.width, self.height = self.xR - self.xL, self.yT - self.yB
        self.restitution = restitution


cdef class Triangle:
    """Hollow class for drawing purposes"""

    cdef readonly xA, yA, xB, yB, xC, yC, restitution

    def __cinit__(self, double xA, double yA, double xB, double yB, double xC, double yC,
                        double restitution):
        self.xA, self.yA, self.xB, self.yB, self.xC, self.yC = xA, yA, xB, yB, xC, yC
        self.restitution = restitution


cdef class AngleSensor:
    cdef CppAngleSensor *c_sensor
    cdef readonly Node origin, satellite
    cdef readonly AngleSensor ref_sensor

    def __init__(self, Node origin, Node satellite, AngleSensor ref_sensor):
        self.origin    = origin
        self.satellite = satellite
        self.ref_sensor = ref_sensor

    @property
    def value(self):
        return self.c_sensor.value()

    def update(self):
        return self.c_sensor.update()


cdef class AngularVelocitySensor:
    cdef CppAngularVelocitySensor *c_sensor
    cdef readonly AngleSensor angle_sensor

    def __init__(self, AngleSensor angle_sensor):
        self.angle_sensor = angle_sensor

    @property
    def value(self):
        return self.c_sensor.value()

    def update(self):
        return self.c_sensor.update()


cdef class TouchSensor:
    cdef CppTouchSensor *c_sensor
    cdef readonly list nodes

    def __init__(self, list nodes):
        self.nodes = nodes

    @property
    def value(self):
        return self.c_sensor.value()

    def update(self):
        return self.c_sensor.update()


cdef class Space:

    cdef CppSpace *c_space
    cdef readonly object nodes, links, springs, rects, triangles, entities, update_functions
    cdef readonly list sensors

    def __cinit__(self, double dt, int n_substep=5, gravity=0.0, restitution_threshold=1.0):
        self.nodes, self.links, self.rects, self.triangles = [], [], [], []
        self.entities, self.update_functions = [], []
        self.sensors = []

        try:
            gravity_x, gravity_y = gravity
        except TypeError:
            gravity_x, gravity_y = 0, gravity

        self.c_space = new CppSpace(dt, n_substep, gravity_x, gravity_y, restitution_threshold)

    @property
    def ticks(self):
        return self.c_space.ticks

    @property
    def t(self):
        return self.c_space.t

    @property
    def dt(self):
        return self.c_space.dt()

    @dt.setter
    def dt(self, double value):
        self.c_space.set_dt(value)

    @property
    def gravity(self):
        return self.c_space.gravity_x, self.c_space.gravity_y

    @gravity.setter
    def gravity(self, value):
      try:
          self.c_space.gravity_x, self.c_space.gravity_y = value
      except TypeError:
          self.c_space.gravity_x, self.c_space.gravity_y = 0, value

    @property
    def restitution_threshold(self):
        return self.c_space.restitution_threshold

    @restitution_threshold.setter
    def restitution_threshold(self, double value):
        self.c_space.restitution_threshold = value

    cpdef Node add_node(self, double x, double y, double mass=1.0, double friction=0.5,
                              double fixed=False):
        c_node = self.c_space.add_node(x, y, mass, friction, fixed)
        node = Node()
        node.c_node = c_node
        self.nodes.append(node)
        return node

    cpdef Link add_link(self, Node node_a, Node node_b, double stiffness=10000.0,
                              double damping_ratio=1.0, bool actuated=False, double max_impulse=100):
        c_link = self.c_space.add_link(node_a.c_node, node_b.c_node,
                                       stiffness, damping_ratio, actuated, max_impulse)
        link = Link(node_a, node_b)
        link.c_link = c_link
        self.links.append(link)
        return link

    cpdef Spring add_spring(self, Node node_a, Node node_b, double stiffness=10000.0,
                                 double damping_ratio=1.0, bool actuated=False, double max_impulse=100):
        c_link = self.c_space.add_spring(node_a.c_node, node_b.c_node,
                                         stiffness, damping_ratio, actuated, max_impulse)
        link = Spring(node_a, node_b)
        link.c_spring = c_link
        self.links.append(link)
        return link

    cpdef Rect add_rect(self, double xL, double xR, double yB, double yT, double restitution):
        self.c_space.add_rect(xL, xR, yB, yT, restitution)
        rect = Rect(xL, xR, yB, yT, restitution)
        self.rects.append(rect)
        return rect

    cpdef Triangle add_triangle(self, double xA, double yA, double xB, double yB,
                                      double xC, double yC, double restitution):
        self.c_space.add_triangle(xA, yA, xB, yB, xC, yC, restitution)
        triangle = Triangle(xA, yA, xB, yB, xC, yC, restitution)
        self.triangles.append(triangle)
        return triangle

    cpdef AngleSensor add_angle_sensor(self, Node origin, Node satellite, AngleSensor ref_sensor=None):
        if ref_sensor is None:
            c_sensor = self.c_space.add_angle_sensor(origin.c_node, satellite.c_node)
        else:
            c_sensor = self.c_space.add_relative_angle_sensor(origin.c_node, satellite.c_node, ref_sensor.c_sensor)
        sensor = AngleSensor(origin, satellite, ref_sensor)
        sensor.c_sensor = c_sensor
        self.sensors.append(sensor)
        return sensor

    cpdef TouchSensor add_touch_sensor(self, list nodes):
        cdef vector[CppNode*] cpp_nodes # = new vector[CppNode*]()
        for node in nodes:
            cpp_nodes.push_back((<Node> node).c_node)
        c_sensor = self.c_space.add_touch_sensor(cpp_nodes)
        sensor = TouchSensor(nodes)
        sensor.c_sensor = c_sensor
        self.sensors.append(sensor)
        return sensor

    cpdef AngularVelocitySensor add_angular_velocity_sensor(self, AngleSensor angle_sensor):
        c_sensor = self.c_space.add_angular_velocity_sensor(angle_sensor.c_sensor)
        sensor = AngularVelocitySensor(angle_sensor)
        sensor.c_sensor = c_sensor
        self.sensors.append(sensor)
        return sensor

    cpdef list[double] sensor_values(self):
        return [sensor.value for sensor in self.sensors]

    cpdef void step(self) except +:
        for update_function in self.update_functions:
            update_function(self)

        self.c_space.step()

        for sensor in self.sensors:
            sensor.update()

        for entity in self.entities:
            entity.update(self.t)

    cpdef add_entity(self, entity):
        self.entities.append(entity)

    cpdef add_update_function(self, update_function):
        """Add a function to be run each step. Take the space as argument."""
        self.update_functions.append(update_function)
