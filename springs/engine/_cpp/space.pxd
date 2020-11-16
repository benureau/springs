from libcpp cimport bool
from libcpp.vector cimport vector


    # Nodes

cdef extern from "node.h" namespace "springs":

    cdef cppclass Node:
        Node(double, double, double, double, double, bool) except +
        double dt, x, y, v_x, v_y, friction

        double mass()
        void set_mass(double)
        double inv_mass()

        bool fixed()
        void set_fixed(bool)

        void update_position()
        void translate(double, double)

cdef extern from "node.cpp":
    pass


    # Links

cdef extern from "link.h" namespace "springs":
    cdef cppclass Link:
        Link(double, Node*, Node*, double, double, bool) except +

        bool   actuated
        double relax_length
        double expand_factor
        double max_impulse
        double max_length

        double length()

        double dt()
        void set_dt(double)

        double stiffness()
        void set_stiffness(double)

        double damping_ratio()
        void set_damping_ratio(double)

        double frequency()
        void set_frequency(double)

        void contract(double)
        void relax()

        double force()

cdef extern from "link.cpp":
    pass


    # Springs

cdef extern from "spring.h" namespace "springs":
    cdef cppclass Spring:
        Spring(double, Node*, Node*, double, double, bool) except +

        bool   actuated
        double relax_length
        double expand_factor
        double max_impulse
        double max_length

        double length()

        double dt()
        void set_dt(double)

        double stiffness()
        void set_stiffness(double)

        double damping_ratio()
        void set_damping_ratio(double)

        double frequency()
        void set_frequency(double)

        void contract(double)
        void relax()

cdef extern from "spring.cpp":
    pass


    # Rects

cdef extern from "rect.h" namespace "springs":
    cdef cppclass Rect:
        double xL, xR, yB, yT, width, height, restitution

cdef extern from "rect.cpp":
    pass


    # Triangles

cdef extern from "trig.h" namespace "springs":
    cdef cppclass Triangle:
        double xA, yA, xB, yB, xC, yC, restitution

cdef extern from "trig.cpp":
    pass


    # Sensors

cdef extern from "sensors.h" namespace "springs":
    cdef cppclass Sensor:
        Sensor() except +
        double value() except +
        double update() except +

    cdef cppclass AngleSensor(Sensor):
        AngleSensor(Node*, Node*) except +
        # double update() except +

    cdef cppclass TouchSensor(Sensor):
        TouchSensor(vector[Node*]) except +

    cdef cppclass AngularVelocitySensor(Sensor):
        AngularVelocitySensor(AngularVelocitySensor*, double) except +

    # cdef cppclass SensorHub:
    #     void add_sensor(Sensor) except +
    #     vector[double] sensor_values() except +

cdef extern from "sensors.cpp":
    pass


    # Space

cdef extern from "space.h" namespace "springs":
    cdef cppclass Space:
        Space(double, int, double, double, double) except +
        # ~Space()

        double gravity_x, gravity_y, t, restitution_threshold
        int n_substep, ticks

        double dt()
        void set_dt(double)

        Node* add_node(double, double, double, double, double)
        Link* add_link(Node*, Node*, double, double, bool, double)
        Spring* add_spring(Node*, Node*, double, double, bool, double)
        Rect* add_rect(double, double, double, double, double)
        Triangle* add_triangle(double, double, double, double, double, double, double)

        # SensorHub* sensors
        AngleSensor* add_angle_sensor(Node*, Node*)
        AngleSensor* add_relative_angle_sensor(Node*, Node*, AngleSensor*)
        TouchSensor* add_touch_sensor(vector[Node*])
        AngularVelocitySensor* add_angular_velocity_sensor(AngleSensor*)

        # void add_sensor(Sensor*)
        # vector[double] sensor_values()

        void step() except +

cdef extern from "space.cpp":
    pass
