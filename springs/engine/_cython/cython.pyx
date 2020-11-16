#cython: language_level=3, boundscheck=False
#distutils: language = c++

import copy

from libcpp cimport bool
from libc.math cimport floor, sqrt, hypot


cdef double max_translation = 2.0;
cdef double max_translation_squared = max_translation * max_translation;

cdef double clamp(double min_v, double value, double max_v):
    return max(min_v, min(max_v, value))

# cpdef double distance(Node a, Node b):
#     return hypot(a.x - b.x, a.y - b.y)


cdef class Node:

    cdef public   double dt
    cdef readonly double x, y            # position
    cdef readonly double x_prev, y_prev  # position at the previous timestep
    cdef public   double v_x, v_y        # velocity
    cdef readonly double friction
    cdef public object links

    cdef double          _mass
    cdef public double   _inv_mass
    cdef readonly bool   _fixed
    cdef readonly double _dt_sq

    def __cinit__(self, double dt, double x, double y, double mass, double friction, bool fixed):
        self.links = []   # links the node is connected to.
        self._fixed = fixed  # can't set self.fixed yet
        self.x, self.y, self.v_x, self.v_y = x, y, 0.0, 0.0
        self.dt, self.mass, self.friction = dt, mass, friction
        self._dt_sq = self.dt * self.dt

    @property
    def position(self):
        return self.x, self.y

    @property
    def velocity(self):
        return self.v_x, self.v_y

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, value):
        assert value >= 0.0
        self._mass = 0.0 if self._fixed else value
        self._inv_mass = 0.0 if self._fixed else 1 / self._mass
        for link in self.links:
            link._update()

    @property
    def fixed(self):
        return self._fixed

    @fixed.setter
    def fixed(self, bool value):
        if self._fixed != value:
            self._inv_mass = 0.0 if self._fixed else 1 / self._mass
        self._fixed = value
        for link in self.links:
            link._update()

    cpdef void update_position(self):
        self.x_prev, self.y_prev = self.x, self.y
        if self.fixed:
            self.v_x = 0
            self.v_y = 0
        else:
            # bounding large velocities to avoid instabilities
            translation_squared = self._dt_sq * (self.v_x * self.v_x + self.v_y * self.v_y)
            if translation_squared > max_translation_squared:
                translation = sqrt(translation_squared)
                self.v_x *= max_translation / translation
                self.v_y *= max_translation / translation

            self.x = self.x + self.dt * self.v_x
            self.y = self.y + self.dt * self.v_y

    cpdef void translate(self, float dx, float dy):
        self.x += dx
        self.y += dy


cdef class Link:
    """Code based on the soft constraints of Erin Catto, and of his Box2D implementation."""

    cdef readonly double _dt
    cdef readonly Node node_a, node_b

    cdef readonly double mass, inv_mass
    cdef readonly double u_x, u_y

    cdef readonly double impulse, max_impulse, bias, gamma
    cdef readonly double _frequency, _stiffness, _damping_ratio

    cdef readonly double expand_factor
    cdef public double relax_length
    cdef readonly double max_length
    cdef public bool actuated, active


    def __cinit__(self, double dt, Node node_a, Node node_b, double stiffness,
                        double damping_ratio, bool actuated, double max_impulse):
        self._dt = dt
        self.node_a, self.node_b = node_a, node_b
        self._stiffness, self._damping_ratio = stiffness, damping_ratio
        self.actuated = actuated
        self.expand_factor, self.relax_length = 1, self._distance_unit_vector()
        self.max_impulse = max_impulse
        self.max_length = self.relax_length

        self._update()


    @property
    def dt(self):
        return self._dt

    @dt.setter
    def dt(self, value):
        assert value > 0.0
        self._dt = value
        self._update()

    @property
    def stiffness(self):
        return self._stiffness

    @stiffness.setter
    def stiffness(self, value):
        assert value > 0.0
        self._stiffness = value
        self._update()

    @property
    def frequency(self):
        return self._frequency

    @frequency.setter
    def frequency(self, value):
        assert value > 0.0
        _mass = 1 / (self.node_a._inv_mass + self.node_b._inv_mass);
        self._frequency = sqrt(self._stiffness / _mass) / 6.28318530718
        self._update()

    @property
    def damping_ratio(self):
        return self._damping_ratio

    @damping_ratio.setter
    def damping_ratio(self, value):
        self._damping_ratio = value
        self._update()

    @property
    def target_length(self):
        return self.expand_factor * self.relax_length

    cpdef void contract(self, expand_factor):
        assert self.actuated
        self.expand_factor = expand_factor

    cpdef void relax(self):
        assert self.actuated
        self.expand_factor = 1

    cdef void _update(self):
        """Precompute invariants.

        Must be run each time the mass of the objects, the parameters of the
        link or the dt is changed (automatically done through properties above).
        """
        self.active = not (self.node_a.fixed and self.node_b.fixed)
        if self.active:
            self.inv_mass = self.node_a._inv_mass + self.node_b._inv_mass
            self.mass = 0.0 if self.inv_mass == 0 else 1 / self.inv_mass

            omega = sqrt(self.stiffness * self.inv_mass)
            self._frequency = omega / 6.28318530718  # omega / 2π
            damping = 2 * self.mass * self._damping_ratio * omega
            self.gamma = 1.0 / (self._dt * (damping + self._dt * self._stiffness))

            self.inv_mass += self.gamma
            self.mass = 1 / self.inv_mass
            self.impulse = 0.0


    cdef void _update_velocities(self, double impulse):
        """Apply an impulse: modify the connected node velocities."""
        if impulse != 0:
          P_x, P_y = impulse * self.u_x, impulse * self.u_y
          if not self.node_a._fixed:
              self.node_a.v_x -= P_x * self.node_a._inv_mass
              self.node_a.v_y -= P_y * self.node_a._inv_mass

          if not self.node_b._fixed:
              self.node_b.v_x += P_x * self.node_b._inv_mass
              self.node_b.v_y += P_y * self.node_b._inv_mass


    cdef double _distance_unit_vector(self):
        """Return the distance of the constraints, and update the unit vector."""
        d_x, d_y = (self.node_b.x - self.node_a.x), (self.node_b.y - self.node_a.y)
        d = sqrt(d_x*d_x + d_y*d_y)
        if d > 0:
            self.u_x, self.u_y = d_x / d, d_y / d
        return d

    cpdef double prestep(self):
        """Prepare the link for a new engine timestep."""
        if self.active:
            d = self._distance_unit_vector()
            # self.max_length = max(d, self.max_length)
            if d > 0:
              diff_d = d - self.expand_factor * self.relax_length
              self.bias = diff_d * self._dt * self._stiffness * self.gamma
              # self.impulse = 0
              self._update_velocities(self.impulse)
            return self.impulse
        return 0.0

    cdef double _relative_velocity(self):
        vBA_x, vBA_y = self.node_b.v_x - self.node_a.v_x, self.node_b.v_y - self.node_a.v_y
        return self.u_x * vBA_x + self.u_y * vBA_y

    cpdef void substep(self):
        """Constraint update for a link.

        Update the velocities to satisfy the current constraints. For best results, all the links
        should update their constraints multiple time during a timestep udpate.
        """
        if self.active:
            v_rn = self._relative_velocity()
            impulse = - self.mass * (v_rn + self.bias + self.gamma * self.impulse)
            self.impulse += impulse

            self._update_velocities(impulse)


cdef class Rect:

    cdef readonly double xL, xR, yB, yT, width, height
    cdef public restitution

    def __cinit__(self, double xL, double xR, double yB, double yT, double restitution=0.5):
        self.xL, self.xR, self.yB, self.yT  = xL, xR, yB, yT
        self.restitution = restitution
        self.width, self.height = xR - xL, yT - yB

    cpdef bool collides(self, Node node):
        return self.xL < node.x <= self.xR and self.yB < node.y <= self.yT


cdef class Collision:

    cdef readonly Rect rect
    cdef readonly Node node
    cdef readonly double threshold, diff_v_x, diff_v_y

    cdef bool _x_not_y_collision, _disabled
    cdef double _bias


    def __cinit__(self, Rect rect, Node node, double threshold):
        self.rect, self.node, self.threshold = rect, node, threshold
        self.diff_v_x, self.diff_v_y = 0, 0
        self._disabled = False

        # find collision edge (approximated, we project to the closest edge)
        diff_xL, diff_xR = self.node.x - self.rect.xL, self.rect.xR - self.node.x
        diff_yB, diff_yT = self.node.y - self.rect.yB, self.rect.yT - self.node.y
        self._x_not_y_collision = min(diff_xL, diff_xR) < min(diff_yB, diff_yT)
        rst = self.rect.restitution
        if self._x_not_y_collision:
            if diff_xL < diff_xR:
                self.node.x = self.rect.xL
                if self.node.v_x < 0:
                    self._disabled = True
                else:
                    self._bias = 0.0 if self.node.v_x < threshold  else - self.node.v_x * rst
            else:
                self.node.x = self.rect.xR
                if self.node.v_x > 0:
                    self._disabled = True
                else:
                    self._bias = 0.0 if -threshold < self.node.v_x <= 0 else - self.node.v_x * rst
        else:
            if diff_yB < diff_yT:
                self.node.y = self.rect.yB
                if self.node.v_y < 0:
                    self._disabled = True
                else:
                    self._bias = 0.0 if self.node.v_y < threshold  else - self.node.v_y * rst
            else:
                self.node.y = self.rect.yT
                if self.node.v_y > 0:
                    self._disabled = True
                else:
                    self._bias = 0.0 if -threshold < self.node.v_y else - self.node.v_y * rst


    cpdef void substep(self):

        if self._disabled:
            return

        if self._x_not_y_collision:
            # friction
            max_friction = self.node.friction * abs(self.diff_v_x)
            # FIXME hard-coded threshold.
            if abs(self.node.v_y) > 1:  # moving contact: dynamic (half) friction
                max_friction /= 2  # FIXME: precompute!
            new_diff_v_y = clamp(-max_friction, self.diff_v_y - self.node.v_y, max_friction)
            self.node.v_y += new_diff_v_y - self.diff_v_y
            self.diff_v_y = new_diff_v_y

            # restitution
            new_diff_v_x = self.diff_v_x + self._bias - self.node.v_x
            self.node.v_x += new_diff_v_x - self.diff_v_x
            self.diff_v_x = new_diff_v_x
        else:
            # friction
            max_friction = self.node.friction * abs(self.diff_v_y)
            if abs(self.node.v_x) > 1:  # moving contact: dynamic (half) friction
                max_friction /= 2
            new_diff_v_x = clamp(-max_friction, self.diff_v_x - self.node.v_x, max_friction)
            self.node.v_x += new_diff_v_x - self.diff_v_x
            self.diff_v_x = new_diff_v_x

            # restitution
            new_diff_v_y = self.diff_v_y + self._bias - self.node.v_y
            self.node.v_y += new_diff_v_y - self.diff_v_y
            self.diff_v_y = new_diff_v_y


cdef class CollisionDetector:
    """Detecting node/rectangle collisions.

    Implements a grid approach to decrease the time complexity of detection.
    This code assume that rectangles don't move or change size. While adding
    rectangles during simulation is supported, it is not implemented in a
    time-efficient manner.
    """

    cdef readonly object rects
    cdef readonly int    n_bins, n_bins_x, n_bins_y
    cdef readonly double size_x, size_y
    cdef public bool autosize_x, autosize_y

    cdef object _bins
    cdef double _min_x_bin, _min_y_bin

    def __cinit__(self, double size_x, double size_y):
        self.size_x, self.size_y = size_x, size_y
        self.autosize_x, self.autosize_y = size_x < 0, size_y < 0
        self.rects, self._bins = [], None

    cdef void add_rect(self, Rect rect):
        self.rects.append(rect)
        self._bins = None

    cdef void _autosize(self):
        """Automatically chooses size_x and size_y based on the rectangles dimensions"""
        if self.autosize_x:
            widths = [rect.xR - rect.xL for rect in self.rects]
            self.size_x = 3 * sum(widths) / len(widths) # heuristic
        if self.autosize_y:
            heights = [rect.yT - rect.yB for rect in self.rects]
            self.size_y = 3 * sum(heights) / len(heights) # heuristic

    cdef void _prepare(self):
        """Create the bins for the rectangles."""
        self._autosize()

        min_x = min(r.xL for r in self.rects)
        max_x = max(r.xR for r in self.rects)
        min_y = min(r.yB for r in self.rects)
        max_y = max(r.yT for r in self.rects)

        self._min_x_bin = self.size_x * floor(min_x/self.size_x)
        self._min_y_bin = self.size_y * floor(min_y/self.size_y)
        self.n_bins_x = int(floor(max_x/self.size_x) - floor(min_x/self.size_x) + 1)
        self.n_bins_y = int(floor(max_y/self.size_y) - floor(min_y/self.size_y) + 1)
        self.n_bins = self.n_bins_x * self.n_bins_y

        self._bins = [[[] for _ in range(self.n_bins_y)] for _ in range(self.n_bins_x)]

        for rect in self.rects:
            min_x_index, min_y_index = self._bin_x(rect.xL), self._bin_y(rect.yB)
            max_x_index, max_y_index = self._bin_x(rect.xR), self._bin_y(rect.yT)
            for i in range(min_x_index, max_x_index + 1):
                for j in range(min_y_index, max_y_index + 1):
                    self._bins[i][j].append(rect)

    cdef int _bin_x(self, double x):
        """Return the x bins a given point belongs to"""
        return int(floor((x - self._min_x_bin)/self.size_x))

    cdef int _bin_y(self, double y):
        """Return the y bins a given point belongs to"""
        return int(floor((y - self._min_y_bin)/self.size_y))

    cdef object detect_collisions(self, nodes, double restitution_threshold):
        """Return actual collisions as a list of `Collision` instances"""
        if self._bins is None:
            if len(self.rects) == 0 or len(nodes) == 0:
                return ()
            self._prepare()

        collisions = []
        for node in nodes:
            bin_x, bin_y = self._bin_x(node.x), self._bin_y(node.y)
            if 0 <= bin_y < self.n_bins_y and 0 <= bin_x < self.n_bins_x:
                for rect in self._bins[bin_x][bin_y]:
                    if rect.collides(node):
                        col = Collision(rect, node, threshold=restitution_threshold)
                        collisions.append(col)
                        # probably problematic when two or more rectangles overlap and share an edge
        return collisions



cdef class Space:

    cdef int n_substep
    cdef readonly int ticks
    cdef double _dt
    cdef public double gravity_x, gravity_y, restitution_threshold
    cdef readonly double t
    cdef readonly object nodes, links, rects, triangles, entities, update_functions
    cdef readonly CollisionDetector collision_detector

    def __cinit__(self, dt, n_substep=5, gravity=(0.0, 0.0), restitution_threshold=1.0):
        """
        :param n_substep:  number of constraint resolution steps per constraints per step() call.
        :param restitution_threshold:  below this speed, restitution of collisions will be zero.
        """
        self._dt = dt
        self.n_substep = n_substep
        self.restitution_threshold = restitution_threshold
        try:
            self.gravity_x, self.gravity_y = gravity
        except TypeError:
            self.gravity_x, self.gravity_y = 0, gravity
        self.collision_detector = CollisionDetector(-1, -1)

        self.t, self.ticks = 0, 0
        self.nodes, self.links, self.rects, self.triangles = [], [], [], []
        self.entities, self.update_functions = [], []

    @property
    def dt(self):
        return self._dt

    @dt.setter
    def dt(self, value):
        self._dt = value
        for link in self.links:
            link.dt = value
        for node in self.nodes:
            node.dt = value

    @property
    def gravity(self):
        return (self.gravity_x, self.gravity_y)


    # adding core elements

    cpdef Node add_node(self, double x, double y, double mass=1.0, double friction=0.5,
                        bool fixed=False):
          node = Node(self.dt, x, y, mass=mass, friction=friction, fixed=fixed)
          self.nodes.append(node)
          return node

    cpdef Link add_link(self, Node node_a, Node node_b, double stiffness=10000.0,
                        double damping_ratio=1.0, bool actuated=False, double max_impulse=100):
        link = Link(self._dt, node_a, node_b, stiffness, damping_ratio, actuated, max_impulse)
        self.links.append(link)
        return link

    cpdef void add_rect(self, double xL, double xR, double yB, double yT, double restitution):
        rect = Rect(xL, xR, yB, yT, restitution)
        self.rects.append(rect)
        self.collision_detector.add_rect(rect)


    # specialized functions

    def add_entity(self, entity):
        self.entities.append(entity)


    # engine loop

    cpdef void step(self):
        for update_function in self.update_functions:
            update_function(self)

        # gravity
        for node in self.nodes:
            if not node.fixed:
                node.v_x += self.gravity_x * self._dt
                node.v_y += self.gravity_y * self._dt

        for link in self.links:
            link.prestep()

        collisions = self.collision_detector.detect_collisions(self.nodes, self.restitution_threshold)

        for _ in range(self.n_substep):
            for collision in collisions:
                collision.substep()
            for link in self.links:
                link.substep()

        for p in self.nodes:
            p.update_position()

        self.t += self.dt
        self.ticks += 1

        for entity in self.entities:
            entity.update(self.t)

    def add_update_function(self, update_function):
        """Add a function to be run before each step. Take the space as argument."""
        self.update_functions.append(update_function)
