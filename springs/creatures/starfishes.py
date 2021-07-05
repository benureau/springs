from .. import utils
from .tentacles import Tentacle, DevTentacle
from .parts import Section
from . import motors
import statistics


class Starfish:

    default_materials = {
        'node_center' : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_tip'    : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True},
        'link_center'      : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

    def __init__(self, space, arm_dimensions, center_pos, center_radius,
                       materials=None, tentacle_cls=Tentacle, section_cls=Section,
                       muscle_n_groups=None, muscle_cls=motors.SectionOneMuscle,
                       sensor_cfg=None,
                       **kwargs):
        self.space = space
        self.center_radius = center_radius
        self.sensors = []
        self.materials = materials if materials is not None else self.default_materials

        # self.core_accelerometer = core_accelerometer
        # self.core_position      = core_position
        # self.tips_accelerometer = tips_accelerometer
        # self.tips_position      = tips_position

        self.muscle_n_groups = muscle_n_groups

        self.links, self.springs, self.muscles = [], [], []

        self.tentacles = []
        bases = self._create_center(center_pos, len(arm_dimensions), section_cls.base_size)
        for arm_dim, base in zip(arm_dimensions, bases):
            heights, widths = zip(*arm_dim)
            if widths[-1] == 0:  # FIXME: 0?
                widths = widths[:-1]
            tentacle = tentacle_cls(self.space, base, heights, widths, materials=self.materials,
                                    section_cls=section_cls, **kwargs)
            self.tentacles.append(tentacle)

        # FIXME: add center
        self.nodes = self.center[:]
        self.links, self.springs, self.muscles = [], [], []
        for tentacle in self.tentacles:
            self.nodes.extend(tentacle.new_nodes)
            self.links.extend(tentacle.links)
            self.springs.extend(tentacle.springs)
            self.muscles.extend(tentacle.muscles)
        self.new_nodes = self.nodes

        if muscle_n_groups is not None:
            self.create_muscle_interface(n_group=muscle_n_groups, muscle_cls=muscle_cls)

        self.sensors = StarfishSensors(space, self, sensor_cfg)
        self._controllers = []
        #self._create_sensors()


    def add_controller(self, fun, *args, **kwargs):
        """Add a controller function to the entity.

        The controller function should accept two arguments:
        - the entity (here `self`)
        - the physics engine time (here `self.space.t`)
        - any other argument provided in `*args` and `*kwargs`

        When there are more than one controller functions, they are
        executed in the order they were added.
        """
        self._controllers.append((fun, args, kwargs))

    def _make_link(self, node_a, node_b, material_name):
        link = self.space.add_link(node_a, node_b, **self.materials[material_name])
        self.links.append(link)

    def _create_center(self, center_pos, n_base, base_size=2):
        """
        :param n_base:     number of bases
        :param base_size:  number of nodes per base
        """
        c_x, c_y = center_pos
        assert base_size > 1
        n = n_base * (base_size - 1)
        if n_base == 1:
            x, y = c_x + self.center_radius, c_y
            self.center = [self.space.add_node(x, y, **self.materials['node_center'])]
            return [n_node*(self.center[0],)]

        nodes = utils.polygon(n, c_x, c_y, self.center_radius)
        self.center = [self.space.add_node(x, y, **self.materials['node_center']) for x, y in nodes]

        if n_base == 2:
            for i in range(base_size-1):
                self._make_link(self.center[i], self.center[i+1], 'link_center')
                self._make_link(self.center[i+1], self.center[i], 'link_center')
            bases = []
            bases.append(tuple(self.center[i] for i in range(base_size)))
            bases.append(reversed(tuple(self.center[i] for i in range(base_size))))
            return bases
        else:
            for i, node in enumerate(self.center):
                j1 = (i+1) % len(self.center)
                j2 = (i+2) % len(self.center)
                j3 = (i+len(self.center)//3) % len(self.center)

                self._make_link(self.center[j1], node, 'link_center')
                self._make_link(self.center[j2], node, 'link_center')
                self._make_link(self.center[j3], node, 'link_center')

            return [tuple(self.center[(base_size-1)*j-i]
                    for i in range(base_size)) for j in range(n_base)]

    def create_muscle_interface(self, n_group=2, muscle_cls=motors.SectionOneMuscle):
        """Create the muscle interface for the starfish. Look at the Tentacle class for details.

        :param n_group:     number of muscle groups. Each member of a group receive the same
                            activation.
        :param muscle_cls:  class of muscle interface for an individual section of a tentacle.
        """
        groups = []
        for tentacle in self.tentacles:
            tentacle.muscles_proximodistal(n_group, muscle_cls=muscle_cls)
            groups.append(tentacle.muscle_interface)
        self.muscle_interface = motors.MuscleInterface(groups)
        return self.muscle_interface


    def update(self, t):
        # self.sensors.update()
        for controller_fun, args, kwargs in self._controllers:
            controller_fun(self, self.space, *args, **kwargs)

    def eval_state(self):
        return self.avg_center_xy

    @property
    def center_xy(self):
        """Average position of the center nodes"""
        return self.avg_center_xy

    @property
    def avg_center_xy(self):
        """Average position of the center nodes"""
        xs, ys = [node.x for node in self.center], [node.y for node in self.center]
        return statistics.mean(xs), statistics.mean(ys)

    @property
    def avg_nodes_xy(self):
        """Average position of all node of the starfish"""
        return statistics.mean([n.x for n in self.nodes]), statistics.mean([n.y for n in self.nodes])

    def translate(self, dx, dy):
        """Virtual (instantaneous, no physics) translation of the starfish"""
        for node in self.nodes:
            node.translate(dx, dy)



sensor_cfg_template_str = """
sensors:
    touch: 'all_nodes'  # or 'muscle_group_sides', or null
    center_angle: true  # if True, create angle sensors. if 'angle' is not None,
                        # override to True.
    angle: 'all_nodes'  # or 'muscle_group', or null
    angular_velocity: true   # will add one sensor for each angle sensor.
"""
sensor_cfg_template = {'sensors': {'touch': 'muscle_group_sides', 'angle': 'muscle_group',
                                   'angular_velocity': True, 'center_angle': True}}

class StarfishSensors:

    def __init__(self, space, starfish, sensor_cfg):
        self.space = space
        self.starfish = starfish
        self.cfg = sensor_cfg
        if self.cfg is not None:
            self._create_touch_sensors()
            self._create_angle_sensors()

    def _create_touch_sensors(self):
        if self.cfg['touch'] == 'muscle_group_sides':
            for tentacle in self.starfish.tentacles:
                left_nodes, right_nodes = [], []
                for group in tentacle.section_groups:
                    for section in group:
                        left_nodes .append(section.node_map['top_left'])
                        right_nodes.append(section.node_map['top_right'])
                self.space.add_touch_sensor(left_nodes)
                self.space.add_touch_sensor(right_nodes)
        elif self.cfg['touch'] == 'all_nodes':
            for node in starfish.nodes:
                self.space.add_touch_sensor([node])
        elif self.cfg['touch'] is None:
            pass
        else:
            raise ValueError("Unrecognized value for touch sensors '{}'.".format(self.cfg['touch']))

    def _create_angle_sensors(self):
        if self.cfg.get('angle', None) is not None:
            self.cfg['center_angle'] = True

        # create center angle
        if self.cfg['center_angle']:
            center = self.starfish.center
            node_1, node_2 = center[0], center[len(center)//2]
            center_angle_sensor = self.space.add_angle_sensor(node_1, node_2)
            if self.cfg.get('angular_velocity', False):
                self.space.add_angular_velocity_sensor(center_angle_sensor)

            # create peripheral sensors
            if self.cfg.get('angle', None) == 'muscle_group':
                for tentacle in self.starfish.tentacles:
                    previous_sensor = center_angle_sensor
                    for group in tentacle.section_groups:
                        last_section = group[-1]
                        node_1, node_2 = last_section.node_map['base_left'], last_section.node_map['top_left']
                        previous_sensor = self.space.add_angle_sensor(node_1, node_2, previous_sensor)
                        if self.cfg.get('angular_velocity', False):
                            self.space.add_angular_velocity_sensor(previous_sensor)
            elif self.cfg.get('angle', None) == 'all_nodes':
                raise NotImplementedError
            elif self.cfg.get('angle', None) is None:
                pass
            else:
                raise ValueError("Unrecognized value for angle sensors '{}'.".format(self.cfg.get('angle', None)))



class DevStarfish(Starfish):

    def __init__(self, *args, tentacle_cls=DevTentacle, **kwargs):
        super().__init__(*args, tentacle_cls=tentacle_cls, **kwargs)

    def change_height_dev_factor(self, factor):
        for tentacle in self.tentacles:
            tentacle.height_dev_factor = factor

    def change_width_dev_factor(self, factor):
        for tentacle in self.tentacles:
            tentacle.width_dev_factor = factor

    def change_mass(self, new_mass):
        for node in self.nodes:
            node.mass = new_mass
