import copy

from .. import utils
from . import motors
from .parts import Section, Tip


def _even_divide(values, k):
    """Divide a list of values into `k` sections, as evenly as possible

    > _even_divide([1, 2, 3, 4, 5], 3) == [[1, 2], [3, 4], [5]]
    """
    n = len(values)
    base = (n // k)
    rest = n - base * k

    results, offset = [], 0
    for _ in range(rest):
        results.append(values[offset:offset+base+1])
        offset += base + 1
    if base > 0:
        for _ in range(k - rest):
            results.append(values[offset:offset+base])
            offset += base
    return results



class Tentacle:
    """

    """

    default_materials = {
        'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'node_tip'    : {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True},
        'link_tip'         : {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}

    def __init__(self, space, base, heights, widths, section_cls=Section, materials=None, **kwargs):
        self.space, self._base = space, base
        self.materials = materials if materials is not None else self.default_materials

        self.size = len(widths)
        assert len(heights) in (self.size, self.size+1)

        self.sections = []
        for i in range(self.size):
            s = section_cls(space, base, heights[i], widths[i], materials=self.materials)
            self.sections.append(s)
            base = s.forward_base

        self.tip = None
        if len(heights) == len(widths) + 1:
            tip_base = self.sections[-1].forward_base
            self.tip = Tip(space, tip_base, heights[-1], materials=self.materials)

        self.nodes, self.links, self.springs, self.muscles = list(self.base), [], [], []
        for section in self.sections:
            self._populate(section)
        if self.tip is not None:
            self._populate(self.tip)
        self.new_nodes = self.nodes[2:]  # excluding the base

        self.left_muscles, self.right_muscles, self.width_muscles = [], [], []
        for section in self.sections:
            lmap = section.link_map
            if lmap['left'] .actuated: self.left_muscles .append(lmap['left'])
            if lmap['right'].actuated: self.right_muscles.append(lmap['right'])
            #if lmap['width'].actuated: self.width_muscles.append(lmap['width']) # FIXME

    def _populate(self, part):
        self.nodes.extend(part.new_nodes)
        self.links.extend(part.links)
        self.springs.extend(part.springs)
        self.muscles.extend(part.muscles)

    def update(self, t):
        pass

    @property
    def base(self):
        return self._base

    def height_material(self):
        return [section.height_material for section in self.sections]

    def width_material(self):
        return [section.width_material for section in self.sections]

    def diag_material(self):
        return [section.diag_material for section in self.sections]

    def widths(self):
        return [section.width for section in self.sections]

    def heights(self):
        return [section.width for section in self.sections]

    def change_frequency(self, material_name, value):
        value = utils.listify(value, self.size)
        for section, v_i in zip(self.sections, value):
            section.change_frequency(material_name, v_i)

    def change_damping(self, material_name, value):
        value = utils.listify(value, self.size)
        for section, v_i in zip(self.sections, value):
            section.change_damping(material_name, v_i)

    def muscles_proximodistal(self, n_group=2, muscle_cls=motors.SectionOneMuscle):
        """Create groups of muscles to be actuated together along the tentacle.

        The group are formed of the same type of muscles (e.g. witdh muscles, or left-height
        muscles), grouped by proximity along the proximo-distal axis. Thus, a tentacle with four
        sections, each with height muscles and width springs, and `n_group == 2` will generate
        four muscles groups: one for the two first sections' left-height muscles, one for the two
        last sections' left-height muscles, and two more for the right-heigh muscles, similarly.

        :param n_group:     number of muscle groups. Each member of a group receive the same
                            activation.
        :param muscle_cls:  class of muscle interface for an individual section.
        """
        groups = []
        self.section_groups = _even_divide(self.sections, n_group)
        for group in self.section_groups:
            mb = motors.MuscleBroadcast([muscle_cls(section) for section in group])
            groups.append(mb)
        self.muscle_interface = motors.MuscleInterface(groups)



class DevTentacle(Tentacle):

    def __init__(self, space, base, heights, widths, *args,
                       height_dev_factor=1.0, width_dev_factor=1.0, **kwargs):

        self._height_dev_factor = utils.listify(height_dev_factor, len(heights))
        self._width_dev_factor  = utils.listify(width_dev_factor,  len(widths))

        # heights and width at development factor 1.0
        self.heights_one = tuple(heights)
        self.widths_one  = tuple(widths)
        starting_heights = [height_dev_factor * h for h in heights]
        starting_widths  = [width_dev_factor * w for w in widths]

        super().__init__(space, base, starting_heights, starting_widths, *args, **kwargs)

    @property
    def height_dev_factor(self): return self._height_dev_factor

    @property
    def width_dev_factor(self): return self._width_dev_factor

    @height_dev_factor.setter
    def height_dev_factor(self, value):
        value = utils.listify(value, len(self.heights_one))
        height_sections = self.sections
        if self.tip is not None:
            height_sections = self.sections + [self.tip]
        for i, section in enumerate(height_sections):
            section.height = value[i] * self.heights_one[i]
        self._height_dev_factor = value

    @width_dev_factor.setter
    def width_dev_factor(self, value):
        value = utils.listify(value, self.size)  # dangerous with/without tips
        for i, v_i in enumerate(value):
            self.sections[i].width = v_i * self.widths_one[i]
        self._width_dev_factor = value
