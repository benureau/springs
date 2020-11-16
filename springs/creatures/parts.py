import math
import copy

from .. import utils


class Tip:
    default_materials = {
        'node_tip': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_tip': {'stiffness':  500000.0, 'damping_ratio': 1.0, 'actuated': False}}


    def __init__(self, space, base, height, materials=None):
        self.space, self.base, self._height = space, base, height
        self.materials = materials if materials is not None else self.default_materials

        self.nodes, self.links = list(self.base), []
        self.node_map, self.link_map = {'tip': None}, {}

        self._build()
        self.muscles = [link for link in self.links if link.actuated]
        self.springs = [link for link in self.links if not link.actuated]


    def _build(self):
        """Create the node and links of the section"""
        x, y = utils.pos_rel(0, self.height, self.base[0], self.base[-1])
        node = self.space.add_node(x, y, **self.materials['node_tip'])
        self.nodes.append(node)
        self.new_nodes = [node]
        self.node_map['tip'] = node

        for i, base_node in enumerate(self.base):
            link = self.space.add_link(base_node, node, **self.materials['link_tip'])
            self.links.append(link)
            self.link_map['tiplink_{}'.format(i)] = link

    # Height and width
    @property
    def height(self): return self._height

    @height.setter # FIXME
    def height(self, value):
        """Change the height of the section. Adjust the diagonal relax length accordingly."""
        assert value >= 0
        base_d = utils.distance(self.base[0], self.base[1])  # relax lengths
        for link in self.links:
            half_base = base_d / 2
            link.relax_length = math.sqrt(half_base * half_base + value * value)
        self._height = value


class Section:
    """A section of a tentacle.

    :param space:             the section will be added to this space.
    :param base:              a pair of Nodes, to serve as base of the section. Given a base (A, B),
                              with A.y == B.y == 0 and A.x < B.x, then the square will be created
                              on the positive side of the x-axis.
    :param height:            height of the section.
    :param width:             width of the section. Can be 0, in which case, the section is a
                              triangle, and only one new node is created.
    :param muscle_frequency:  the frequency of the muscles links.
    :param muscle_damping:    the damping of the muscles links.
    :param spring_frequency:  the frequency of the springs links.
    :param spring_damping:    the damping of the springs links.
    :param spring_frequency:  the frequency of the bones links.
    :param spring_damping:    the damping of the bones links.
    :param side_muscles:      if True, muscles links will be created on the side of the section.
                              Else, they are replaced by passive springs.
    :param internal_muscle:   if True, a muscle link is created between the two end nodes of the
                              section.
    """
    base_size = 2
    default_materials = {
        'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True}}

    def __init__(self, space, base, height, width, materials=None):
        self.space, self._base     = space, base
        self._height, self._width = height, width
        self.materials = materials if materials is not None else self.default_materials

        self.nodes, self.links = [base[0], base[1]], []
        self.new_nodes = []
        self.muscles, self.springs = [], []

        self.node_map = {'base_left': base[0], 'base_right': base[-1],
                          'top_left': None,     'top_right': None}
        self.link_map = {} #'left': None, 'right': None, 'width': None,
                           #'diag_LR': None, 'diag_RL': None}
        self.link_by_mat = {name: [] for name in self.materials.keys() if name.startswith('link_')}

        self._build()


    ## Read-only properties
    @property
    def base(self): return self._base

    @property
    def forward_base(self): return (self.node_map['top_left'], self.node_map['top_right'])

    # Height and width
    @property
    def height(self): return self._height

    @height.setter
    def height(self, value):
        """Change the section's height. Adjust the side and diagonal relax lengths accordingly"""
        assert value >= 0
        for link in [self.link_map['left'], self.link_map['right']]:
            link.relax_length = value
        self._height = value
        diag_length = self._diag_length()
        self.link_map['diag_LR'].relax_length = diag_length
        self.link_map['diag_RL'].relax_length = diag_length

    @property
    def width(self): return self._width

    @width.setter
    def width(self, value):
        """Change the width of the section. Adjust the side and diagonal relax lengths accordingly"""
        assert value >= 0
        self.link_map['width'].relax_length = value
        self._width = value
        diag_length = self._diag_length()
        self.link_map['diag_LR'].relax_length = diag_length
        self.link_map['diag_RL'].relax_length = diag_length

    def _diag_length(self):
        """Natural diagonal length"""
        base_d = utils.distance(*self.base)
        avg_width = (base_d + self.width) / 2
        half_diff_width = base_d - avg_width
        return math.sqrt(avg_width * avg_width + self.height * self.height - half_diff_width * half_diff_width)


    # Material changes
    def stiffness(self, material_name):
        """Return a material stiffness"""
        return self.materials[material_name][0]

    def damping(self, material_name):
        """Return a material damping"""
        return self.materials[material_name][1]

    def actuated(self, material_name):
        """Return if a material is actuated"""
        return self.materials[material_name][2]

    def change_stiffness(self, material_name, value):
        assert value >= 0
        for link in self.link_by_mat[material_name]:
            link.stiffness = value

    def change_damping(self, material_name, value):
        assert value >= 0
        for link in self.link_by_mat[material_name]:
            link.damping = value


    # Other
    def relax(self):
        for link in self.muscles:
            link.relax()

    def _make_link(self, node_a, node_b, material_name, name=None):
        materials = copy.deepcopy(self.materials[material_name])
        link_type = materials.pop('type', 'link')
        add_link_map = {'link': self.space.add_link}
        if hasattr(self.space, 'add_spring'):
            add_link_map['spring'] = self.space.add_spring

        link = add_link_map[link_type](node_a, node_b, **materials)

        self.links.append(link)
        self.link_by_mat[material_name].append(link)
        if name is not None:
            assert name not in self.link_map or self.link_map[name] is None
            self.link_map[name] = link

        if link.actuated:
            self.muscles.append(link)
        else:
            self.springs.append(link)
        return link

    # Creating the section
    def _build(self):
        """Create the node and links of the section"""
        base_d = utils.distance(self.base[0], self.base[-1])
        assert base_d > 0
        y = math.sqrt(self.height ** 2 - ((base_d - self.width) / 2) ** 2)
        #if y <= 0:
        #    print('warning: unsafe starting values for section')  # FIXME: log.warn, more precise msg.
        y = max(y, 0.01)  # avoid zero lock, or built the wrong way.

        x_left,  y_left  = utils.pos_rel(-self.width/2, y, self.base[0], self.base[-1])
        x_right, y_right = utils.pos_rel( self.width/2, y, self.base[0], self.base[-1])
        n_left  = self.space.add_node(x_left,  y_left,  **self.materials['node_section'])
        n_right = self.space.add_node(x_right, y_right, **self.materials['node_section'])
        self.nodes.extend([n_left, n_right])
        self.new_nodes = [n_left, n_right]

        self.node_map['top_left']  = n_left
        self.node_map['top_right'] = n_right

        # diagonal "X" links
        self._make_link(self.base[ 0], n_right, 'link_sec_diag',  name='diag_LR')
        self._make_link(self.base[-1],  n_left, 'link_sec_diag',  name='diag_RL')
        # height (side) links
        self._make_link(self.base[ 0],  n_left, 'link_sec_side',  name='left')
        self._make_link(self.base[-1], n_right, 'link_sec_side',  name='right')
        # width (internal) link
        self._make_link(n_left,       n_right, 'link_sec_width', name='width')


class CentralBoneSection(Section):

    base_size = 3
    default_materials = {
        'node_section': {'mass': 1.0, 'friction': 0.5, 'fixed': False},
        'link_sec_diag'    : {'stiffness':  100000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_big_diag': {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_center'  : {'stiffness':   50000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_width'   : {'stiffness':   12000.0, 'damping_ratio': 1.0, 'actuated': False},
        'link_sec_side'    : {'stiffness':    3000.0, 'damping_ratio': 1.0, 'actuated':  True}}

    @property
    def forward_base(self):
        return (self.node_map['top_left'],
                self.node_map['top_middle'],
                self.node_map['top_right'])

    # Creating the section
    def _build(self):
        """Create the node and links of the section"""
        b_left, b_middle, b_right = self.base
        base_d = utils.distance(b_left, b_right)

        # distance to upper
        y = math.sqrt(self.height ** 2 - ((base_d - self.width) / 2) ** 2)
        y = max(y, 0.1)  # avoid zero lock, or built the wrong way.

        x_left  ,  y_left  = utils.pos_rel(-self.width/2, y, self.base[0], self.base[-1])
        x_right , y_right  = utils.pos_rel( self.width/2, y, self.base[0], self.base[-1])
        x_middle, y_middle = utils.pos_rel(  0, self.height, self.base[0], self.base[-1])

        n_left   = self.space.add_node(x_left,   y_left,   **self.materials['node_section'])
        n_right  = self.space.add_node(x_right,  y_right,  **self.materials['node_section'])
        n_middle = self.space.add_node(x_middle, y_middle, **self.materials['node_section'])
        self.nodes.extend([n_left, n_middle, n_right])
        self.new_nodes = [n_left, n_middle, n_right]

        self.node_map['top_left']   = n_left
        self.node_map['top_right']  = n_right
        self.node_map['top_middle'] = n_middle

        # diagonal "X" links
        self._make_link(self.base[0], n_middle, 'link_sec_diag',     name='diag_LM')
        self._make_link(self.base[1], n_left,   'link_sec_diag',     name='diag_ML')
        self._make_link(self.base[1], n_right,  'link_sec_diag',     name='diag_MR')
        self._make_link(self.base[2], n_middle, 'link_sec_diag',     name='diag_RM')

        #
        self._make_link(self.base[0], n_right,  'link_sec_big_diag', name='diag_LR')
        self._make_link(self.base[2], n_left,   'link_sec_big_diag', name='diag_RL')

        # height (side) links
        self._make_link(self.base[0], n_left,   'link_sec_side',     name='left')
        self._make_link(self.base[1], n_middle, 'link_sec_center',   name='middle')
        self._make_link(self.base[2], n_right,  'link_sec_side',     name='right')

        # width (internal) link
        self._make_link(n_left,       n_middle, 'link_sec_width',    name='widthL')
        self._make_link(n_middle,     n_right,  'link_sec_width',    name='widthR')

    # Height and width  # necessary to redefine?
    @property
    def width(self): return self._width

    @width.setter
    def width(self, value):
        """Change the width of the section. Adjust the side and diagonal relax lengths accordingly"""
        assert value >= 0
        self.link_map['widthL'].relax_length = value/2
        self.link_map['widthR'].relax_length = value/2
        self._width = value
        small_diag1, small_diag2, big_diag = self._diag_lengths()
        self.link_map['diag_LM'].relax_length = small_diag1
        self.link_map['diag_ML'].relax_length = small_diag2
        self.link_map['diag_RM'].relax_length = small_diag1
        self.link_map['diag_MR'].relax_length = small_diag2
        self.link_map['diag_LR'].relax_length = big_diag
        self.link_map['diag_RL'].relax_length = big_diag

    # Height and width
    @property
    def height(self): return self._height

    @height.setter
    def height(self, value):
        """Change the width of the section. Adjust the side and diagonal relax lengths accordingly"""
        assert value >= 0
        self.link_map[  'left'].relax_length = value
        self.link_map['middle'].relax_length = value
        self.link_map[ 'right'].relax_length = value
        self._height = value
        small_diag1, small_diag2, big_diag = self._diag_lengths()
        self.link_map['diag_LM'].relax_length = small_diag1
        self.link_map['diag_ML'].relax_length = small_diag2
        self.link_map['diag_RM'].relax_length = small_diag1
        self.link_map['diag_MR'].relax_length = small_diag2
        self.link_map['diag_LR'].relax_length = big_diag
        self.link_map['diag_RL'].relax_length = big_diag

    def _diag_lengths(self):
        """Natural diagonal length"""
        base_d = utils.distance(self.base[0], self.base[-1]) # FINXME: not the relax length!
        avg_width = (base_d + self.width) / 2
        half_diff_width = base_d - avg_width
        big_diag = math.sqrt(avg_width * avg_width + self.height * self.height - half_diff_width * half_diff_width)
        small_diag1 = math.sqrt(base_d * base_d / 4 + self.height * self.height)
        small_diag2 = math.sqrt(self.width * self.width / 4 + self.height * self.height)
        return small_diag1, small_diag2, big_diag
