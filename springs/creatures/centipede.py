from .starfishes import Starfish


class Centipede(Starfish):

    def _create_center(self, leftbottom_pos, n, base_size=None):
        self.center, bases = [], []

        size = self.center_radius
        lb_x, lb_y = leftbottom_pos
        bone_freq = self.materials['center'][1]['stiffness']

        a = self.space.add_node(lb_x, lb_y)
        b = self.space.add_node(lb_x, lb_y + size)
        self.space.add_link(a, b, bone_freq)
        self.center.append(a)
        self.center.append(b)
        for i in range(n):
            new_a = self.space.add_node(a.x + size, a.y)
            new_b = self.space.add_node(b.x + size, b.y)
            self.space.add_link(a, new_a, bone_freq)
            self.space.add_link(b, new_b, bone_freq)
            self.space.add_link(b, new_a, bone_freq)
            self.space.add_link(a, new_b, bone_freq)
            self.space.add_link(new_a, new_b, bone_freq)
            bases.append((new_a, a))
            a, b = new_a, new_b
            self.center.append(a)
            self.center.append(b)

        return bases
