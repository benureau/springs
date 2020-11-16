

class DirectMuscles:
    """One on one mapping the muscles links of the given part"""
    def __init__(self, part):
        self.muscles = part.muscles

        def __len__(self):
            return len(self.muscles)

        def actuate(self, m_signal):
            """Change the target lengths of the links to generate movement.

            :param m_signal:  motor signal, as two values, α and β. α controls the bend, can be negative
                              or positive, and its neutral value is 0. β controls the height of the
                              section, must be positive, and its neutral value is one.
            """
            for muscle, ms_i in zip(self.muscles, m_signal):
                muscle.contract(ms_i)

        def relax(self):
            """Return the target lengths of the links of the section to their relax lengths"""
            for muscle in self.muscles:
                muscle.relax()


class SectionOneMuscle:
    """Make antagonist external muscles work together in a section.

    This is the same class as `SectionTwoMuscles`,
    One parameter influence the bend of the section (when one muscle contract, the other expands),
    while the other influence the height (both muscles contract or expand together)
    """

    def __init__(self, section):  # QUESTION: Warning if not actuated?
        # self.section = section
        self.left_muscle  = section.link_map['left']
        self.right_muscle = section.link_map['right']

    def __len__(self):
        return 1

    def actuate(self, m_signal):
        """Change the target lengths of the links to generate movement.

        :param m_signal:  motor signal, as two values, α and β. α controls the bend, can be negative
                          or positive, and its neutral value is 0. β controls the height of the
                          section, must be positive, and its neutral value is one.
        """
        α = m_signal[0]
        self.left_muscle .contract(1 - α)
        self.right_muscle.contract(1 + α)

    def relax(self):
        """Return the target lengths of the links of the section to their relax lengths"""
        self.left_muscle .relax()
        self.right_muscle.relax()


class SectionTwoMuscles:
    """Make antagonist external muscles work together in a section.

    One parameter influence the bend of the section (when one muscle contract, the other expands),
    while the other influence the height (both muscles contract or expand together)
    """

    def __init__(self, section):  # QUESTION: Warning if not actuated?
        # self.section = section
        self.left_muscle  = section.link_map['left']
        self.right_muscle = section.link_map['right']

    def __len__(self):
        return 2

    def actuate(self, m_signal):
        """Change the target lengths of the links to generate movement.

        :param m_signal:  motor signal, as two values, α and β. α controls the bend, can be negative
                          or positive, and its neutral value is 0. β controls the height of the
                          section, must be positive, and its neutral value is one.
        """
        α, β = m_signal
        self.left_muscle .contract((1 - α) * β)
        self.right_muscle.contract((1 + α) * β)

    def relax(self):
        """Return the target lengths of the links of the section to their relax lengths"""
        self.left_muscle .relax()
        self.right_muscle.relax()


class MuscleBroadcast:
    """Broadcast the same motor activation to different muscle classes"""

    def __init__(self, groups):
        self.groups = groups
        assert all(len(group) == len(self.groups[0]) for group in self.groups)
        self.length = len(self.groups[0])

    def add_group(self, group):
        self.groups.append(group)

    def __len__(self):
        return self.length

    def actuate(self, m_signal):
        for group in self.groups:
            group.actuate(m_signal)

    def relax(self):
        for group in self.groups:
            group.relax()


class MuscleInterface:

    def __init__(self, groups):
        self.groups = list(groups)
        self.length = sum(len(group) for group in self.groups)

    def add_group(self, group):
        self.groups.append(group)
        self.length += len(group)

    def __len__(self):
        return self.length

    def __iter__(self):
        return (group for group in self.groups)

    def actuate(self, m_signal):
        """Change the target lengths of the muscle groups to generate movement.

        :param m_signal:  motor signal, as a list of scalar values.
        """
        assert self.length == len(m_signal)
        offset = 0
        for group in self.groups:
            group.actuate(m_signal[offset:offset+len(group)])
            offset += len(group)

    def relax(self):
        """Return the target lengths of the links of the groups to their relax lengths"""
        for group in self.groups:
            group.relax()
