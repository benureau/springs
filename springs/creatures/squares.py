import math
import numpy as np



class SideLink:

    def __init__(self, space, nodeA, nodeB, frequency, damping=1.0, actuated=True,
                       min_amp=0.5, max_amp=1.5):
        self.link = space.add_link(nodeA, nodeB, frequency, damping_ratio=1.0,
                                   actuated=actuated)
        self.min_amp, self.max_amp = min_amp, max_amp
        self.squares = []

    def actuate(self):
        if len(self.squares) == 2:
            s0, s1 = self.squares
            m_signal = 1 + (s0.freq * s0.m_signal + s1.freq * s1.m_signal)/(s0.freq + s1.freq)
        elif len(self.squares) == 1:
            m_signal = 1 + self.squares[0].m_signal
        else:
            raise ValueError('number of squares is not 1 or 2 (is {})'.format(len(self.squares)))
        self.link.contract(max(self.min_amp, min(self.max_amp, m_signal)))

    def relax(self):
        size, sum_stif = 0, 0
        for s in self.squares:
            size += s.stiffness * s.size
            sum_stif += s.stiffness
        self.link.relax_length = size/sum_stif


class Square:

    def __init__(self, space, size, stiffness, neighbors, center, start_size,
                       amp_limit=0.5, amp_factor=0.5, α_freq=20.0, β_freq=10.0, γ_stif=4, friction=0.5):
        self.space = space
        self.nodeBL, self.nodeBR, self.nodeUL, self.nodeUR = None, None, None, None
        self.linkU,  self.linkB,  self.linkL,  self.linkR  = None, None, None, None
        self.neighborU, self.neighborB, self.neighborL, self.neighborR = neighbors

        self.size, self.stiffness = size, stiffness + 10e-10
        self.freq = α_freq * stiffness + β_freq
        self.amp_factor = amp_factor / (self.stiffness + 1)**γ_stif
        self.min_amp, self.max_amp = 1 - amp_limit, 1 + amp_limit
        self.friction = friction

        self.m_signal = 0
        self._create(center, start_size)

    def _create(self, center, start_size):
        # populating the square with existing nodes and links
        if self.neighborU is not None:
            self.nodeUL, self.nodeUR = self.neighborU.nodeBL, self.neighborU.nodeBR
            self.linkU = self.neighborU.linkB
        if self.neighborB is not None:
            self.nodeBL, self.nodeBR = self.neighborB.nodeUL, self.neighborB.nodeUR
            self.linkB = self.neighborB.linkU
        if self.neighborL is not None:
            self.nodeUL, self.nodeBL = self.neighborL.nodeUR, self.neighborL.nodeBR
            self.linkL = self.neighborL.linkR
        if self.neighborR is not None:
            self.nodeUR, self.nodeBR = self.neighborR.nodeUL, self.neighborR.nodeBL
            self.linkR = self.neighborR.linkL

        # creating missing nodes
        if self.nodeBL is None:
            self.nodeBL = self.space.add_node(center[0] - start_size/2, center[1] - start_size/2, friction=self.friction)
        if self.nodeBR is None:
            self.nodeBR = self.space.add_node(center[0] + start_size/2, center[1] - start_size/2, friction=self.friction)
        if self.nodeUL is None:
            self.nodeUL = self.space.add_node(center[0] - start_size/2, center[1] + start_size/2, friction=self.friction)
        if self.nodeUR is None:
            self.nodeUR = self.space.add_node(center[0] + start_size/2, center[1] + start_size/2, friction=self.friction)

        # creating missing side links
        if self.linkU is None:
            self.linkU = SideLink(self.space, self.nodeUL, self.nodeUR, self.freq, actuated=True,
                                  min_amp=self.min_amp, max_amp=self.max_amp)
        self.linkU.squares.append(self)
        if self.linkB is None:
            self.linkB = SideLink(self.space, self.nodeBL, self.nodeBR, self.freq, actuated=True,
                                  min_amp=self.min_amp, max_amp=self.max_amp)
        self.linkB.squares.append(self)
        if self.linkL is None:
            self.linkL = SideLink(self.space, self.nodeUL, self.nodeBL, self.freq, actuated=True,
                                  min_amp=self.min_amp, max_amp=self.max_amp)
        self.linkL.squares.append(self)
        if self.linkR is None:
            self.linkR = SideLink(self.space, self.nodeUR, self.nodeBR, self.freq, actuated=True,
                                  min_amp=self.min_amp, max_amp=self.max_amp)
        self.linkR.squares.append(self)

        # creating diagonal links
        self.diagA = self.space.add_link(self.nodeUL, self.nodeBR, self.freq, actuated=True)
        self.diagB = self.space.add_link(self.nodeUR, self.nodeBL, self.freq, actuated=True)

    def actuate(self, m_signal):
        self.m_signal = self.amp_factor * m_signal
        self.diagA.contract(max(self.min_amp, min(self.max_amp, 1 + self.m_signal)))
        self.diagB.contract(max(self.min_amp, min(self.max_amp, 1 + self.m_signal)))


class FusedSquares:

    def __init__(self, space, square_size, square_stiffness,
                       damping=1.0, period=1.0, growth_factor=1.0, origin=(0, 0),
                       amp_limit=0.4, α_freq=20.0, β_freq=10.0, γ_stif=4, **kwargs):
        self.space       = space
        self.shape       = (len(square_size), len(square_size[0]))
        self.square_size = square_size
        self.square_stif = square_stiffness
        self.damping     = damping
        self.period      = period
        self.growth_factor = growth_factor

        self.square_matrix = [[None for j in range(self.shape[1])] for i in range(self.shape[0])]
        self._create(origin)


    def _create(self, origin):
        avg_size = self.growth_factor * np.mean(self.square_size)
        temp_side_map = {}
        n, m = self.shape

        for i in range(n):
            for j in range(m):
                neighborU, neighborB, neighborL, neighborR = None, None, None, None
                if i > 0:
                    neighborL = self.square_matrix[i-1][j]
                if i < n-1:
                    neighborR = self.square_matrix[i+1][j]
                if j > 0:
                    neighborB = self.square_matrix[i][j-1]
                if j < m-1:
                    neighborU = self.square_matrix[i][j+1]
                square = Square(self.space, self.square_size[i][j], self.square_stif[i][j],
                                (neighborU, neighborB, neighborL, neighborR),
                                (origin[0] + avg_size*(0.5 + i), origin[1] + avg_size*(0.5 + j)), avg_size)
                self.square_matrix[i][j] = square
                for sidelink in [square.linkU, square.linkB, square.linkL, square.linkR]:
                    temp_side_map[id(sidelink)] = sidelink

        self.sidelinks = [link for link in temp_side_map.values()]
        for sidelink in self.sidelinks:
            sidelink.relax()


    def actuate(self, m_signal):
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                m_i = m_signal[i * self.shape[1] + j]
                self.square_matrix[i][j].actuate(m_i)
        for sidelink in self.sidelinks:
            sidelink.actuate()

    @property
    def avg_center(self):
        #FIXME: actual average
        return self.square_matrix[0][0].nodeBL.position


class Squares:

    def __init__(self, space, square_size, square_stiffness, square_actuation,
                       damping=1.0, period=1.0, growth_factor=1.0, origin=(0, 0), **kwargs):
        self.space       = space
        self.square_size = square_size
        self.square_stif = square_stiffness
        self.square_actu = square_actuation
        self.damping     = damping
        self.period      = period
        self.growth_factor = growth_factor
        self._create(origin)

    def _stif2freq(self, stif):
        return np.exp(5.0 * stif + 2)

    def _create(self, origin):
        avg_size = self.growth_factor * np.mean(self.square_size)

        n, m = len(self.square_size), len(self.square_size[0])
        self.node_matrix = []

        # creating squares
        for i in range(n):
            self.node_matrix.append([])
            for j in range(m):
                nodeUL = self.space.add_node(origin[0] + avg_size*i,     origin[1] + avg_size*j)
                nodeUR = self.space.add_node(origin[0] + avg_size*i,     origin[1] + avg_size*(j+1))
                nodeBL = self.space.add_node(origin[0] + avg_size*(i+1), origin[1] + avg_size*j)
                nodeBR = self.space.add_node(origin[0] + avg_size*(i+1), origin[1] + avg_size*(j+1))
                self.node_matrix[-1].append(((nodeUL, nodeUR), (nodeBL, nodeBR)))

                freq  = self._stif2freq(self.square_stif[i][j])
                relax_length = self.square_size[n-1][j]

                linkU = self.space.add_link(nodeUL, nodeUR, freq)
                linkB = self.space.add_link(nodeBL, nodeBR, freq)
                linkR = self.space.add_link(nodeUR, nodeBR, freq)
                linkL = self.space.add_link(nodeUL, nodeBL, freq)
                diagA = self.space.add_link(nodeUL, nodeBR, freq)
                diagB = self.space.add_link(nodeUR, nodeBL, freq)

                for link in [linkU, linkB, linkR, linkL]:
                    link.relax_length = self.square_size[i][j]
                for link in [diagA, diagB]:
                    link.relax_length = np.sqrt(2) * self.square_size[i][j]

        # stiching square up with their neightbors
        for i in range(n):
            for j in range(m):
                if i < n-1:
                    self.space.add_link(self.node_matrix[i][j][1][0], self.node_matrix[i+1][j][0][0], 250, damping_ratio=1.0)
                    self.space.add_link(self.node_matrix[i][j][1][1], self.node_matrix[i+1][j][0][1], 250, damping_ratio=1.0)
                if j < m-1:
                    self.space.add_link(self.node_matrix[i][j][0][1], self.node_matrix[i][j+1][0][0], 250, damping_ratio=1.0)
                    self.space.add_link(self.node_matrix[i][j][1][1], self.node_matrix[i][j+1][1][0], 250, damping_ratio=1.0)
