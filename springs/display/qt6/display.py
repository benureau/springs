import os
import sys
import math
import signal
from pathlib import Path
from collections.abc import Iterable

from PyQt6.QtWidgets import QWidget, QApplication
from PyQt6.QtGui import QPainter, QPainterPath, QColor, QPen, QIcon, QFont, QColorConstants
from PyQt6.QtCore import Qt, QTimer, QPoint
from PyQt6 import QtCore, QtGui

import numpy as np
import setproctitle


setproctitle.setproctitle('Springs')
# ctrl-c to exit the program without fuss.
signal.signal(signal.SIGINT, signal.SIG_DFL)


class SpaceCanvas(QWidget):

    HUD_FONT_SIZE = 20

    def __init__(self, spaces, update_hook=None, width=900, height=900, duration=float('inf'),
                       node_radius=2.5, link_width=1.1, spf=0.02, zoom=1.0, paused=False,
                       autotranslate=True, origin_x=0, origin_y=-100, size_factor=1.0, hud=True,
                       viewport=None, message=None):
        """
        size_factor draws all the objects smaller or bigger.
        """
        super().__init__()

        self.message = message
        self.spaces = spaces
        self.spf, self.hud = spf, hud
        self._ref_width, self._ref_height, self._ref_zoom = width, height, zoom
        self.zoom, self.size_factor, self.viewport = zoom, size_factor, viewport
        self.origin_x, self.origin_y = origin_x, origin_y
        self.setGeometry(0, 0, width, height)

        self.setWindowTitle('Springs Simulator')
        self.show()

        self.node_color   = QColor( 75,  75,  75)
        self.grip_color   = QColor(255,   0,   0)
        self.bone_color   = QColor(100, 100, 100)

        self.spring_color = QColor(  0, 200, 255)
        self.muscle_color = QColor(255, 127,   0)

        #self.spring_color     = QColor(255,   0,   0, 127)
        #self.muscle_color     = QColor(253,  28, 184)
        self.background_color = QColor(255, 255, 255)
        self.grey             = QColor(150, 150, 150)
        self.light_grey       = QColor(240, 240, 240)
        self.white            = QColor(255, 255, 255)

        p = self.palette()
        p.setColor(self.backgroundRole(), self.background_color)
        self.setPalette(p)
        self.setAutoFillBackground(True)

        self._paused = paused
        self._one_step = False  # if True, perform one step and pause

        self.update_hook = update_hook
        #self.start_timer()

        self.node_radius = node_radius
        self.link_width  = link_width

        self.autotranslate_flag = autotranslate


    def _autozoom(self):
        width_factor = self.width()/self._ref_width
        height_factor = self.height()/self._ref_height
        factor = min(width_factor, height_factor)
        self.zoom = factor * self._ref_zoom

    def _draw_accelerometer(self, qp, sensor):
        return
        qp.setPen(QPen( QColor(255, 0, 0), 1.1))
        pos = np.array(sensor.node.node.position)

        qp.drawLine(QtCore.QPointF(*pos), QtCore.QPointF(pos[0] + sensor.value[0][0],
                                                         pos[1] + sensor.value[0][1]))

    def visible_coordinates(self):
        return (self.origin_x, self.origin_x +  self.width()/self.zoom,
                self.origin_y, self.origin_y + self.height()/self.zoom)

    def autotranslate(self):

        if self.viewport is not None:
            xL, xR = self.viewport[0]
            yB, yT = self.viewport[1]
            self.origin_x = xL
            self.origin_y = yB
            self.zoom = min(self.width() / (xR - xL), self.height() / (yT - yB))

        elif len(self.spaces[0].entities) > 0:
            first_entity = self.spaces[0].entities[-1]
            if hasattr(first_entity, 'center_xy'):
                frame_xL, frame_xR, frame_yB, frame_yT = self.visible_coordinates()
                x, y = first_entity.center_xy
                x, y = self.size_factor * x, self.size_factor * y
                # screen_pos = self.mapToGlobal(QPoint(x, y))
                margin_x = (frame_xR - frame_xL) / 10
                margin_y = (frame_yT - frame_yB) / 100
                if x < frame_xL + margin_x:
                    self.origin_x -= (frame_xL + margin_x) - x
                elif frame_xR - margin_x < x:
                    self.origin_x += x - (frame_xR - margin_x)
                if y < frame_yB + margin_y:
                    self.origin_y -= (frame_yB + margin_y) - y
                elif frame_yT - margin_y < y:
                    self.origin_y += y - (frame_yB - margin_y)

    def draw_milestones(self, qp):
        frame_xL, frame_xR, frame_yB, frame_yT = self.visible_coordinates()

#        qp.setPen(QPen(QColor(185, 185, 185), 2))
        qp.setPen(QPen(QColor(128, 128, 128), 2))
        qp.setFont(QFont("Helvetica", self.HUD_FONT_SIZE * 1.8, QFont.Weight.Bold))
        qp.save()
        qp.scale(1,-1)
        m_size = 789.2820323027551
        for i in range(int(frame_xL//m_size) - 1, int(frame_xR//m_size) + 2):
            qp.drawText(i * m_size - 18, 80, "{}".format(i))
        qp.restore()

#        qp.setPen(QPen(QColor(200, 200, 200), 2, Qt.DotLine))
        qp.setPen(QPen(QColor(200, 200, 200), 3))
        for i in range(int(frame_xL//m_size) - 1, int(frame_xR//m_size) + 2):
            qp.drawLine(QtCore.QPointF(i * m_size, -20),
                        QtCore.QPointF(i * m_size, 0))

    def draw_hud(self, qp):
        # frame_xL, frame_xR, frame_yB, frame_yT = self.visible_coordinates()

        spacing = self.HUD_FONT_SIZE * 1.4

        qp.setPen(QPen(QColor(185, 185, 185), 2))
        qp.setFont(QFont("Helvetica", self.HUD_FONT_SIZE, QFont.Weight.Bold))
        # qp.save()
        # qp.scale(1,-1)
        # qp.translate(0, -self.height() / self.zoom)
        if self.message is not None:
            qp.drawText(20, spacing, self.message)


        qp.drawText(20, spacing * 3, "{:.1f}s".format(self.spaces[0].t))
        if len(self.spaces[0].entities) > 0:
            first_entity = self.spaces[0].entities[-1]
            if hasattr(first_entity, 'center_xy'):
                x, y = first_entity.center_xy
                x, y = self.size_factor * x, self.size_factor * y
                qp.drawText(20, spacing * 2, "{:.2f} adult bodylengths".format(x/789.2820323027551))


        # qp.drawText(frame_xL + 20/self.zoom, frame_yB +  55/self.zoom, "{:.1f}s".format(self.spaces[0].t))
        # x, y = self.spaces[0].entities[-1].center_xy
        # qp.drawText(frame_xL + 20/self.zoom, frame_yB + 105/self.zoom, "{:.2f}m".format(x/1000))

        # qp.restore()


    def paintEvent(self, e):
        self._autozoom()
        if self.autotranslate_flag:
            self.autotranslate()

        qp = QPainter()
        qp.begin(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        qp.save()
        qp.scale(self.zoom, -self.zoom)
        # rotate so that gravity point down
        gravity_x, gravity_y = self.spaces[0].gravity
        if not gravity_x == gravity_y == 0:  # if no gravity, y axis points up.
            qp.rotate(math.degrees(- math.atan2(gravity_y, gravity_x) - 1.5707963267948966))
        qp.translate(-self.origin_x, - self.origin_y - self.height()/self.zoom)

        for space in self.spaces:
            # for entity in space.entities:
            #     for sensor in entity.sensors:
            #         if sensor.name == 'accelerometer':
            #             self._draw_accelerometer(qp, sensor)

            for link in space.links:
                self.draw_link(qp, link)

            # for link in space.springs:
            #     self.draw_link(qp, link)

            for node in space.nodes:
                self.draw_node(qp, node)

            for rect in space.rects:
                self.draw_rect(qp, rect)

            for triangle in space.triangles:
                self.draw_triangle(qp, triangle)

        if self.hud:
            self.draw_milestones(qp)

        qp.restore()
        if self.hud:
            self.draw_hud(qp)
        qp.end()

    def draw_link(self, qp, link):
        # if link.actuated:
        #     pen_color = self.muscle_color
        # else:
        #     if link.stiffness < 1000:
        #         pen
        #         pen_color = self.spring_color
        #     else:
        #         pen_color = self.bone_color

        if link.actuated:
            grey = 200
            contract = min(1.0, max(-1.0, 10.0*(1.0 - link.expand_factor)))
            contract = -contract
            if contract <= 0.0:
                pen_color = QColor((1+contract)*grey - contract*255, (1+contract)*grey - contract*40, (1+contract)*grey)
            else:
                pen_color = QColor((1-contract)*grey, (1-contract)*grey + contract*200, (1-contract)*grey + contract*255)
        else:
            grey = 150
            pen_color = QColor(200, 200, 200)

        # stress = 2.0*min(1, abs((link.length() - link.relax_length)/link.relax_length))
        # grey = 125#235 - 10 * math.log(link.stiffness + 1)
        # pen_color = QColor((1-stress)*grey + stress*255, (1-stress)*grey + stress*40, (1-stress)*grey)

        pen = QPen(pen_color)
        if link.actuated:
            pen.setWidthF(2*self.link_width)
            # pen.setWidthF(self.link_width*(2 + 4*stress))
        else:
            pen.setWidthF(self.link_width)
        qp.setPen(pen)
        qp.drawLine(QtCore.QPointF(self.size_factor*link.node_a.x, self.size_factor*link.node_a.y),
                    QtCore.QPointF(self.size_factor*link.node_b.x, self.size_factor*link.node_b.y))

    def draw_node(self, qp, node):
        if node.fixed:
            qp.setPen(QPen(self.grip_color, 1))
            qp.setBrush(self.grip_color)
            radius = 1.5 * self.node_radius
        else:
            qp.setPen(QPen(self.node_color, 1))
            qp.setBrush(self.node_color)
            radius = self.node_radius
        qp.drawEllipse(QtCore.QPointF(self.size_factor*node.x, self.size_factor*node.y), radius, radius)

    def draw_rect(self, qp, rect):
        qp.setPen(QPen(self.grey, 1))
        qp.setBrush(QColor(128, 128, 128, 25))
        qp.setBrush(QColorConstants.Transparent)
        qp.drawRect(self.size_factor*rect.xL, self.size_factor*rect.yB, self.size_factor*rect.width, self.size_factor*rect.height)

    def draw_triangle(self, qp, triangle):
        qp.setPen(QPen(self.grey, 1))
        # qp.setBrush(QColor(128, 128, 128, 25))
        qp.setBrush(QColorConstants.Transparent)
        points = QtGui.QPolygonF([QtCore.QPointF(self.size_factor*triangle.xA, self.size_factor*triangle.yA),
                                  QtCore.QPointF(self.size_factor*triangle.xB, self.size_factor*triangle.yB),
                                  QtCore.QPointF(self.size_factor*triangle.xC, self.size_factor*triangle.yC)])
        qp.drawPolygon(points)


    def _cross(self, qp, x, y, size=3):
        qp.setPen(QPen(self.grey, 1))
        qp.drawEllipse(QtCore.QPointF(x, y), 0.7, 0.7)

        # qp.drawLine(QtCore.QPointF(x - size, y), QtCore.QPointF(x + size, y))
        # qp.drawLine(QtCore.QPointF(x, y - size), QtCore.QPointF(x, y + size))

    def pause(self):
        self._paused = not(self._paused)

    def keyPressEvent(self, event):
        if type(event) == QtGui.QKeyEvent:
            if event.key() == Qt.Key_P:
                self.pause()
            elif event.key() == Qt.Key_N:
                self._one_step = True
            event.accept()
        else:
            event.ignore()


#TODO: exit when finished (pause default)
class SpaceDisplay(QApplication):

    def __init__(self, spaces, width=900, height=900, update_hook=None, duration=float('inf'),
                       spf=0.02, zoom=1.0, display=None, paused=False, pause_at_end=False, steps=None,
                       screenshot_folder=None, fullscreen=False, hud=True, start_skip=0,
                       origin_x=0, origin_y=-100, **kwargs):
        """
        :param spf:       seconds per frame.
        :param display:   display number. `None` for letting pyqt choose (probably 0).
        :param duration:  duration of the simulation. Does not yield precise results. Use `steps`
                          instead to get repeatable results. Setting `steps` overrides the `duration`
                          parameter. Default to infinity. Note that when simulating multiple
                          spaces, only the first space matter when checking for the duration.
        :param steps:     how many step of the physic engine to compute. Overrides the `duration`
                          parameter if not None.
        """
        super().__init__(sys.argv)


        self._set_icon()
        setproctitle.setproctitle('Springs Simulator')
        self.setApplicationName('Springs Simulator')

        self.screenshot_folder = screenshot_folder
        self.screenshot_count  = 0

        self.spaces = spaces
        if not isinstance(self.spaces, Iterable):
            self.spaces = [spaces]

        # skip to start space time if needed
        while self.spaces[0].t < start_skip:
            for space in self.spaces:
                space.step()

        self.space_canvas = SpaceCanvas(self.spaces, update_hook=update_hook, duration=duration,
                                        width=width, height=height, spf=spf, zoom=zoom,
                                        paused=paused, origin_x=origin_x, origin_y=origin_y,
                                        hud=hud, **kwargs)
        if display is not None:
            self.screen = self.screens()[display].availableGeometry()
        else:
            self.screen = self.primaryScreen().availableGeometry()

        self.space_canvas.move(self.screen.right() - self.space_canvas.width(), self.screen.top() + 100)
        if fullscreen:
            self.space_canvas.showFullScreen()

        self.steps = steps
        self.duration = duration

        self.pause_at_end = pause_at_end

        self.start_timer()

    def start_timer(self):
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        # self.timer.setTimerType(Qt.QTimer)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start()

    def _finished(self):
        """Return True if the total number of step or duration has been reached"""
        if self.steps is not None:
            return self.spaces[0].ticks >= self.steps
        else:
            return self.spaces[0].t > self.duration

    def step(self):
        if self._finished(): # we avoid computing more steps
            if self.space_canvas._paused:
                return
            elif self.pause_at_end:
                self.space_canvas.pause()
                self.pause_at_end = False
                return
            else:
                self.exit()
                return
        for space in self.spaces:
            space.step()

    def exit(self):
        super().exit()

    @QtCore.pyqtSlot()
    def on_timer(self):
        if self.spaces[0].t > self.duration:
            self.exit()

        elif self.space_canvas._one_step:
            self.step()
            self.space_canvas._one_step = False
            self.space_canvas._paused = True

        elif not self.space_canvas._paused:
            for _ in range(max(1, round(self.space_canvas.spf/self.spaces[0].dt))):
                self.step()

        self.space_canvas.update()
        self._save_screenshot()

    def _save_screenshot(self):
        if self.screenshot_folder is not None and not self.space_canvas._paused:
            png_path = str(Path(self.screenshot_folder, 'shot{:05d}.png'.format(self.screenshot_count)))
            # TODO: expanduser, create if does not exist.
            self.space_canvas.grab().save(png_path)
            space = self.space_canvas.spaces[0]
            print('saving screenshot {} ({:.2f}s:{}ticks)'.format(png_path, space.t, space.ticks))
            self.screenshot_count += 1

    def _set_icon(self):
        path = os.path.join(os.path.dirname(__file__), 'icon.png')
        self.setWindowIcon(QIcon(path))

    def run(self):
        return self.exec()


if __name__ == '__main__':
    app = SpringApp(sys.argv)
