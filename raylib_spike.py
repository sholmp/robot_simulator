from pyray import *
import numpy as np


class Robot:
    def __init__(self):
        self.q = np.array([np.pi/8, np.pi/8])
        self.x0 = np.array([100, 100])

        self.l1 = 100
        self.l2 = 100

    def setQ(self, q : np.array):
        self.q = q

    def draw(self):
        q0 = self.q[0]
        q1 = self.q[1]
        x0 = self.x0[0]
        y0 = self.x0[1]

        x1 = x0 + int(self.l1 * np.cos(q0))
        y1 = y0 + int(self.l1 * np.sin(q0))
        x2 = x1 + int(self.l2 * np.cos(q0 + q1))
        y2 = y1 + int(self.l2 * np.sin(q0 + q1))

        draw_line(x0, y0, x1, y1, VIOLET)
        draw_line(x1, y1, x2, y2, VIOLET)


    def endEffectorPosition(self):
        q0 = self.q[0]
        q1 = self.q[1]

        xe = self.x0[0] + self.l1 * np.cos(q0) + self.l2 * np.cos(q0 + q1)
        ye = self.x0[1] + self.l1 * np.sin(q0) + self.l2 * np.sin(q0 + q1)

        return np.array([xe, ye])

    def jacobian(self):
        l1 = self.l1
        l2 = self.l2
        q0 = self.q[0]
        q1 = self.q[1]

        J00 = -l1 * np.sin(q0)
        J01 = -l2 * np.sin(q0 + q1)
        J10 = l1 * np.cos(q0)
        J11 = l2 * np.cos(q0 + q1)

        J = np.array([[J00, J01], [J10, J11]])
        return J

    def invkin(self, x_desired):
        dx = x_desired - self.endEffectorPosition()
        while np.linalg.norm(dx) > 0.00001:
            dq = np.linalg.inv(r.jacobian()) @ dx
            r.setQ(self.q + dq)
            dx = x_desired - self.endEffectorPosition()

        return self.q

init_window(800, 450, "Hello")
r = Robot()

qs = np.linspace(0,2 * np.pi, 50000)

idx = 0

q0, q1 = r.invkin([170, 270])
print(r.endEffectorPosition())

print(q0, q1)

while not window_should_close():
    begin_drawing()
    clear_background(WHITE)

    r.draw()

    draw_text("Hello world", 190, 200, 20, VIOLET)
    end_drawing()
close_window()
