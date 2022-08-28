from math import inf
import numpy as np
from matplotlib.backend_bases import MouseButton
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import matplotlib.pyplot as plt


fig, ax = plt.subplots()

pathdata = [
    (Path.MOVETO, (1.58, -2.57)),
    (Path.CURVE4, (0.35, -1.1)),
    (Path.CURVE4, (-1.75, 2.0)),
    (Path.CURVE4, (0.375, 2.0)),
    (Path.LINETO, (0.375, 2.0)),
    (Path.CURVE4, (2.2, 3.2)),
    (Path.CURVE4, (3, 0.05)),
    (Path.CURVE4, (1.58, -2.57)),
    # (Path.CLOSEPOLY, (1.58, -2.57)),
]

START_POINT = 0
CONTROL_POINT_1 = 1
CONTROL_POINT_2 = 2
END_POINT = 3
flags = [
    START_POINT,
    CONTROL_POINT_1,
    CONTROL_POINT_2,
    END_POINT,
    START_POINT,
    CONTROL_POINT_1,
    CONTROL_POINT_2,
    END_POINT,
]

codes, verts = zip(*pathdata)
path = Path(verts, codes)
patch = PathPatch(
    path, facecolor='white', edgecolor='red', alpha=1.0)
ax.add_patch(patch)


class PathInteractor:
    """
    An path editor.

    Press 't' to toggle vertex markers on and off.  When vertex markers are on,
    they can be dragged with the mouse.
    """

    showverts = True
    epsilon = 10  # max pixel distance to count as a vertex hit

    def __init__(self, pathpatch):

        self.ax = pathpatch.axes
        canvas = self.ax.figure.canvas
        self.pathpatch = pathpatch
        self.pathpatch.set_animated(True)

        x, y = zip(*self.pathpatch.get_path().vertices)

        x = np.roll(x, 2)
        y = np.roll(y, 2)

        self.line1, = ax.plot(
            x[:4], y[:4], marker='o', markerfacecolor='r', animated=True)

        self.line2, = ax.plot(
            x[4:], y[4:], marker='o', markerfacecolor='r', animated=True)

        self._ind = None  # the active vertex

        self._lock_heading = False

        canvas.mpl_connect('draw_event', self.on_draw)
        canvas.mpl_connect('button_press_event', self.on_button_press)
        canvas.mpl_connect('key_press_event', self.on_key_press)
        canvas.mpl_connect('button_release_event', self.on_button_release)
        canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas = canvas

    def get_ind_under_point(self, event):
        """
        Return the index of the point closest to the event position or *None*
        if no point is within ``self.epsilon`` to the event position.
        """
        # display coords
        xy = np.asarray(self.pathpatch.get_path().vertices)
        xyt = self.pathpatch.get_transform().transform(xy)
        xt, yt = xyt[:, 0], xyt[:, 1]
        d = np.sqrt((xt - event.x)**2 + (yt - event.y)**2)
        ind = d.argmin()

        if d[ind] >= self.epsilon:
            ind = None

        return ind

    def on_draw(self, event):
        """Callback for draws."""
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.pathpatch)
        self.ax.draw_artist(self.line1)
        self.ax.draw_artist(self.line2)
        self.canvas.blit(self.ax.bbox)

    def on_button_press(self, event):
        """Callback for mouse button presses."""
        if (event.inaxes is None
                or event.button != MouseButton.LEFT
                or not self.showverts):
            return
        self._ind = self.get_ind_under_point(event)

    def on_button_release(self, event):
        """Callback for mouse button releases."""
        if (event.button != MouseButton.LEFT
                or not self.showverts):
            return
        self._ind = None

    def on_key_press(self, event):
        """Callback for key presses."""
        if not event.inaxes:
            return
        if event.key == 't':
            self.showverts = not self.showverts
            self.line1.set_visible(self.showverts)
            self.line2.set_visible(self.showverts)
            if not self.showverts:
                self._ind = None
        if event.key == 'e':
            if self._lock_heading:
                print("Unlock heading")
            else:
                print("Lock heading")
            self._lock_heading = not self._lock_heading
        self.canvas.draw()

    def on_mouse_move(self, event):
        """Callback for mouse movements."""
        if (self._ind is None
                or event.inaxes is None
                or event.button != MouseButton.LEFT
                or not self.showverts):
            return

        vertices = self.pathpatch.get_path().vertices
        roll = 2 - 4 * ((self._ind + 2) // 4)
        vertices_roll = np.roll(vertices, roll, axis=0)
        point_type = self._ind % 4
        if point_type == START_POINT or point_type == END_POINT:
            offset = np.array([event.xdata, event.ydata]) - vertices[self._ind] 
            vertices_roll[0:4] += offset
        elif self._lock_heading:
            if point_type == CONTROL_POINT_1:
                px, py = vertices_roll[0]
                qx, qy = vertices_roll[1]
            elif point_type == CONTROL_POINT_2:
                px, py = vertices_roll[2]
                qx, qy = vertices_roll[3]
            slope = (py - qy) / (px - qx)
            intercept = py - px * slope
            slope_otho = -1.0 / slope
            intercept_otho = event.ydata - event.xdata * slope_otho
            if abs(slope_otho) == inf:
                x_intersect, y_intersect = event.xdata, py
            elif slope_otho == 0.0:
                x_intersect, y_intersect = px, event.ydata
            else:
                x_intersect = (intercept_otho - intercept) / (slope - slope_otho)
                y_intersect = slope * x_intersect + intercept
            vertices_roll[(point_type + 2) % 4] = x_intersect, y_intersect
        else:
            if point_type == CONTROL_POINT_1:
                vertices_roll[3] = event.xdata, event.ydata
                px, py = vertices_roll[2]
                qx, qy = vertices_roll[3]
                direction = np.arctan2(py-qy, px-qx)
                d = np.linalg.norm(vertices_roll[1] - vertices_roll[0])
                vertices_roll[0] = px + np.cos(direction) * d, py + np.sin(direction) * d
            elif point_type == CONTROL_POINT_2:
                vertices_roll[0] = event.xdata, event.ydata
                px, py = vertices_roll[1]
                qx, qy = vertices_roll[0]
                direction = np.arctan2(py-qy, px-qx)
                d = np.linalg.norm(vertices_roll[2] - vertices_roll[3])
                vertices_roll[3] = px + np.cos(direction) * d, py + np.sin(direction) * d
        vertices = np.roll(vertices_roll, -1 * roll, axis=0)
        self.pathpatch.get_path().vertices = vertices
        vertices_roll = np.roll(vertices, 2, axis=0)

        self.line1.set_data(vertices_roll[:4].T)
        self.line2.set_data(vertices_roll[4:].T)

        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.pathpatch)
        self.ax.draw_artist(self.line1)
        self.ax.draw_artist(self.line2) 
        self.canvas.blit(self.ax.bbox)


interactor = PathInteractor(patch)
ax.set_title('Create the Initial Path Curve')
ax.set_xlim(-3, 4)
ax.set_ylim(-3, 4)

plt.show()