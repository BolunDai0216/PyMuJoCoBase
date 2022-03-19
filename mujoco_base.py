import mujoco as mj
from pdb import set_trace
import numpy as np


class MuJoCoBase():
    def __init__(self, xml_path):
        # For callback functions
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0

        self.cam = mj.MjvCamera()
        self.opt = mj.MjvOption()

        # Create GLFW window
        mj.glfw.glfw.init()
        self.window = mj.glfw.glfw.create_window(1200, 900, "Demo", None, None)
        mj.glfw.glfw.make_context_current(self.window)
        mj.glfw.glfw.swap_interval(1)

        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)

        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)

        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(
            self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # install GLFW mouse and keyboard callbacks
        mj.glfw.glfw.set_key_callback(self.window, self.keyboard)
        mj.glfw.glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        mj.glfw.glfw.set_mouse_button_callback(self.window, self.mouse_button)
        mj.glfw.glfw.set_scroll_callback(self.window, self.scroll)

        self.cam.azimuth = 90.0
        self.cam.distance = 4.0
        self.cam.elevation = -45.0

    def keyboard(self, window, key, scancode, act, mods):
        if act == mj.glfw.glfw.PRESS and key == mj.glfw.glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)

    def mouse_button(self, window, button, act, mods):
        # update button state
        self.button_left = (mj.glfw.glfw.get_mouse_button(
            self.window, mj.glfw.glfw.MOUSE_BUTTON_LEFT) == mj.glfw.glfw.PRESS)
        self.button_middle = (mj.glfw.glfw.get_mouse_button(
            self.window, mj.glfw.glfw.MOUSE_BUTTON_MIDDLE) == mj.glfw.glfw.PRESS)
        self.button_right = (mj.glfw.glfw.get_mouse_button(
            self.window, mj.glfw.glfw.MOUSE_BUTTON_RIGHT) == mj.glfw.glfw.PRESS)

        # update mouse position
        mj.glfw.glfw.get_cursor_pos(self.window)

    def mouse_move(self, window, xpos, ypos):
        # compute mouse displacement, save
        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        # no buttons down: nothing to do
        if (not self.button_left) and (not self.button_middle) and (not self.button_right):
            return

        # get current window size
        width, height = mj.glfw.glfw.get_window_size(self.window)

        # get shift key state
        PRESS_LEFT_SHIFT = mj.glfw.glfw.get_key(
            window, mj.glfw.glfw.KEY_LEFT_SHIFT) == mj.glfw.glfw.PRESS
        PRESS_RIGHT_SHIFT = mj.glfw.glfw.get_key(
            window, mj.glfw.glfw.KEY_RIGHT_SHIFT) == mj.glfw.glfw.PRESS
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if self.button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        print(f"moved camera..., {dx/height}, {dy/height}")
        mj.mjv_moveCamera(self.model, action, dx/height,
                          dy/height, self.scene, self.cam)

    def scroll(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.model, action, 0.0, -0.05 *
                          yoffset, self.scene, self.cam)

    def simulate(self):
        while not mj.glfw.glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                mj.mj_step(self.model, self.data)

            viewport = mj.MjrRect(0, 0, 1200, 900)

            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            mj.glfw.glfw.swap_buffers(self.window)
            mj.glfw.glfw.poll_events()

        mj.glfw.glfw.terminate()
