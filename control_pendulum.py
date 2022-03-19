import mujoco as mj
import numpy as np

from mujoco_base import MuJoCoBase


class ContolPendulum(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)

    def reset(self):
        # Set initial angle of pendulum
        self.data.qpos[0] = np.pi/2

        # Set camera configuration
        self.cam.azimuth = 90.0
        self.cam.distance = 5.0
        self.cam.elevation = -5
        self.cam.lookat = np.array([0.012768, -0.000000, 1.254336])

    def controller(self, actuator_type="torque"):
        """
        This function implements a PD controller

        Since there are no gravity compensation,
        it will not be very accurate at tracking
        the set point. It will be accurate is
        gravity is turned off.
        """
        if actuator_type == "torque":
            self.model.actuator_gainprm[0, 0] = 1
            self.data.ctrl[0] = -10 * \
                (self.data.sensordata[0] - 0.0) - \
                1 * (self.data.sensordata[1] - 0.0)
        elif actuator_type == "servo":
            kp = 10.0
            self.model.actuator_gainprm[1, 0] = kp
            self.model.actuator_biasprm[1, 1] = -kp
            self.data.ctrl[1] = -0.5

            kv = 1.0
            self.model.actuator_gainprm[2, 0] = kv
            self.model.actuator_biasprm[2, 2] = -kv
            self.data.ctrl[2] = 0.0

    def simulate(self):
        while not mj.glfw.glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                # Apply PD controller
                self.controller(actuator_type="servo")

                # Step simulation environment
                mj.mj_step(self.model, self.data)

            print(self.data.qpos[0])

            # get framebuffer viewport
            viewport_width, viewport_height = mj.glfw.glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            mj.glfw.glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            mj.glfw.glfw.poll_events()

        mj.glfw.glfw.terminate()


def main():
    xml_path = "./xml/pendulum.xml"
    sim = ContolPendulum(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
