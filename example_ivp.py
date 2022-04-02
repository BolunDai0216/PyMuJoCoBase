import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv

from mujoco_base import MuJoCoBase
from pdb import set_trace
from scipy import optimize


class InitialValueProblem(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 5.0

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        # Set initial guess
        v = 10.0
        theta = np.pi / 4
        time_of_flight = 2.0

        set_trace()
        sol = optimize.root(self.opt_func, x0=[v, theta, time_of_flight], method="broyden1")

        pos = self.simulator(v, theta, time_of_flight)
        print(f"Position at the end: ({pos[0]}, {pos[1]})")

        self.data.qvel[0] = v * np.cos(theta)
        self.data.qvel[2] = v * np.sin(theta)
    
    def simulator(self, v, theta, time_of_flight):
        self.data.qvel[0] = v * np.cos(theta)
        self.data.qvel[2] = v * np.sin(theta)

        while (self.data.time < time_of_flight):
            # Step simulation environment
            mj.mj_step(self.model, self.data)
        
        # Get position
        pos = np.array([self.data.qpos[0], self.data.qpos[2]])

        # Reset Data
        mj.mj_resetData(self.model, self.data)

        return pos
    
    def opt_func(self, v, theta, time_of_flight):
        pos = self.simulator(v, theta, time_of_flight)
        # err = (pos[0] - 5.0)**2 + (pos[1] - 2.1)**2
        err = pos - np.array([5.0, 2.1])

        return err

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                # Step simulation environment
                mj.mj_step(self.model, self.data)

            if self.data.time >= self.simend:
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            self.cam.lookat[0] = self.data.qpos[0]
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()
    
    def set_position_servo(self, actuator_no, kp):
        self.model.actuator_gainprm[actuator_no, 0] = kp
        self.model.actuator_biasprm[actuator_no, 1] = -kp
    
    def set_velocity_servo(self, actuator_no, kv):
        self.model.actuator_gainprm[actuator_no, 0] = kv
        self.model.actuator_biasprm[actuator_no, 2] = -kv



def main():
    xml_path = "./xml/projectile_opt.xml"
    sim = InitialValueProblem(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
