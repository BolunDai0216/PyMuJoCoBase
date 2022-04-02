import mujoco as mj
import numpy as np
from mujoco.glfw import glfw

from mujoco_base import MuJoCoBase

try:
    import nlopt
except ImportError:
    print("nlopt not imported, switching to pre-computed solution")
    NLOPT_IMPORTED = False


class ManipulatorIK(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 10.0
        self.data_sim = mj.MjData(self.model)

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        x = np.array([-0.5, 1.0])
        self.x_target = np.array([1.75, 1.5])
        self.inverse_kinematics(x, self.x_target)
        # self.forward_kinematics([-0.5, 1.0])

    def forward_kinematics(self, x):
        self.data_sim.qpos[0] = x[0]
        self.data_sim.qpos[1] = x[1]
        self.data_sim.ctrl[0] = self.data_sim.qpos[0]
        self.data_sim.ctrl[2] = self.data_sim.qpos[1]

        mj.mj_forward(self.model, self.data_sim)

        end_eff_pos = np.array([
            self.data_sim.sensordata[0],
            self.data_sim.sensordata[2]
        ])

        return end_eff_pos

    def cost_func(self, x, grad):
        cost = 0.0

        return cost

    def equality_constraints(self, result, x, grad):
        end_eff_pos = self.forward_kinematics(x)
        result[0] = end_eff_pos[0] - self.x_target[0]
        result[1] = end_eff_pos[1] - self.x_target[1]

    def inverse_kinematics(self, x, x_target):
        # Define optimization problem
        opt = nlopt.opt(nlopt.LN_COBYLA, 2)

        # Define lower and upper bounds
        opt.set_lower_bounds([-np.pi, -np.pi])
        opt.set_upper_bounds([np.pi, np.pi])

        # Set objective funtion
        opt.set_min_objective(self.cost_func)

        # Define equality constraints
        tol = [1e-8, 1e-8]
        opt.add_equality_mconstraint(self.equality_constraints, tol)

        # Set relative tolerance on optimization parameters
        opt.set_xtol_rel(1e-4)

        # Solve problem
        sol = opt.optimize(x)

        return sol

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


def main():
    xml_path = "./xml/manipulator.xml"
    sim = ManipulatorIK(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
