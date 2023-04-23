import time

import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("./xml/dp_leg_high.xml")
data = mujoco.MjData(model)

# remove gravity
model.opt.gravity[2] = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 seconds.
    start = time.time()

    # set camera
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -45.0
    viewer.cam.distance = 10.0
    viewer.cam.lookat = np.array([0.0, 0.0, 0.0])

    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # controller
        data.ctrl = (
            10.0 * (np.array([np.pi / 3, np.pi / 6]) - data.qpos) - 10.0 * data.qvel
        )

        # The mj_step call can be replaced with a user-defined function that evaluates
        # a policy, applies a control signal, and steps an environment.
        mujoco.mj_step(model, data)

        # Example of modifying a viewer option: toggle contact points every second.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        # Synchronize so that the viewer picks up changes to the physics state.
        viewer.sync()

        # Rudimentary time keeping, doesn't attempt to catch up if physics stepping
        # takes too long.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
