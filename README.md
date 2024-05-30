# PyMuJoCoBase

This repo contains starter code and examples for running simulations in MuJoCo with its Python bindings. This repo is inspired by the C code developed for Prof.Pranav Bhounsule's [MuJoCo Bootcamp](https://pab47.github.io/mujoco.html).

## Installation

First, install the Python bindings for MuJoCo (newly tested on v3.1.5)

```console
python3 -m pip install mujoco
```

Then, run the following commands to install `PyMuJoCoBase`

```console
git clone https://github.com/BolunDai0216/PyMuJoCoBase.git
cd PyMuJoCoBase
python3 -m pip install .
```

## Contents

The main purpose of this repo is providing the starter code required to run a MuJoCo simulation with keyboard and mouse callbacks using its Python bindings. The base class is in `mujoco_base.py`. To create your own MuJoCo simulation, you can create a new class that inherits `mujoco_base.MuJoCoBase`. An example of this usage is provided in `example_projectile.py`, the new class should implement the functions

```[Python]
- reset()       # Initializes the enviroment and control callback
- controller()  # Adds control actions
- simulate()    # Copy the simulate() function from 
                # mujoco_base.MuJoCoBase and add your own twist
```

## How to apply control

The sole purpose of the `controller()` function is to change the values of the `data.ctrl` which corresponds to the actuator control values. There are two ways to achieve this, one is to set `controller()` as the [control callback function](https://mujoco.readthedocs.io/en/latest/APIreference.html?highlight=%20control#mjcb-control)

```python
mj.set_mjcb_control(self.controller)
```

in `reset()`, an example usage is shown in `example_manipulator_drawing.py`. This tells MuJoCo to run the controller automatically. Another way to is to run `controller()` directly in `simulate()`, i.e., add

```python
while (self.data.time - simstart < 1.0/60.0):
    self.controller(self.model, self.data)
    mj.mj_step(self.model, self.data)
```

to `simulate()`, please refer to `example_manipulator_ik.py` for this usage. This is a more manual approach, which applies the new control action at each time step.

## MuJoCo Bootcamp Examples

All of the examples in the [MuJoCo Bootcamp](https://pab47.github.io/mujoco.html) are translated into Python. The examples include:

```[Markdown]
- Projectile with drag (pymjbase-projectile-example)
- Control a simple pendulum (pymjbase-pendulum-example)
- Control a double pendulum (pymjbase-dbpendulum-example)
- Leg swing (pymjbase-leg-swing-example)
- Manipulator drawing (pymjbase-manipulator-drawing-example)
- Control an underactuated pendulum (pymjbase-underactuated-pendulum-example)
- Gymnast swing/release on a bar (pymjbase-gymnast-example)
- 2D Hopper (pymjbase-hopper-example)
- Initial Value Problem (pymjbase-ivp-example)
- Inverse Kinematics (pymjbase-manipulator-ik-example)
- 2D Biped (pymjbase-biped-example)
```

Run the command in the parenthesis to see the example, for example, to run the inverse kinematics example we can use the command:

```console
pymjbase-manipulator-ik-example
```

## NLOPT

I could not get the [Python bindings for NLOPT](https://github.com/DanielBok/nlopt-python) to work on an M1 Mac, its only tested on Ubuntu 20.04 LTS.

## Contact

If you like this repo please consider giving it a star! If you have any questions or suggestions regarding this repo you can contact me at `bd1555 at nyu dot edu`.
