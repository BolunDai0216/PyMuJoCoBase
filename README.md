# PyMuJoCoBase

This repo contains starter code and examples for running simulations in MuJoCo with its Python bindings. This repo is inspired by the C code developed for Prof.Pranav Bhounsule's [MuJoCo Bootcamp](https://pab47.github.io/mujoco.html).

## Contents

The main purpose of this repo is providing the starter code required to run a MuJoCo simulation with keyboard and mouse callbacks using its Python bindings. The base class is in `mujoco_base.py`. To create your own MuJoCo simulation, you can create a new class that inherits `mujoco_base.MuJoCoBase`. An example of this usage is provided in `example_projectile.py`, the new class should implement the functions

```[Python]
- reset()       # Initializes the enviroment and control callback
- controller()  # Adds control actions
- simulate()    # Copy the simulate() function from 
                # mujoco_base.MuJoCoBase and add your own twist
```

## MuJoCo Bootcamp Examples

All of the examples in the [MuJoCo Bootcamp](https://pab47.github.io/mujoco.html) are translated into Python. The examples include:

```[Markdown]
- Projectile with drag
- Control a simple pendulum
- Control a double pendulum
- Leg swing
- Manipulator drawing
- Control an underactuated pendulum
- Gymnast swing/release on a bar
- 2D Hopper
- Initial Value Problem
- Inverse Kinematics
- 2D Biped
```

## NLOPT

I could not get the [Python bindings for NLOPT](https://github.com/DanielBok/nlopt-python) to work on an M1 Mac, its only tested on Ubuntu 20.04 LTS.

## Contact

If you like this repo please consider giving it a star! If you have any questions or suggestions regarding this repo you can contact me at `bd1555 at nyu dot edu`.
