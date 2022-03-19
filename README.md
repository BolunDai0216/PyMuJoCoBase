# PyMuJoCoBase

This repo contains starter code and examples for running simulations in MuJoCo with its Python bindings. The repo is inspired by the C code used in Prof.Pranav Bhounsule's [MuJoCo Bootcamp](https://pab47.github.io/mujoco.html).

## Contents

The main purpose of the this repo is providing the starter code required to run a MuJoCo simulation with keyboard and mouse callbacks. The base class is in `mujoco_base.py`. To create your own MuJoCo simulation, you can create a new class that inherits `mujoco_base.MuJoCoBase`. An example of this usage is provided in `projectile.py`, the new class should implement the functions

```[Python]
- reset()       # Initializes the enviroment and control callback
- controller()  # Adds control actions
- simulate()    # Copy the simulate() function from 
                # mujoco_base.MuJoCoBase and add your own twist
```

## TODO

I plan to translate all of the examples in the [MuJoCo Bootcamp](https://pab47.github.io/mujoco.html) into Python. Current progress: (2/12).

- [x] Projectile with drag
- [x] Control a simple pendulum
- [ ] Control a double pendulum
- [ ] Leg swing
- [ ] Manipulator drawing
- [ ] Control an underactuated pendulum
- [ ] Gymnast swing/release on a bar
- [ ] 2D Hopper
- [ ] Nonlinear Optimization
- [ ] Initial Value Problem
- [ ] Inverse Kinematics
- [ ] 2D Biped

## Contact

If you have any questions regarding this repo you can contact me at `bd1555 at nyu dot edu`.
