# PyMuJoCoBase

This repo contains starter code and examples for running simulations in MuJoCo with its Python bindings. The repo is inspired by the C code used in Prof.Pranav Bhounsule's [MuJoCo Mini Course](https://pab47.github.io/mujoco.html).

## Contents

The main purpose of the this repo is providing the starter code required to run a MuJoCo simulation with keyboard and mouse callbacks. The base class is in `mujoco_base.py`. To create your own MuJoCo simulation, you can create a new class that inherits `mujoco_base.MuJoCoBase`. An example of this usage is provided in `projectile.py`, the new class should implement the functions

```[Python]
- reset()  # initializes the enviroment
- controller()  # adds control actions
- simulate()  # copy the simulate() function from mujoco_base.MuJoCoBase and add your own twist
```

## TODO

I plan to translate all of the examples in the [MuJoCo Mini Course](https://pab47.github.io/mujoco.html) in to Python. Current progress: (1/12).

## Contact

If you have any questions regarding this repo you can contact me at `bd1555 at nyu dot edu`.
