# Algorithm package for MDP

## For running the project

### Step 1: Install the package in development mode

In the root folder containing `setup.py` file, run

```sh
pip install -e .
```

The flag `-e` is short for --editable, and `.`  refers to the current working
directory, so together, it means to install the current directory
in editable mode. This will also install any dependencies declared with install_requires

### Step 2: Run the main file

In the root folder containing `setup.py` file, run

```sh
python -m mdpalgo
```

## Details to note:

The arena:

- The map is a 20 by 20 grid of cells
- Each cell represents 10cm x 10cm

The robot:

- Modeled as a 3x3 on the grid
- Actual robot size is about 20cm x 21cm
- Turning radius:
  - Notes suggested about 25cm turning radius
  - In the simulator, the turning radius used is 3x3 which is 30cm by 30cm

The "obstacle" model:

- Physical size is identical to size on grid (1x1) (10cm x 10cm)
- Obstacle border given for astar path planning is about 20cm (2 cells away)
- Obstacle border given for hardcoded shortest path is about 10cm (1 cell away)
