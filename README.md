# Project 2: Bug Navigation - Python (F23)

Autonomous navigation for the MBot-Omni: drive to a goal pose while avoiding
obstacles with a Bug 2 algorithm. See the [project instructions](https://hellorob.org/projects/p2).

**Project webpage (P2.5):** https://jacobmonroe2.github.io/Bug-Navigation/
(source in [`docs/index.html`](docs/index.html); enable GitHub Pages from the
`docs/` folder in the repository settings to publish it).

## Files

| File | Task | Description |
|---|---|---|
| [`hit_the_spot.py`](hit_the_spot.py) | P2.1 | `drive_to_pose()` and a demo that visits a sequence of goals, printing the pose at each |
| [`bug_navigation.py`](bug_navigation.py) | P2.3, P2.4 | Bug 2 navigation state machine; prompts for a goal, prints the state every iteration, stops if the goal is unreachable |
| [`utils.py`](utils.py) | | Shared helpers: `wrap_angle`, `find_min_ray_in_slice`, `StallBooster`, etc. |
| [`DESIGN.md`](DESIGN.md) | P2.2 | Algorithm description with pseudocode and state machine diagram |
| [`docs/index.html`](docs/index.html) | P2.5 | Project webpage |
| [`sim/`](sim/) | | Offline 2D simulator used to test the code without the robot |

## Running on the robot

Run on the robot's Raspberry Pi with the MBot Bridge running.

Hit the spot (P2.1). With no arguments it drives a square; or pass goals as
`x y theta` triples:

```bash
python3 hit_the_spot.py
```

```bash
python3 hit_the_spot.py 1 0 0  1 1 1.57  0 0 0
```

Bug navigation (P2.4). It prompts for the goal `x`, `y` and `theta`, or they
can be given on the command line:

```bash
python3 bug_navigation.py
```

```bash
python3 bug_navigation.py 2.5 0 0
```

Both scripts reset the odometry at start, print the robot's state/pose every
iteration, stop the robot on exit (including Ctrl+C) and print the final odometry.

## Testing without the robot

`sim/run_sim.py` substitutes a small 2D simulator for `mbot_bridge.api.MBot`
and runs the real navigation code. It has an ideal robot and a realistic one
modelled on the real MBot: wheels that stall at low speed, noisy Lidar and
communication delay.

Run every scenario with both robot models:

```bash
python3 sim/run_sim.py all
```

Watch a single run with the full state printout. Add `--real` for the realistic robot:

```bash
python3 sim/run_sim.py bug small_room 1.3 0 0 --real
```

Test worlds: `empty`, `box`, `wall`, `two_boxes`, `corridor`, `u_trap`,
`small_room`, `narrow_gap`, `blocked_door`, `cluttered`. ASCII maps of each run
are written to `sim/out/`.
