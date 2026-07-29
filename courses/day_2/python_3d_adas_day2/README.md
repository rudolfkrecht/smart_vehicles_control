# Python 3D-like ADAS simulator — Day 2

This is the Day 2 continuation of the simulator used for Day 1. It retains the
same car, highway, inclines, longitudinal physics and PI speed controller, then
opens the lateral controller for student development.

## Start on Windows

Double-click:

```text
run_windows.bat
```

or run:

```bat
py -3.12 run_simulator.py
```

No pip packages are required for this simulator. It uses Tkinter from the
standard Python installation.

## Controller modes

| Mode | Purpose |
|---|---|
| Reference | completed PI plus adaptive Pure Pursuit |
| Student | reloads `student_controller.py` after Reset |
| Manual | arrow-key driving for exploration |

The starter student controller already contains the completed Day 1 PI
controller. Its steering command is initially zero. Lesson 5 guides you through
implementing Pure Pursuit.

## Headless evaluation

```bat
py -3.12 run_simulator.py --headless --controller student --duration 75 --target-speed 12 --csv results\final.csv
```

The command prints speed, lateral error, road departure, steering activity,
lateral acceleration and lap-progress metrics.

## Files

| File/directory | Purpose |
|---|---|
| `student_controller.py` | student-editable Day 2 controller |
| `student_controller_solution.py` | complete worked solution |
| `simulator/controllers.py` | reference controller and Pure Pursuit helper |
| `simulator/model.py` | force-balance and bicycle vehicle model |
| `simulator/track.py` | closed stadium highway and elevation |
| `simulator/simulation.py` | observations, logging and metrics |
| `simulator/gui.py` | Tkinter interface |
| `tests` | automated simulator tests |

Read `LESSON5_GUIDED.md`, then `LESSON6_PROJECT.md`.

## Verification

```bat
py -3.12 -m unittest discover -s tests -v
py -3.12 run_simulator.py --headless --controller reference --duration 75 --target-speed 12
```
