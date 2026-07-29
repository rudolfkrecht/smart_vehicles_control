# Python 3D-like ADAS simulator — Day 3

This standard-library simulator continues the cumulative vehicle-control
project:

- **Day 1:** PI throttle/brake control;
- **Day 2:** speed-dependent Pure Pursuit steering;
- **Day 3:** lead-vehicle sensing and Adaptive Cruise Control.

The blue ego vehicle follows a closed two-lane highway containing long
straights, two bends, a 5° climb, a plateau and a 5° descent. A red lead
vehicle follows a deterministic stop-and-go schedule.

## Start on Windows

Double-click:

```text
run_windows.bat
```

or open Command Prompt in this directory:

```bat
py -3.12 run_simulator.py
```

No `pip install` is required for the 3D-like simulator.

## Controller choices

- **Reference:** complete cumulative solution;
- **Student:** reloads `student_controller.py` after every Reset;
- **Manual:** arrow-key throttle, brake and steering.

The Student starter contains the completed Day 1 and Day 2 controllers but
initially ignores traffic. Implement the Day 3 target-selection block before
tuning it.

## Headless experiments

Cruise-only baseline:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 70 --target-speed 14 --csv results\baseline.csv
```

Reference benchmark:

```bat
py -3.12 run_simulator.py --headless --controller reference --duration 105 --target-speed 14 --csv results\reference.csv
```

Final student test:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 105 --target-speed 14 --csv results\final.csv
```

## Main observations

The controller receives:

- ego speed and pose;
- driver cruise setting;
- lead detection;
- measured gap and lead speed;
- closing speed and TTC;
- lane preview point and tracking errors.

The CSV includes:

- actual and selected speeds;
- throttle, brake and steering;
- lead speed;
- actual and desired gaps;
- closing speed and TTC;
- behaviour mode;
- collision flag;
- lane errors and lap progress.

## Project files

| File or directory | Purpose |
|---|---|
| `student_controller.py` | Student-editable cumulative controller |
| `student_controller_solution.py` | Worked solution |
| `simulator/` | Dynamics, track, traffic, control, renderer and GUI |
| `tests/` | Automated regression tests |
| `LESSON5_GUIDED.md` | Guided implementation instructions |
| `LESSON6_PROJECT.md` | Individual project instructions |

## Validation

```bat
py -3.12 -m unittest discover -s tests -v
```

The reference 105-second scenario completes more than one lap without
collision or road departure.

## Model boundary

This is an educational controller-development simulator. It does not reproduce
production radar, tire saturation, cut-ins, weather, hardware delay or
automotive safety certification.
