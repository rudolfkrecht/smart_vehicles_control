# Python 3D-like ADAS simulator — Day 4

This standard-library simulator completes the cumulative project:

- **Day 1:** PI throttle/brake control;
- **Day 2:** speed-dependent Pure Pursuit steering;
- **Day 3:** lead-vehicle sensing and Adaptive Cruise Control;
- **Day 4:** sensor-health supervision, fault scenarios and batch evaluation.

The blue ego vehicle follows a closed two-lane highway with bends, a 5° climb,
a plateau and a 5° descent. The red lead vehicle follows a deterministic
stop-and-go schedule.

## Start on Windows

Double-click:

```text
run_windows.bat
```

or open Command Prompt in this directory:

```bat
py -3.12 run_simulator.py
```

No `pip install` is required for this simulator.

## Controllers

- **Reference:** complete cumulative solution;
- **Student:** reloads `student_controller.py` after every Reset;
- **Manual:** arrow-key throttle, brake and steering.

The Student starter already contains PI, Pure Pursuit and ACC. Implement the
marked Day 4 supervisor TODO.

## Scenarios

Choose a scenario in the GUI or with `--scenario`:

```text
nominal
radar_dropout
lateral_push
brake_fade
combined
```

Example:

```bat
py -3.12 run_simulator.py --headless --controller student --scenario radar_dropout --duration 50 --target-speed 14
```

## Final evaluation

Run one unchanged controller over all five scenarios:

```bat
py -3.12 evaluate_project.py --controller student --duration 105 --target-speed 14 --csv results\day4_results.csv
```

Use the worked solution for comparison:

```bat
py -3.12 evaluate_project.py --controller solution --duration 105 --target-speed 14
```

## Main controller observations

In addition to the Day 1–3 signals, the controller receives:

- `range_sensor_healthy`;
- `range_measurement_age`;
- `active_fault`.

The CSV also records:

- scenario name;
- requested and applied braking;
- braking efficiency;
- supervisor activation;
- safe-stop mode;
- lateral-push application;
- true collision and lane evidence.

## Project files

| File or directory | Purpose |
|---|---|
| `student_controller.py` | Student-editable cumulative controller |
| `student_controller_solution.py` | Worked Day 4 solution |
| `evaluate_project.py` | Five-scenario evaluation and CSV export |
| `simulator/faults.py` | Fixed scenario definitions |
| `simulator/` | Dynamics, track, traffic, renderer and GUI |
| `tests/` | Automated regression tests |
| `LESSON5_GUIDED.md` | Guided implementation |
| `LESSON6_PROJECT.md` | Individual final project |

## Validation

```bat
py -3.12 -m unittest discover -s tests -v
```

## Model boundary

This is an educational controller-development simulator. It does not establish
production sensor diagnostic coverage, tire limits, functional-safety
integrity, legal compliance or real-vehicle safety.
