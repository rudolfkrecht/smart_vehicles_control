# Lesson 6 — Individual final project: multi-scenario ADAS evidence

- **Format:** Controller validation in groups
- **Editable file:** `python_3d_adas_day4\student_controller.py`
- **Main outcome:** One frozen cumulative controller that passes a documented
  five-scenario suite

## Project brief

Continue from the sensor-health supervisor implemented in Lesson 5. Validate
one unchanged controller over:

1. nominal stop-and-go traffic;
2. radar dropout;
3. a lateral disturbance;
4. reduced braking authority;
5. a combined challenge.

The objective is not to maximize one number. Your controller must preserve the
safety gates, remain on the road, make useful progress and provide clear
evidence about when supervision was active.

Do not rewrite or retune the Day 1 PI, Day 2 Pure Pursuit or Day 3 ACC layers.
The Day 4 project evaluates integration and supervision.

## Acceptance criteria

The supplied suite applies these criteria to every 105-second scenario:

| Requirement | Pass condition |
|---|---:|
| Collision samples | 0 |
| Road departure | 0% |
| Minimum true gap | greater than 3.0 m |
| Maximum lane error | at most 1.60 m |
| Lap progress | at least 95% |
| Radar-fault scenarios | at least one `SAFE_STOP` sample |
| Commands | throttle and brake never positive together |

The larger $1.60$ m lateral-error limit allows the deliberate 1.35 m state
displacement, but the vehicle must recover without leaving the road.

These are course criteria for this simulator. They are not production ADAS
requirements.

## Prepare and freeze the inherited controller

From the package root:

```bat
cd python_3d_adas_day4
mkdir results
```

Confirm that `student_controller.py` contains:

```python
supervisor_active = (
    not observation.range_sensor_healthy
    or observation.range_measurement_age > self.MAX_RANGE_AGE_S
)
```

and:

```python
elif mode == "SAFE_STOP":
    signed_command = min(
        signed_command,
        -self.SAFE_STOP_MIN_BRAKE,
    )
```

Record your values:

| Parameter | Value |
|---|---:|
| `MAX_RANGE_AGE_S` | |
| `SAFE_STOP_MIN_BRAKE` | |

Save a copy of the starting controller:

```bat
copy student_controller.py results\student_controller_start.py
```

## Confirm the nominal case

Run:

```bat
python run_simulator.py --headless --controller student --scenario nominal --duration 105 --target-speed 14 --csv results\nominal.csv
```

Record:

| Metric | Nominal result | Pass/fail |
|---|---:|---|
| Collision samples | | |
| Minimum gap | | |
| Maximum lane error | | |
| Outside road | | |
| Lap progress | | |
| Safe-stop samples | | |

In the nominal case, `Safe stop samples` should be zero. A supervisor that
activates without a sensor fault reduces availability and probably contains an
incorrect health condition.

## Confirm the radar-dropout case

Run:

```bat
python run_simulator.py --headless --controller student --scenario radar_dropout --duration 105 --target-speed 14 --csv results\radar_dropout.csv
```

Compare with nominal:

| Metric | Nominal | Radar dropout | Interpretation |
|---|---:|---:|---|
| Minimum gap | | | |
| Collision samples | | | |
| Safe-stop samples | | | |
| Peak deceleration | | | |
| Peak jerk | | | |
| Progress | | | |

Safety is the first decision. Comfort and progress describe the cost of the
fallback.

## Test actuator and lateral robustness

Run:

```bat
python run_simulator.py --headless --controller student --scenario brake_fade --duration 105 --target-speed 14 --csv results\brake_fade.csv
```

```bat
python run_simulator.py --headless --controller student --scenario lateral_push --duration 105 --target-speed 14 --csv results\lateral_push.csv
```

Record:

| Scenario | Min gap | Max lane error | Road departure | Progress | Pass/fail |
|---|---:|---:|---:|---:|---|
| Brake fade | | | | | |
| Lateral push | | | | | |

Interpret:

- brake fade should mainly affect longitudinal safety and deceleration;
- the push should mainly affect the maximum lateral error;
- `SAFE_STOP` is not expected in either case because the range sensor is
  healthy;
- successful recovery is provided by the existing Day 2 controller, not by the
  new Day 4 range supervisor.

## Run the complete evaluation suite

The evaluator runs all five cases using a new instance of the same controller:

```bat
python evaluate_project.py --controller student --duration 105 --target-speed 14 --csv results\day4_results.csv
```

Copy the terminal table:

| Scenario | Result | Minimum gap | Maximum lane error | Progress | Failure reason |
|---|---|---:|---:|---:|---|
| Nominal | | | | | |
| Radar dropout | | | | | |
| Lateral push | | | | | |
| Brake fade | | | | | |
| Combined | | | | | |

The target is:

```text
Suite result: 5/5 scenarios passed
```

Do not edit `simulator\faults.py`, `evaluate_project.py`, scenario timing or
acceptance thresholds. They are part of the fixed experiment.

## Diagnose before changing anything

If a scenario fails, use this order:

1. collision;
2. road departure;
3. minimum gap;
4. missing safe-stop activation;
5. lane recovery;
6. progress.

Complete before editing:

| Failed scenario | Failed metric | Probable cause | Proposed code change | Possible side effect |
|---|---|---|---|---|
| | | | | |

Examples:

- collision during radar dropout → health override absent or after PI;
- no safe-stop samples → `supervisor_active` never becomes true;
- nominal safe stop → health and target-detection conditions confused;
- lateral push fails → inherited steering code was accidentally changed;
- brake-fade failure → inherited ACC margin was changed;
- throttle and brake together → final signed-command conversion is wrong.

Only change the supervisor code if the diagnosis supports it. Rerun the full
suite after any change.

## Graphical combined-scenario check

Run:

```bat
python run_simulator.py
```

Choose:

- controller: **Student**;
- scenario: `combined`;
- cruise speed: 14.0 m/s.

Watch:

1. lateral recovery after the push;
2. radar status becoming failed;
3. transition to `SAFE_STOP`;
4. requested braking under reduced authority;
5. resumption after radar recovery;
6. continued lane tracking on bends and inclines.

Visual inspection supports diagnosis, but the headless suite remains the
reproducible result.

## Produce the final evidence

Freeze the controller:

```bat
copy student_controller.py results\student_controller_final.py
```

Run the final suite once:

```bat
python evaluate_project.py --controller student --duration 105 --target-speed 14 --csv results\day4_results_final.csv
```

Do not edit after this run. Record:

```text
Scenarios passed: ___ / 5
Worst minimum gap: ___ m in scenario __________
Worst maximum lane error: ___ m in scenario __________
Radar safe-stop activation observed: yes / no
Collision samples across suite: ___
Road-departure scenarios: ___
```

## Submit and conclude

Submit:

1. `student_controller_final.py`;
2. `day4_results_final.csv`;
3. `radar_dropout.csv`;
4. `lateral_push.csv`;
5. the completed scenario table;
6. this conclusion:

```text
The unmodified Day 3 controller failed __________ because __________.
My Day 4 supervisor activates when __________.
Across the final suite, ___/5 scenarios passed.
The smallest true gap was ___ m in __________.
The largest lane error was ___ m in __________.
The safety benefit of the fallback was __________.
Its main performance or comfort cost was __________.
This evidence applies only to __________ and does not prove __________.
```

??? success "Worked solution and expected benchmark"
    The complete solution is:

    ```text
    student_controller_solution.py
    ```

    Run:

    ```bat
    python evaluate_project.py --controller solution --duration 105 --target-speed 14
    ```

    Expected approximate results:

    | Scenario | Min gap | Max lane error | Progress | Result |
    |---|---:|---:|---:|---|
    | nominal | 7.52 m | 0.09 m | 108.5% | PASS |
    | radar dropout | 19.35 m | 0.09 m | 108.5% | PASS |
    | lateral push | 7.52 m | 1.36 m | 108.5% | PASS |
    | brake fade | 7.44 m | 0.09 m | 108.5% | PASS |
    | combined | 19.47 m | 1.10 m | 108.5% | PASS |

    The solution records 540 safe-stop samples in each radar-dropout case and
    zero collisions in all five scenarios.

## Fast-finisher extension

Compare the `radar_dropout` case at target speeds 12 and 18 m/s:

```bat
python run_simulator.py --headless --controller student --scenario radar_dropout --duration 105 --target-speed 12 --csv results\dropout_12mps.csv
```

```bat
python run_simulator.py --headless --controller student --scenario radar_dropout --duration 105 --target-speed 18 --csv results\dropout_18mps.csv
```

Explain why passing at 14 m/s does not prove an identical margin throughout an
unlimited operating domain.
