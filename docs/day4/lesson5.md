# Lesson 5 — Guided safety-supervisor implementation

- **Format:** Guided implementation in groups
- **Editable file:** `python_3d_adas_day4\student_controller.py`
- **Main outcome:** A working range-sensor fallback using `SAFE_STOP`

You will extend the same cumulative simulator used on Days 1–3.

Already complete:

- Day 1 PI throttle/brake control and anti-windup;
- Day 2 speed-dependent Pure Pursuit steering;
- Day 3 ACC desired gap, traffic target and behaviour modes;
- the blue ego vehicle, red lead vehicle and closed highway;
- 5° ascent and descent;
- CSV logging, headless evaluation and graphical replay.

New on Day 4:

- named repeatable fault scenarios;
- a radar health flag and measurement age;
- reduced applied braking;
- a one-time lateral disturbance;
- a combined scenario;
- a supervisory-state field in the command.

Your task is narrow: keep the Day 1–3 code unchanged and add the safe response
for invalid range information.

## Available Day 4 signals

| Observation | Meaning | Unit/type |
|---|---|---|
| `range_sensor_healthy` | range diagnostic reports valid operation | Boolean |
| `range_measurement_age` | age of unavailable/stale information | s |
| `lead_detected` | a valid relevant lead target is available | Boolean |
| `lead_distance` | measured bumper gap when valid | m |
| `time_to_collision` | TTC when closing, otherwise infinity | s |
| `active_fault` | scenario annotation for the dashboard and CSV | string |

The command includes:

| Command field | Purpose |
|---|---|
| `selected_target_speed` | target used by the PI controller |
| `mode` | `CRUISE`, `FOLLOW`, `BRAKE`, `EMERGENCY` or `SAFE_STOP` |
| `supervisor_active` | records whether Day 4 overrode normal operation |

## Observe the complete reference

From the package root:

```bat
cd python_3d_adas_day4
python run_simulator.py
```

In the GUI:

1. choose **Reference**;
2. select scenario `radar_dropout`;
3. set cruise speed to 14.0 m/s;
4. press **Reset**;
5. observe normal ACC operation;
6. watch the range sensor change from `HEALTHY` to `FAILED`;
7. identify the `SAFE_STOP` banner and braking action;
8. observe recovery after the sensor becomes healthy.

The dropout is active from 34 to 43 s. The red lead vehicle still exists and
continues its schedule even though the controller cannot measure it.

Answer:

1. Why does `lead_detected` become false?
2. Why must the controller not interpret this as an empty road?
3. Why does Pure Pursuit remain active in this specific scenario?

## Establish the unsafe Day 3 baseline

The student starter contains the completed Day 3 controller but leaves:

```python
supervisor_active = False
```

Create a results folder:

```bat
mkdir results
```

Run:

```bat
python run_simulator.py --headless --controller student --scenario radar_dropout --duration 50 --target-speed 14 --csv results\radar_dropout_baseline.csv
```

Record:

| Metric | Unsupervised Day 3 baseline |
|---|---:|
| Minimum true gap | |
| Minimum finite TTC | |
| Collision samples | |
| Safe-stop samples | |
| Lap progress | |

The starter is expected to collide. During dropout it sees no lead target,
selects the cruise setting and accelerates toward an unobserved traffic
hazard.

!!! note "Negative gap"
    A negative simulated gap means the ego vehicle has passed through the lead
    vehicle geometry after collision. The simulator records collision samples
    instead of implementing damage dynamics.

## Trace the existing control flow

Open:

```text
student_controller.py
```

Find these sections in order:

1. ACC desired gap and traffic target;
2. Brake/Emergency mode selection;
3. the Day 4 TODO;
4. PI speed error and anti-windup;
5. minimum Brake/Emergency command;
6. Pure Pursuit steering;
7. `ControlCommand(...)`.

The important data path is:

```text
ACC selected target
        ↓
Day 4 health check and optional override
        ↓
PI speed error
        ↓
throttle or brake
```

The health check must occur before PI calculates `speed_error`. Otherwise the
PI would still use the unsafe cruise target for that sample.

## Implement the sensor-health override

Find:

```python
# Day 4 TODO:
```

Replace:

```python
supervisor_active = False
```

with:

```python
supervisor_active = (
    not observation.range_sensor_healthy
    or observation.range_measurement_age > self.MAX_RANGE_AGE_S
)

if supervisor_active:
    selected_target = 0.0
    mode = "SAFE_STOP"
```

Interpret each part:

- `not range_sensor_healthy` reacts to a reported diagnostic failure;
- the age check protects against stale data;
- a zero selected target asks the existing PI layer to decelerate;
- the explicit mode makes the fallback visible and measurable.

Do not use:

```python
if not observation.lead_detected:
```

That condition would stop the vehicle whenever a healthy sensor correctly
reported an empty road.

## Add minimum fallback braking

After:

```python
elif mode == "EMERGENCY":
    signed_command = min(signed_command, -0.90)
```

add:

```python
elif mode == "SAFE_STOP":
    signed_command = min(
        signed_command,
        -self.SAFE_STOP_MIN_BRAKE,
    )
```

The supplied setting is:

```python
self.SAFE_STOP_MIN_BRAKE = 0.45
```

Because a signed negative command is converted to brake, `min()` ensures the
command is at least as negative as $-0.45$.

Finally, confirm that the returned command contains:

```python
supervisor_active=supervisor_active,
```

## Run the repeatable guided test

Run:

```bat
python run_simulator.py --headless --controller student --scenario radar_dropout --duration 50 --target-speed 14 --csv results\radar_dropout_supervised.csv
```

Record:

| Requirement | Result | Pass/fail |
|---|---:|---|
| Collision samples $=0$ | | |
| Minimum true gap $>3$ m | | |
| Safe-stop samples $>0$ | | |
| Fault samples $>0$ | | |
| Outside road $=0\%$ | | |
| Maximum lane error $<0.75$ m | | |

The packaged reference produces approximately:

| Metric | 50 s radar-dropout reference |
|---|---:|
| Minimum gap | 27.74 m |
| Minimum finite TTC | 8.76 s |
| Collision samples | 0 |
| Safe-stop samples | 540 |
| Maximum lane error | 0.092 m |
| Road departure | 0% |
| Progress | 52.8% |

Small numerical differences are acceptable.

## Diagnose common errors

If collision remains:

- confirm the override is before `speed_error`;
- confirm the condition uses `not observation.range_sensor_healthy`;
- confirm `selected_target = 0.0`;
- confirm the PI uses `selected_target`;
- confirm the `SAFE_STOP` brake condition is part of the same `if/elif` chain;
- press **Reset** in the GUI after saving.

If the car stops on an empty nominal road:

- check that you did not use `not observation.lead_detected`;
- run scenario `nominal`;
- confirm the range sensor is healthy even before the lead is detected.

If throttle and brake appear together:

- confirm the final conversion uses one branch for positive signed command and
  one branch for negative signed command.

## Graphical checkpoint

Start the GUI, choose **Student** and scenario `radar_dropout`, then reset.

Complete:

```text
Health condition implemented: yes / no
Maximum accepted range age:
SAFE_STOP minimum brake:
Collision samples before:
Collision samples after:
Minimum gap after:
Why lead_detected alone is insufficient:
```

??? success "Complete worked controller"
    The complete implementation is:

    ```text
    student_controller_solution.py
    ```

    Use it only after you have diagnosed your own result.

## Fast-finisher extension

Compare `SAFE_STOP_MIN_BRAKE` values 0.30 and 0.60 in the same dropout case.
Record minimum gap, peak deceleration and peak jerk. Explain why a stronger
minimum command is not automatically better in every metric.
