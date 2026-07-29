# Lesson 5 — Guided ACC implementation in the 3D-like simulator

- **Format:** Guided implementation in groups
- **Editable file:** `python_3d_adas_day3\student_controller.py`
- **Main outcome:** A working ACC target-selection layer

You will now add traffic awareness to the same cumulative simulator used on
Days 1 and 2.

The supplied code already contains:

- the blue 3D-like ego car and chase camera;
- the closed highway, bends and 5° inclines;
- the Day 1 PI speed controller with anti-windup;
- the Day 2 speed-dependent Pure Pursuit controller;
- a red lead vehicle with a deterministic stop-and-go schedule;
- simulated gap and lead-speed sensing;
- CSV logging and repeatable headless evaluation.

Your task is to calculate the desired gap and select a traffic-aware target
speed. Do not retune the Day 1 or Day 2 parameters.

## Signals available to your controller

| Observation | Meaning | Unit |
|---|---|---:|
| `target_speed` | driver cruise setting | m/s |
| `speed` | ego speed | m/s |
| `lead_detected` | relevant lead target available | Boolean |
| `lead_distance` | bumper-to-bumper gap | m |
| `lead_speed` | measured lead speed | m/s |
| `closing_speed` | ego speed minus lead speed | m/s |
| `time_to_collision` | TTC when closing, otherwise infinity | s |
| `dt` | control interval | s |

The controller returns throttle, brake, steering and three dashboard values:

- `selected_target_speed`;
- `desired_gap`;
- `mode`.

## Observe the reference controller

From the package root:

```bat
cd python_3d_adas_day3
py -3.12 run_simulator.py
```

In the graphical interface:

1. select **Reference**;
2. set the cruise speed to 14.0 m/s;
3. press **Reset**;
4. identify the blue ego and red lead vehicle;
5. observe the yellow sensor line on the minimap when detection begins;
6. compare cruise and selected target speeds;
7. watch the actual and desired gap traces.

Answer:

1. Why does the selected target become lower than the cruise setting?
2. Why can the selected target temporarily become lower than lead speed?
3. Which controller still calculates steering?
4. What happens when the lead vehicle restarts?

## Establish the unsafe cruise-only baseline

Open:

```text
student_controller.py
```

The starter currently contains:

```python
selected_target = observation.target_speed
mode = "CRUISE"
```

It ignores the lead vehicle. Run:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 70 --target-speed 14 --csv results\cruise_only_baseline.csv
```

Create `results` first if needed:

```bat
mkdir results
```

Record:

| Metric | Cruise-only baseline |
|---|---:|
| Minimum gap | |
| Minimum finite TTC | |
| Collision samples | |
| Maximum lane error | |
| Lap progress | |

The baseline is expected to collide. That result proves why traffic-aware
target selection is needed.

## Calculate desired gap

In `update()`, the starter already contains:

```python
desired_gap = (
    self.STANDSTILL_GAP_M
    + self.TIME_HEADWAY_S * observation.speed
)
```

With the supplied settings:

```python
self.STANDSTILL_GAP_M = 6.0
self.TIME_HEADWAY_S = 1.5
```

calculate the desired gap at:

| Ego speed | Desired gap |
|---:|---:|
| 0 m/s | |
| 6 m/s | |
| 10 m/s | |
| 14 m/s | |

??? success "Calculation check"
    | Ego speed | Desired gap |
    |---:|---:|
    | 0 m/s | 6 m |
    | 6 m/s | 15 m |
    | 10 m/s | 21 m |
    | 14 m/s | 27 m |

## Implement the continuous ACC target

Keep the initial default:

```python
selected_target = observation.target_speed
mode = "CRUISE"
```

Directly after it, add:

```python
if observation.lead_detected:
    gap_error = observation.lead_distance - desired_gap

    traffic_target = (
        observation.lead_speed
        + self.GAP_GAIN_PER_S * gap_error
        - self.CLOSING_GAIN
        * max(0.0, observation.closing_speed)
    )

    selected_target = clamp(
        traffic_target,
        0.0,
        observation.target_speed,
    )
    mode = "FOLLOW"
```

Interpret the code:

1. start from lead speed;
2. increase the target if the actual gap is larger than desired;
3. reduce the target if the ego car is closing;
4. keep the result between zero and the cruise setting.

Check the gain units:

```python
self.GAP_GAIN_PER_S = 0.20
self.CLOSING_GAIN = 0.50
```

- `GAP_GAIN_PER_S` has units $\mathrm{s^{-1}}$;
- `CLOSING_GAIN` is dimensionless.

Save the file. In the graphical simulator, select **Student** and press
**Reset**. Reset reloads your Python code.

## Add visible Brake and Emergency logic

Inside the `if observation.lead_detected:` block, after the continuous target,
add:

```python
if (
    observation.lead_distance < 0.72 * desired_gap
    or observation.time_to_collision < 2.5
    or selected_target < observation.speed - 1.0
):
    mode = "BRAKE"

if (
    observation.lead_distance <= 3.0
    or observation.time_to_collision <= 1.2
):
    selected_target = 0.0
    mode = "EMERGENCY"
```

After the PI command is calculated, find:

```python
signed_command = clamp(...)
```

Add:

```python
if mode == "BRAKE":
    signed_command = min(signed_command, -0.22)
elif mode == "EMERGENCY":
    signed_command = min(signed_command, -0.90)
```

These caps request a minimum braking action in the two safety modes.

!!! important
    Emergency braking is a modelled response, not a guarantee that the
    collision can be avoided. A late threshold may activate correctly but
    still leave insufficient stopping distance.

## Run a repeatable guided test

Run:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 70 --target-speed 14 --csv results\guided_acc.csv
```

Record:

| Requirement | Result | Pass/fail |
|---|---:|---|
| Collision samples $=0$ | | |
| Minimum gap $>6$ m | | |
| Minimum TTC $>4$ s | | |
| Maximum lane error $<0.75$ m | | |
| Outside road $=0\%$ | | |
| Emergency samples $=0$ | | |

If the car still collides:

- verify that the PI error uses `selected_target`, not
  `observation.target_speed`;
- verify the sign of `gap_error`;
- verify that closing speed is subtracted;
- verify `max(0.0, observation.closing_speed)`;
- verify target clipping to zero and the cruise setting;
- verify the Brake/Emergency code is inside the detection block.

## Graphical safety check

Start the graphical simulator and select **Student**.

Watch:

1. the first sensor detection;
2. the lead vehicle slowing to 6 m/s;
3. the lead vehicle stopping;
4. the ego vehicle stopping without contact;
5. both vehicles restarting;
6. lane tracking through the bend.

Confirm:

- throttle and brake are never positive together;
- the selected target changes before the ego gap becomes critical;
- the blue car does not pass through the red car;
- lateral control remains stable.

## Checkpoint

Complete:

```text
ACC implemented: yes / no
Standstill gap:
Time headway:
Minimum measured gap:
Minimum finite TTC:
Collision samples:
One reason closing speed is useful:
```

??? success "Complete worked controller"
    The complete implementation is included in:

    ```text
    python_3d_adas_day3\student_controller_solution.py
    ```

    Attempt the exercise before copying it.

## Fast-finisher extension

Set:

```python
self.CLOSING_GAIN = 0.0
```

Rerun the 70-second test. Compare minimum gap and braking response. Explain why
gap feedback alone reacts differently from gap plus relative-speed feedback.
