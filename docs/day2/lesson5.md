# Lesson 5 — Guided Pure Pursuit in the 3D-like ADAS simulator

- **Editable file:** `python_3d_adas_day2/student_controller.py`
- **Main outcome:** A working fixed-look-ahead Pure Pursuit controller

You will now transfer the lateral controller from the two-dimensional
laboratory to the same closed-highway simulator used on Day 1.

The supplied code already contains:

- the 3D-like car and chase camera;
- the closed two-lane highway and its inclines;
- the force-balance longitudinal model;
- the completed Day 1 PI speed controller with anti-windup;
- vehicle position, heading and preview-point observations;
- steering actuator limits and lag;
- CSV logging and performance metrics.

Your task is to implement only the Pure Pursuit steering calculation.

## Signals used by the lateral controller

| Observation | Meaning | Unit |
|---|---|---:|
| `x`, `y` | current rear-axle position | m |
| `heading` | current car heading | rad |
| `preview_x`, `preview_y` | selected path point ahead | m |
| `preview_distance` | requested distance ahead | m |
| `cross_track_error` | signed lane-centre error | m |
| `heading_error` | car heading minus path heading | rad |
| `speed` | current forward speed | m/s |

The controller returns normalized `throttle`, normalized `brake` and steering
in radians.

## Start and inspect the reference

From the package root:

```bat
cd python_3d_adas_day2
py -3.12 run_simulator.py
```

In the graphical interface:

1. select **Reference**;
2. set the target speed to 12.0 m/s;
3. press **Reset**;
4. observe the first straight and first right-hand bend;
5. find lane error, heading error, look-ahead and steering on the dashboard;
6. find the cyan preview target on the minimap.

Answer:

1. When does the steering command first become non-zero?
2. Is the preview target on the car or ahead of it?
3. Does steering respond only after the car reaches the bend?
4. Why is preview useful?

## Run the student baseline

Open:

```text
student_controller.py
```

Select **Student** in the simulator and press **Reset**.

The starter returns:

```python
steering = 0.0
```

The car should complete the first straight because the lane is initially
straight. It should then fail to follow the first bend. This is the intended
baseline, not a simulator error.

Save the run as:

```text
results\baseline_no_steering.csv
```

Record:

| Observation | Baseline result |
|---|---|
| First point where lane error grows rapidly | |
| Maximum visible lane error | |
| Off-road warning appears? | |
| Steering command | |

## Implement Pure Pursuit

In `student_controller.py`, replace:

```python
steering = 0.0
```

with the following steps.

### Step 1: vector to the preview target

```python
dx = observation.preview_x - observation.x
dy = observation.preview_y - observation.y
```

### Step 2: global target bearing

```python
target_bearing = math.atan2(dy, dx)
```

### Step 3: target angle relative to the car

```python
alpha = (
    target_bearing - observation.heading + math.pi
) % (2.0 * math.pi) - math.pi
```

The modulo expression wraps the angle to $[-\pi,\pi)$.

### Step 4: actual geometric target distance

```python
geometric_lookahead = max(1.0, math.hypot(dx, dy))
```

The lower limit prevents division by a value close to zero.

### Step 5: Pure Pursuit steering

```python
steering = math.atan2(
    2.0 * self.WHEELBASE_M * math.sin(alpha),
    geometric_lookahead,
)
```

This implements:

$$
\delta=
\tan^{-1}\left(
\frac{2L\sin\alpha}{L_d}
\right).
$$

### Step 6: steering limit

```python
steering = clamp(
    steering,
    -self.MAX_STEERING_RAD,
    self.MAX_STEERING_RAD,
)
```

Save the file. Select **Student** and press **Reset**. The simulator reloads
your code.

!!! warning
    Do not change `KP`, `KI`, throttle or brake during Day 2. Those values are
    the completed Day 1 longitudinal controller.

## Test fixed look-ahead values

The preview point is selected by:

```python
return clamp(
    self.BASE_LOOKAHEAD_M + self.SPEED_GAIN_S * speed,
    3.0,
    24.0,
)
```

Keep:

```python
self.SPEED_GAIN_S = 0.0
```

Test:

```python
self.BASE_LOOKAHEAD_M = 4.0
```

then 6.0 and 10.0 m. Reset after every edit.

Use reproducible 40-second headless runs:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 40 --target-speed 12 --csv results\fixed_6m.csv
```

Change the filename for each configuration.

Record:

| Base $L_d$ | Max $|e_y|$ | Mean $|e_y|$ | Outside road | Steering-rate RMS | Peak $a_y$ | Decision |
|---:|---:|---:|---:|---:|---:|---|
| 4 m | | | | | | |
| 6 m | | | | | | |
| 10 m | | | | | | |

Short look-ahead can produce small geometric error but high steering activity
and lateral acceleration. Long look-ahead is smoother but may cut the bend.

## Graphical safety check

Run the best fixed configuration graphically:

1. watch the complete first bend;
2. check that the car remains within the lane;
3. watch the preview target move ahead;
4. check that steering changes sign for the second bend;
5. save `fixed_lookahead_final.csv`.

If the car leaves the road:

- confirm the angle-wrap expression;
- confirm `atan2(dy, dx)`, not `atan(dy / dx)`;
- confirm all angles passed to `sin`, `atan2` and the simulator are radians;
- confirm the steering command is not accidentally negated;
- increase look-ahead gradually rather than changing several parameters.

## Checkpoint

Complete:

```text
Pure Pursuit implemented: yes / no
Selected fixed look-ahead:
Maximum lane error:
Outside-road percentage:
Steering-rate RMS:
One trade-off:
```

Keep `student_controller.py` and the three CSV files. Lesson 6 starts from this
working fixed-look-ahead controller.

??? success "Complete lateral solution"
    The complete implementation is included in:

    ```text
    python_3d_adas_day2/student_controller_solution.py
    ```

    Its lateral calculation is:

    ```python
    dx = observation.preview_x - observation.x
    dy = observation.preview_y - observation.y
    target_bearing = math.atan2(dy, dx)
    alpha = (
        target_bearing - observation.heading + math.pi
    ) % (2.0 * math.pi) - math.pi
    geometric_lookahead = max(1.0, math.hypot(dx, dy))
    steering = math.atan2(
        2.0 * self.WHEELBASE_M * math.sin(alpha),
        geometric_lookahead,
    )
    steering = clamp(
        steering,
        -self.MAX_STEERING_RAD,
        self.MAX_STEERING_RAD,
    )
    ```
