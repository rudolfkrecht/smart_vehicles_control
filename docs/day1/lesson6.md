# Lesson 6 — Individual project: drive the first straight


**Format:** Controller-development task in groups

**Editable file:** `python_3d_adas/student_controller.py`

**Main outcome:** A tested longitudinal controller that drives the car along the
first straight without reaching the first bend during the test

## Project brief

Your task is to make the car start from rest and drive along the first straight
section of the 3D-like highway.

The car must:

- approach a target speed of $15\ \mathrm{m/s}$;
- respond correctly to the uphill and downhill sections;
- remain in its lane;
- use valid throttle and brake commands;
- remain on the first straight during the fixed 17-second experiment.

You will develop only the **longitudinal controller**. The supplied Pure Pursuit
controller already calculates the steering command.

!!! important
    Do not edit the steering controller during this lesson. Keep:

    ```python
    steering = pure_pursuit_steering(observation)
    ```

    Lateral control will be developed on a later day.

## The section of road used in this task

The coordinate `track_s` measures distance along the road centreline. The car
starts at:

$$
s=8\ \mathrm{m}.
$$

The first bend begins at:

$$
s=220\ \mathrm{m}.
$$

The first straight contains five parts:

| Track coordinate | Length used | Road condition |
|---:|---:|---|
| $8 \leq s < 20\ \mathrm{m}$ | 12 m | Flat start |
| $20 \leq s < 80\ \mathrm{m}$ | 60 m | $5^\circ$ uphill |
| $80 \leq s < 140\ \mathrm{m}$ | 60 m | Elevated level section |
| $140 \leq s < 200\ \mathrm{m}$ | 60 m | $5^\circ$ downhill |
| $200 \leq s < 220\ \mathrm{m}$ | 20 m | Flat approach to the bend |

The uphill force acting against the vehicle is:

$$
F_\mathrm{hill}=mg\sin(\theta).
$$

For the $1200\ \mathrm{kg}$ car on the $5^\circ$ incline:

$$
F_\mathrm{hill}
=1200(9.81)\sin(5^\circ)
\approx1026\ \mathrm{N}.
$$

The corresponding road grade is:

$$
100\tan(5^\circ)\approx8.75\%.
$$

The same force acts in the direction of motion on the downhill section.

## Success criteria

Your final 17-second run must satisfy all of the following:

| Requirement | Pass condition |
|---|---:|
| Target speed | $15\ \mathrm{m/s}$ |
| Rise time | Reach $13.5\ \mathrm{m/s}$ within 10 s |
| Overshoot | Less than 10% |
| Final speed error | Absolute value below $0.50\ \mathrm{m/s}$ |
| Progress after 17 s | $200 \leq s < 220\ \mathrm{m}$ |
| Lane error | Maximum absolute value below $0.25\ \mathrm{m}$ |
| Throttle | Always between 0 and 1 |
| Brake | Always between 0 and 1 |
| Command logic | Throttle and brake are never positive together |

The final speed error reported by the simulator is calculated from the mean
speed during the final two seconds. It is not simply the last recorded sample.

A P or PI controller may be accepted if it meets every requirement. If integral
action is used, the controller must also include anti-windup.

## Files you may edit

Edit only:

```text
python_3d_adas/student_controller.py
```

Do not edit:

```text
python_3d_adas/simulator/model.py
python_3d_adas/simulator/track.py
python_3d_adas/simulator/simulation.py
python_3d_adas/simulator/controllers.py
```

Changing the vehicle, road or evaluation code would make the comparison
invalid.


## Start the simulator

### Step 1: Open the project

Open **Windows Terminal** or **Command Prompt** and enter:

```bat
cd %USERPROFILE%\Documents\smart_vehicles_control\python_3d_adas
```

If your repository is stored elsewhere, open its `python_3d_adas` folder
instead.

### Step 2: Start the graphical simulator

Run:

```bat
python run_simulator.py
```

Alternatively, double-click:

```text
run_windows.bat
```

### Step 3: Select the correct experiment

1. Select **Student** from the controller list.
2. Set **Target speed** to approximately $15.0\ \mathrm{m/s}$.
3. Press **Reset**.
4. Observe the minimap and identify the first straight.
5. Pause the simulator before the car continues around the first bend.

Do not evaluate the controller from appearance alone. The graphical run helps
you understand the behaviour; the headless run provides reproducible
measurements.

---

## Establish the baseline

Close the graphical simulator or open a second terminal in the same folder.

Run the unchanged Student controller for exactly 17 seconds:

```bat
python run_simulator.py --headless --controller student --duration 17 --target-speed 15 --csv straight_baseline.csv
```

The terminal reports:

- rise time;
- overshoot;
- final speed error;
- speed RMSE;
- maximum cross-track error.

Read the final road position and final speed from the CSV:

```bat
python -c "import csv; rows=list(csv.DictReader(open('straight_baseline.csv'))); r=rows[-1]; print('track_s =', r['track_s_m'], 'm, speed =', r['speed_mps'], 'm/s')"
```

Record the baseline:

| Quantity | Baseline result | Requirement | Pass? |
|---|---:|---:|:---:|
| Rise time | | $\leq10\ \mathrm{s}$ | |
| Overshoot | | $<10\%$ | |
| Final speed error | | $|e_\mathrm{final}|<0.50\ \mathrm{m/s}$ | |
| Position after 17 s | | $200 \leq s <220\ \mathrm{m}$ | |
| Maximum lane error | | $<0.25\ \mathrm{m}$ | |

### Baseline diagnosis

Answer briefly:

1. Which requirements does the starter controller fail?
2. Is the car too slow, too aggressive or both?
3. Does the largest speed error occur at the start, uphill or downhill?
4. Does the steering controller keep the car close to the lane centre?
5. Which controller parameter should be changed first?

---

## Understand the controller

Open:

```text
student_controller.py
```

The controller receives an `Observation`. The variables needed today are:

| Python variable | Meaning | Unit |
|---|---|---|
| `observation.speed` | Measured speed | m/s |
| `observation.target_speed` | Requested speed | m/s |
| `observation.dt` | Controller update interval | s |
| `observation.track_s` | Position along the road | m |
| `observation.slope_radians` | Current road angle | rad |
| `observation.cross_track_error` | Lane-centre error | m |

The speed error is:

$$
e_v=v_\mathrm{ref}-v.
$$

In the Python controller:

```python
speed_error = observation.target_speed - observation.speed
```

For proportional control:

$$
u_\mathrm{raw}=K_Pe_v.
$$

The signed controller output is limited to:

$$
-1\leq u\leq1.
$$

A positive command becomes throttle and a negative command becomes brake:

```python
if signed_command >= 0.0:
    throttle = signed_command
    brake = 0.0
else:
    throttle = 0.0
    brake = -signed_command
```

### Predict before testing

| Situation | Sign of speed error | Expected command | Physical reason |
|---|:---:|---|---|
| Start from rest | | | |
| Speed below target uphill | | | |
| Speed exactly at target | | | |
| Speed above target downhill | | | |

Check your prediction:

- positive error should request throttle;
- zero error should request no corrective command;
- negative error should request braking.

---

## Tune the P controller

Set:

```python
self.KI = 0.00
```

Change only `self.KP`. Begin with three clearly different candidates, for
example:

```text
0.08, 0.14, 0.20
```

These are search values, not guaranteed final values.

For every candidate:

1. Change `self.KP`.
2. Save `student_controller.py`.
3. Run a 17-second headless experiment.
4. Save each run under a different CSV filename.
5. Record the metrics.
6. Change only one value before the next run.

Example for the first candidate:

```bat
python run_simulator.py --headless --controller student --duration 17 --target-speed 15 --csv p_run_1.csv
```

Use `p_run_2.csv` and `p_run_3.csv` for the next candidates.

### P-controller tuning table

| Run | $K_P$ | Rise time [s] | Overshoot [%] | Final error [m/s] | Final $s$ [m] | Pass? |
|---:|---:|---:|---:|---:|---:|:---:|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |
| 4, if needed | | | | | | |

### Interpret the results

- A very small $K_P$ produces weak acceleration and a large speed error.
- Increasing $K_P$ initially reduces rise time.
- If throttle is already saturated at 1, increasing $K_P$ cannot produce more
  than the maximum available drive force.
- A large $K_P$ may react strongly when the car enters the downhill section.
- Select the smallest gain that satisfies the requirements, unless another
  measured criterion supports a different choice.

Write:

```text
Selected P gain:
Reason for selection:
Requirement that limited the choice:
Expected risk of increasing Kp further:
```

---

## Decide whether integral action is needed

First decide from evidence:

> Does the selected P controller satisfy every requirement?

If yes, you may keep the P controller and explain why the simpler structure is
sufficient for this task.

If the persistent speed error is too large, use PI control:

$$
u_\mathrm{raw}
=K_Pe_v+K_I\int e_v\,dt.
$$

In `student_controller.py`, enable the candidate integral:

```python
candidate_integral = (
    self.integral_error + speed_error * observation.dt
)
```

Try a small integral gain. Useful search values are:

```text
0.003, 0.005, 0.010
```

Keep the selected $K_P$ fixed while changing $K_I$.

### Add conditional-integration anti-windup

If $K_I>0$, do not integrate blindly while the actuator is saturated. Use the
candidate command to decide whether integration is allowed:

```python
candidate_raw = (
    self.KP * speed_error
    + self.KI * candidate_integral
)

if (
    -1.0 <= candidate_raw <= 1.0
    or (candidate_raw > 1.0 and speed_error < 0.0)
    or (candidate_raw < -1.0 and speed_error > 0.0)
):
    self.integral_error = candidate_integral
```

This logic:

- integrates when the raw command is inside its limits;
- pauses integration when error would push the command farther into
  saturation;
- allows the integral to unwind when error acts in the opposite direction.

Test no more than two or three $K_I$ values during this lesson.

| $K_P$ | $K_I$ | Rise time [s] | Overshoot [%] | Final error [m/s] | Final $s$ [m] | Pass? |
|---:|---:|---:|---:|---:|---:|:---:|
| | 0.000 | | | | | |
| | | | | | | |
| | | | | | | |

Do not select PI automatically. Select it only if the measurements show a
useful improvement.

---

## Graphical safety check

Start the graphical simulator:

```bat
python run_simulator.py
```

Then:

1. Select **Student**.
2. Set the target to $15\ \mathrm{m/s}$.
3. Press **Reset** to reload the edited file.
4. Watch the throttle, brake, speed, slope and lane error.
5. Observe the change from flat road to $5^\circ$ uphill.
6. Observe the elevated section and the $5^\circ$ downhill.
7. Pause before the car enters the first bend.

Complete the observation table:

| Road part | Speed behaviour | Throttle behaviour | Brake behaviour | Explanation |
|---|---|---|---|---|
| Flat start | | | | |
| Uphill | | | | |
| Elevated level | | | | |
| Downhill | | | | |

Check:

```text
[ ] the vehicle starts without a Python error
[ ] throttle remains between 0 and 1
[ ] brake remains between 0 and 1
[ ] throttle and brake are not active together
[ ] the car remains close to the lane centre
[ ] the downhill response is stable
```

If your changes do not appear, save the file and press **Reset** again.

---

## Formal validation

Run the final controller:

```bat
python run_simulator.py --headless --controller student --duration 17 --target-speed 15 --csv straight_final.csv
```

Inspect its final position and speed:

```bat
python -c "import csv; rows=list(csv.DictReader(open('straight_final.csv'))); r=rows[-1]; print('track_s =', r['track_s_m'], 'm, speed =', r['speed_mps'], 'm/s')"
```

Complete the final scorecard:

| Requirement | Measured result | Pass? |
|---|---:|:---:|
| Rise time $\leq10\ \mathrm{s}$ | | |
| Overshoot $<10\%$ | | |
| $|e_\mathrm{final}|<0.50\ \mathrm{m/s}$ | | |
| $200 \leq s <220\ \mathrm{m}$ after 17 s | | |
| Maximum lane error $<0.25\ \mathrm{m}$ | | |
| Valid and mutually exclusive commands | | |
| Anti-windup present if $K_I>0$ | | |

A controller passes only if every applicable row passes.

---

## Submit the checkpoint

Save:

```text
student_controller.py
straight_baseline.csv
straight_final.csv
completed results tables
```

Write a four-sentence engineering conclusion:

1. State whether you selected P or PI control.
2. State the selected gain or gains.
3. Compare one baseline measurement with the final result.
4. Identify one limitation that remains.

Example structure:

> I selected a ___ controller with ___. The baseline ___ changed from ___ to
> ___. The final controller did/did not satisfy all first-straight
> requirements. A remaining limitation is ___.

## Troubleshooting

### The car does not move

Check that:

- **Student** is selected;
- `self.KP` is greater than zero;
- `speed_error` is calculated as target minus measured speed;
- the file was saved;
- **Reset** was pressed after the edit.

### The simulator reports a syntax error

Compare brackets and indentation with the original file. Python uses
indentation to define the statements inside the `if` block.

### The edited controller is not reloaded

Save `student_controller.py`, return to the graphical simulator and press
**Reset**. Restarting the complete program is not normally necessary.

### The car continues around the bend

That is normal in the graphical simulator if it is allowed to continue. The
formal experiment ends automatically after 17 seconds and must finish with:

$$
s<220\ \mathrm{m}.
$$

### The final position is below 200 m

The car accelerated too slowly. Examine rise time, throttle saturation and the
selected gain before changing several parameters at once.

### The final position is above 220 m

The controller reached the bend before the test ended. Check for excessive
overshoot or an unnecessarily aggressive controller.

## Fast-finisher extension — stop before the bend

Modify the controller so it tracks $15\ \mathrm{m/s}$ initially and then
reduces its local desired speed to zero near the end of the straight.

One possible decision structure is:

```python
if observation.track_s < BRAKING_START_S:
    desired_speed = observation.target_speed
else:
    desired_speed = 0.0

speed_error = desired_speed - observation.speed
```

Choose and justify `BRAKING_START_S`. The additional objective is:

$$
200\leq s<220\ \mathrm{m},
\qquad
v<0.5\ \mathrm{m/s}.
$$

Do not tune the braking point by changing the road or the brake-force model.

??? note "Reference check"
    With the supplied vehicle and track, the unchanged starter controller
    normally fails the 17-second progress and rise-time requirements.

    A correctly implemented controller can typically finish near
    $s=205$–$216\ \mathrm{m}$ with speed close to $15\ \mathrm{m/s}$ while
    remaining on the straight. Exact values depend on the selected gains.

    The supplied reference controller uses $K_P=0.15$ and $K_I=0.005$. In the
    current simulator it gives approximately:

    - rise time: $9.05\ \mathrm{s}$;
    - overshoot: $3.63\%$;
    - final mean speed error: $-0.43\ \mathrm{m/s}$;
    - position after 17 seconds: $s=209.9\ \mathrm{m}$.

    Your results do not need to match these values exactly.
