# Lesson 5 — Guided longitudinal control in the Python 3D simulator

- **Format:** Guided implementation in groups
- **Starting controller:** `student_controller.py`
- **Main outcome:** A tested PI cruise controller with anti-windup

In this lesson, you will transfer the controller from the one-dimensional PyQt
experiment into a vehicle that moves around a closed 3D-like road. The supplied
Pure Pursuit controller handles steering, so you will work only on the
longitudinal controller.

## Learning objectives

By the end of this lesson, you should be able to:

- distinguish measured speed, target speed and speed error;
- convert a signed controller output into mutually exclusive throttle and brake;
- compare P and PI control on flat and inclined road sections;
- explain why the same throttle produces different acceleration on a slope;
- identify saturation and implement conditional-integration anti-windup;
- use quantitative evidence rather than visual judgement alone.

## Start the simulator

From the repository root on Windows, enter:

```bat
cd python_3d_adas
```

Open this file in your editor:

```text
student_controller.py
```

Start the simulator:

```bat
py -3.12 run_simulator.py
```

Select **Student** from the controller list and press **Reset**. The starter is
a working P controller with automatic steering.

## Signals and units

Your controller receives an `Observation`. The most important Day 1 members
are:

| Variable | Meaning | Unit |
|---|---|---|
| `observation.speed` | measured vehicle speed | m/s |
| `observation.target_speed` | cruise reference | m/s |
| `observation.dt` | controller sampling interval | s |
| `observation.slope_radians` | road angle | rad |
| `observation.grade_percent` | road grade | % |
| `observation.acceleration` | longitudinal acceleration | m/s² |
| `observation.cross_track_error` | signed lane-centre error | m |

The controller returns:

| Command | Range | Meaning |
|---|---:|---|
| `throttle` | 0 to 1 | fraction of maximum drive force |
| `brake` | 0 to 1 | fraction of maximum braking force |
| `steering` | approximately −0.49 to +0.49 rad | front steering request |

## Inspect and run the baseline

Run the Student controller without editing it.

1. Set the target to 15 m/s.
2. Press **Reset**.
3. Allow the car to pass through the ascent and elevated section.
4. Observe speed, throttle, acceleration, road slope and lane error.
5. Select **Save CSV** and save the result as `baseline_p.csv`.

Answer before continuing:

1. What is the speed error at the start?
2. Why does the output initially saturate?
3. Why does the P controller not produce exactly 15 m/s on the elevated flat?
4. What new force acts during the 5-degree ascent?
5. Does the supplied lateral controller remain within the lane?

The hill force is:

$$
F_\mathrm{hill}=mg\sin(\theta).
$$

For the default car:

$$
F_\mathrm{hill}
=1200(9.81)\sin(5^\circ)
\approx1026\ \mathrm{N}.
$$

The road grade is not 5%. It is:

$$
\mathrm{grade}=100\tan(5^\circ)\approx8.75\%.
$$

## Tune proportional control

Edit only:

```python
self.KP = 0.08
```

Test at least three values. Save the file and press **Reset** after each change.

| $K_P$ | 90% rise time | Maximum speed | Flat-road final error | Uphill minimum speed | Saturation observed? |
|---:|---:|---:|---:|---:|---|
| | | | | | |
| | | | | | |
| | | | | | |

Explain:

- why a larger gain initially changes response speed;
- why increasing gain has no additional effect while throttle is already 1;
- why the P controller retains an uphill speed error;
- which gain you choose and what evidence supports it.

Do not select a gain solely because the plot looks smooth.

## Add integral action

Uncomment the candidate-integral calculation:

```python
candidate_integral = (
    self.integral_error + speed_error * observation.dt
)
self.integral_error = candidate_integral
```

Set a small positive value:

```python
self.KI = 0.01
```

Increase it gradually. Reset between experiments.

| $K_P$ | $K_I$ | Flat final error | Uphill final error | Overshoot | Comment |
|---:|---:|---:|---:|---:|---|
| | | | | | |
| | | | | | |
| | | | | | |

The units are important. Because the normalized command is dimensionless:

$$
[K_P]=\frac{1}{\mathrm{m/s}}=\mathrm{s/m},
$$

and:

$$
[K_I]
=\frac{1}{(\mathrm{m/s})\mathrm{s}}
=\mathrm{m^{-1}}.
$$

## Prevent integral windup

The current implementation integrates error even when the raw command is above
the actuator limit. Replace the unconditional update with:

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

This is conditional integration:

- integrate when the raw command is within the actuator range;
- stop integrating if it would drive the controller farther into saturation;
- permit integration if it helps move the controller out of saturation.

Compare the same gains with and without this block. Change the target from
15 m/s to 8 m/s after the car reaches its original target.

Record:

| Case | Peak speed after target reduction | Time to settle | Braking begins at | Interpretation |
|---|---:|---:|---:|---|
| No anti-windup | | | | |
| Conditional integration | | | | |

## Evidence-based conclusion

Complete the following summary and prepare to report your main result:

```text
Selected KP:
Selected KI:
Anti-windup method:
Rise time:
Overshoot:
Final flat-road error:
Maximum uphill error:
Maximum lane error:
One trade-off:
One remaining limitation:
```

## Fast-finisher tasks

1. Change vehicle mass in `simulator/model.py` from 1200 kg to 1500 kg. Do the
   same gains still meet the requirements?
2. Derive the throttle required to maintain 15 m/s on the 5-degree climb.
3. Run at 20 m/s. Explain why quadratic drag becomes more important.
4. Export the CSV and calculate RMSE in a separate Python script.
