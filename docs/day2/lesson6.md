# Lesson 6 — Individual project: complete a full lap


- **Editable file:** `python_3d_adas_day2/student_controller.py`
- **Main outcome:** A validated lateral controller that completes one closed
  highway lap

## Project brief

Continue from your working Lesson 5 controller. Your task is to tune the
look-ahead rule so the car follows the target lane for one complete lap at
12 m/s.

The lap includes:

- the first straight with a $5^\circ$ climb, plateau and descent;
- the first constant-radius bend;
- the opposite straight;
- the second bend;
- return to the starting region.

The Day 1 PI controller remains fixed. You are responsible for lateral control.

## Acceptance criteria

Use a 75-second headless run at a target of 12 m/s.

| Requirement | Pass condition |
|---|---:|
| Lap progress | at least 100% |
| Maximum $|e_y|$ | below 0.75 m |
| Mean $|e_y|$ | below 0.25 m |
| Outside-road samples | 0% |
| Steering-rate RMS | below $4^\circ$/s |
| Peak lateral acceleration | below 3.5 m/s² |
| Speed rise time | below 11 s |
| Speed overshoot | below 10% |
| Look-ahead | always between 3 and 24 m |
| Commands | throttle and brake never positive together |

Note: These requirements are simulator acceptance criteria, not a claim of
real-vehicle safety.

## Prepare the experiment

From the package root:

```bat
cd python_3d_adas_day2
```

Create a results folder if it does not already exist:

```bat
mkdir results
```

Confirm that your Lesson 5 Pure Pursuit calculation is present in:

```text
student_controller.py
```

Do not alter:

```python
self.KP = 0.15
self.KI = 0.005
```

## Establish a fixed-look-ahead baseline

Set:

```python
self.BASE_LOOKAHEAD_M = 6.0
self.SPEED_GAIN_S = 0.0
```

Run:

```bat
python run_simulator.py --headless --controller student --duration 75 --target-speed 12 --csv results\baseline_fixed.csv
```

Copy the printed metrics:

| Metric | Fixed baseline |
|---|---:|
| Lap progress | |
| Maximum $|e_y|$ | |
| Mean $|e_y|$ | |
| Outside road | |
| Steering-rate RMS | |
| Peak $a_y$ | |

Mark each acceptance criterion pass or fail. Do not start tuning without a
baseline.

## Introduce speed-dependent look-ahead

Your existing method is:

```python
def preview_distance(self, speed: float) -> float:
    return clamp(
        self.BASE_LOOKAHEAD_M + self.SPEED_GAIN_S * speed,
        3.0,
        24.0,
    )
```

This implements:

$$
L_d=L_{d,0}+K_vv.
$$

Choose one initial adaptive candidate. A sensible search region is:

| Parameter | Suggested range |
|---|---:|
| $L_{d,0}$ | 3–6 m |
| $K_v$ | 0.15–0.45 s |

Before running, calculate its look-ahead at 6, 12 and 16 m/s:

| Speed | Calculated $L_d$ |
|---:|---:|
| 6 m/s | |
| 12 m/s | |
| 16 m/s | |

Explain why $K_v$ has units of seconds.

## Tune systematically

Test at least three configurations. Change one parameter at a time.

Use:

```bat
python run_simulator.py --headless --controller student --duration 75 --target-speed 12 --csv results\candidate_1.csv
```

Use a new filename for each run.

| $L_{d,0}$ | $K_v$ | $L_d$ at 12 m/s | Max $|e_y|$ | Steering-rate RMS | Peak $a_y$ | Lap progress | Decision |
|---:|---:|---:|---:|---:|---:|---:|---|
| | | | | | | | |
| | | | | | | | |
| | | | | | | | |

Selection order:

1. reject road departure;
2. reject incomplete laps;
3. reject excessive lateral acceleration;
4. reject excessive steering activity;
5. compare tracking accuracy among the remaining candidates.

Do not optimize only mean error.

## Challenge your selected controller

Keep the same parameters and perform two shorter comparison runs:

```bat
python run_simulator.py --headless --controller student --duration 45 --target-speed 8 --csv results\robustness_8mps.csv
```

```bat
python run_simulator.py --headless --controller student --duration 45 --target-speed 16 --csv results\robustness_16mps.csv
```

These are sensitivity tests; they do not use the full-lap acceptance criteria.

Record:

| Speed | Look-ahead | Max $|e_y|$ | Steering-rate RMS | Peak $a_y$ | Interpretation |
|---:|---:|---:|---:|---:|---|
| 8 m/s | | | | | |
| 16 m/s | | | | | |

At high speed, a lateral controller may still track the geometric path while
the required lateral acceleration becomes too large. This is a meaningful
limitation and prepares the curve-aware speed-control work of the next stage.

## Final graphical and CSV run

Start:

```bat
python run_simulator.py
```

Select **Student**, set 12 m/s, reset and watch one full lap.

Check:

- the preview target remains ahead of the car;
- steering changes smoothly through both bends;
- the off-road warning never appears;
- the car returns to the starting region;
- the controller remains stable after the downhill section.

Save:

```text
results\final_full_lap.csv
```

Repeat the final headless command so the submitted metrics and CSV come from the
same parameter set:

```bat
python run_simulator.py --headless --controller student --duration 75 --target-speed 12 --csv results\final_full_lap.csv
```

## Submit the checkpoint

Complete the scorecard:

| Requirement | Result | Pass/fail |
|---|---:|---|
| Lap progress $\geq100\%$ | | |
| Max $|e_y|<0.75$ m | | |
| Mean $|e_y|<0.25$ m | | |
| Outside road $=0\%$ | | |
| Steering-rate RMS $<4^\circ$/s | | |
| Peak $a_y<3.5$ m/s² | | |
| Rise time $<11$ s | | |
| Overshoot $<10\%$ | | |

Submit:

1. `student_controller.py`;
2. `baseline_fixed.csv`;
3. `final_full_lap.csv`;
4. completed configuration table;
5. completed scorecard;
6. the following conclusion:

```text
I selected Ld0 = ___ m and Kv = ___ s.
Compared with the fixed baseline, this changed ___.
The most important supporting metrics were ___ and ___.
The main trade-off was ___.
The kinematic simulator does not represent ___, so I cannot yet conclude ___.
```

## Fast-finisher extension

Compare:

- fixed $L_d=6$ m;
- adaptive $L_d=4+0.35v$;
- long fixed $L_d=10$ m.

Create a small Pareto argument: identify configurations for which no other
candidate is simultaneously more accurate and smoother. Explain why controller
selection remains a design decision even when every metric is known.
