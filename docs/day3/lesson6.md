# Lesson 6 — Individual project: stop-and-go traffic

- **Format:** Group controller-development task
- **Editable file:** `python_3d_adas_day3\student_controller.py`
- **Main outcome:** A validated cumulative controller that follows the road and
  lead vehicle for more than one lap

## Project brief

Continue from the working ACC implementation created in Lesson 5. Tune the
traffic-spacing parameters so the blue ego car:

- follows the target lane;
- respects the 14 m/s cruise setting when traffic permits;
- responds to the red lead vehicle;
- stops without collision;
- restarts smoothly;
- completes at least one lap.

The lead-vehicle schedule is fixed and repeatable:

1. 10.5 m/s cruise;
2. gradual slowing to 6 m/s;
3. 6 m/s travel;
4. braking to a complete stop;
5. an 8-second stop;
6. acceleration to 11 m/s;
7. a later second slowdown.

Do not change the Day 1 PI or Day 2 Pure Pursuit parameters.

## Acceptance criteria

Use a 105-second headless run:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 105 --target-speed 14
```

| Requirement | Pass condition |
|---|---:|
| Collision samples | 0 |
| Minimum bumper gap | at least 6 m |
| Minimum finite TTC | at least 4 s |
| Emergency samples | 0 |
| Mean absolute gap error | below 9.5 m |
| Selected-target speed RMSE | below 2.2 m/s |
| Peak deceleration | below 2.5 m/s² |
| Lap progress | at least 100% |
| Maximum lane error | below 0.75 m |
| Outside-road samples | 0% |
| Commands | throttle and brake never positive together |

These are course acceptance criteria for one deterministic model. They are not
real-vehicle safety requirements.

## Prepare the experiment

From the package root:

```bat
cd python_3d_adas_day3
mkdir results
```

Confirm that your Lesson 5 implementation is present in:

```text
student_controller.py
```

Keep:

```python
self.KP = 0.15
self.KI = 0.005
self.BASE_LOOKAHEAD_M = 4.0
self.SPEED_GAIN_S = 0.35
```

You may tune only:

```python
self.STANDSTILL_GAP_M
self.TIME_HEADWAY_S
self.GAP_GAIN_PER_S
self.CLOSING_GAIN
```

## Establish the Lesson 5 baseline

Use:

```python
self.STANDSTILL_GAP_M = 6.0
self.TIME_HEADWAY_S = 1.5
self.GAP_GAIN_PER_S = 0.20
self.CLOSING_GAIN = 0.50
```

Run:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 105 --target-speed 14 --csv results\baseline_acc.csv
```

Copy the printed values:

| Metric | Baseline |
|---|---:|
| Minimum gap | |
| Minimum finite TTC | |
| Collision samples | |
| Mean absolute gap error | |
| Speed-target RMSE | |
| Peak deceleration | |
| Lap progress | |
| Maximum lane error | |

Mark each acceptance criterion pass or fail before tuning.

## Tune spacing policy

Choose at least two new spacing candidates:

| Parameter | Suggested range |
|---|---:|
| Standstill gap $d_0$ | 4–8 m |
| Time headway $T_h$ | 1.0–2.2 s |

Calculate the desired gap at 6 and 14 m/s before running:

| Candidate | $d_0$ | $T_h$ | $d_{\mathrm{des}}$ at 6 m/s | $d_{\mathrm{des}}$ at 14 m/s |
|---|---:|---:|---:|---:|
| 1 | | | | |
| 2 | | | | |

Change only $d_0$ and $T_h$. Keep the two feedback gains at their baseline
values.

Run each candidate with a unique filename:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 105 --target-speed 14 --csv results\spacing_1.csv
```

| $d_0$ | $T_h$ | Min gap | Min TTC | Gap error | Completion | Decision |
|---:|---:|---:|---:|---:|---:|---|
| | | | | | | |
| | | | | | | |

## Tune response gains

Keep your selected spacing policy. Compare at least three gain pairs:

| Parameter | Suggested range | Effect |
|---|---:|---|
| $K_d$ | 0.12–0.30 s⁻¹ | response to gap error |
| $K_{\Delta v}$ | 0.30–0.80 | response to positive closing speed |

Use:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 105 --target-speed 14 --csv results\gain_1.csv
```

| $K_d$ | $K_{\Delta v}$ | Min gap | Speed RMSE | Peak decel. | Gap error | Emergency | Decision |
|---:|---:|---:|---:|---:|---:|---:|---|
| | | | | | | | |
| | | | | | | | |
| | | | | | | | |

Change one gain at a time whenever possible.

Interpretation:

- larger $K_d$ corrects spacing error more strongly but can produce a more
  active target;
- larger $K_{\Delta v}$ responds earlier to closing but may reduce speed more
  than necessary;
- small gains may appear smooth while allowing an unsafe gap to develop.

## Select using a hierarchy

Reject candidates in this order:

1. any collision;
2. minimum gap below 6 m;
3. minimum TTC below 4 s;
4. Emergency activation in the reference scenario;
5. road departure or incomplete lap;
6. excessive deceleration;
7. excessive gap or speed-target error.

Among the remaining candidates, select one balanced configuration.

Do not tune only for:

- maximum progress;
- smallest gap error;
- largest minimum gap;
- lowest deceleration.

Each single objective can produce a poor result elsewhere.

## Final graphical test

Run:

```bat
py -3.12 run_simulator.py
```

Select **Student**, set 14 m/s and press **Reset**.

Watch the complete stop-and-go event. Check:

- Cruise changes to Follow when the lead enters sensor range;
- the selected target falls before the gap becomes critical;
- Brake does not chatter rapidly;
- Emergency does not activate in the final solution;
- the ego car stops with visible clearance;
- it restarts after the lead vehicle;
- Pure Pursuit still follows both bends;
- the car remains stable on the inclines.

If graphical behaviour disagrees with the headless result, verify that you
pressed **Reset** after saving `student_controller.py`.

## Produce the final evidence

Repeat the exact final headless run:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 105 --target-speed 14 --csv results\final_acc_run.csv
```

Complete:

| Requirement | Result | Pass/fail |
|---|---:|---|
| Collision samples $=0$ | | |
| Minimum gap $\geq6$ m | | |
| Minimum TTC $\geq4$ s | | |
| Emergency samples $=0$ | | |
| Mean gap error $<9.5$ m | | |
| Speed RMSE $<2.2$ m/s | | |
| Peak deceleration $<2.5$ m/s² | | |
| Lap progress $\geq100\%$ | | |
| Max lane error $<0.75$ m | | |
| Outside road $=0\%$ | | |

## Submit the checkpoint

Submit:

1. `student_controller.py`;
2. `baseline_acc.csv`;
3. at least three candidate CSV files;
4. `final_acc_run.csv`;
5. completed parameter table;
6. completed scorecard;
7. the following conclusion:

```text
I selected d0 = ___ m and Th = ___ s.
I selected Kd = ___ 1/s and Kclosing = ___.
Compared with the baseline, minimum gap changed from ___ to ___.
The most important supporting metrics were ___ and ___.
The main safety–efficiency trade-off was ___.
The simulator omits ___, so I cannot conclude ___ about a real vehicle.
```

??? success "Reference solution and expected benchmark"
    A valid reference configuration is:

    ```python
    self.STANDSTILL_GAP_M = 6.0
    self.TIME_HEADWAY_S = 1.7
    self.GAP_GAIN_PER_S = 0.22
    self.CLOSING_GAIN = 0.65
    ```

    The packaged 105-second reference run produces approximately:

    | Metric | Reference |
    |---|---:|
    | Minimum gap | 7.52 m |
    | Minimum finite TTC | 9.16 s |
    | Collision samples | 0 |
    | Mean absolute gap error | 8.33 m |
    | Speed-target RMSE | 2.02 m/s |
    | Peak deceleration | 1.37 m/s² |
    | Lap progress | 108.5% |
    | Maximum lane error | 0.092 m |
    | Outside road | 0% |
    | Emergency samples | 0 |

    The complete worked controller is:

    ```text
    student_controller_solution.py
    ```

    Small numerical differences are acceptable. Reproducible evidence and a
    sound explanation matter more than copying every reference digit.

## Fast-finisher robustness test

Keep the final controller and compare:

```bat
py -3.12 run_simulator.py --headless --controller student --duration 75 --target-speed 12 --csv results\robustness_12mps.csv
```

```bat
py -3.12 run_simulator.py --headless --controller student --duration 75 --target-speed 18 --csv results\robustness_18mps.csv
```

Explain why passing the 14 m/s reference scenario does not guarantee the same
margin at every cruise setting.
