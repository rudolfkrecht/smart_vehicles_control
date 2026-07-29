# Day 4 — Robust ADAS integration and evidence

**Central engineering question:** How can you show that an integrated ADAS
controller remains acceptably safe when the experiment is no longer nominal?

During the first three days you built one cumulative controller:

- Day 1: PI longitudinal speed control with anti-windup;
- Day 2: speed-dependent Pure Pursuit lane tracking;
- Day 3: Adaptive Cruise Control using gap, lead speed, closing speed and TTC.

Day 4 keeps all three layers. You will add a supervisory safety layer, expose
the vehicle to repeatable sensor, actuator and lateral disturbances, evaluate
the result with quantitative metrics, and produce a final multi-scenario
evidence package.

![Day 4 cumulative 3D-like simulator](../images/day4_3d_robustness_preview.png)

## Day plan

| Lesson | Format | Main activity | Required output |
|---:|---|---|---|
| 1 | Theory | Robustness, faults, safety supervision and test evidence | Concept check |
| 2 | Guided Python demonstrations | Compare nominal and disturbed runs | Evidence table |
| 3 | Numerical exercises | Calculate errors, TTC, braking and pass/fail metrics | Calculation sheet |
| 4 | Guided PyQt laboratory | Test noise, delay, actuator loss and a lateral push | Robustness table |
| 5 | Guided 3D-like simulation | Implement a range-sensor health supervisor | Working `SAFE_STOP` logic |
| 6 | Individual final project | Evaluate one controller over five scenarios | Controller, suite CSV and conclusion |

Detailed instructions:

1. [Theory: robustness and safety supervision](lesson1.md)
2. [Hands-on Python demonstrations](lesson2.md)
3. [Numerical engineering exercises](lesson3.md)
4. [Guided PyQt robustness exercises](lesson4.md)
5. [Guided 3D supervisor implementation](lesson5.md)
6. [Individual multi-scenario project](lesson6.md)

## What you should be able to do by the end

You should be able to:

- distinguish a nominal demonstration from a robustness test;
- distinguish sensor noise, bias, delay, dropout and actuator-authority loss;
- explain why the controller measurement and the simulator truth must be
  evaluated separately;
- calculate mean, worst-case, safety, comfort and completion metrics;
- define a documented and repeatable test case;
- explain the purpose of a safety supervisor;
- implement a conservative response to invalid range-sensor information;
- preserve the Day 1–3 control layers while adding a higher-level override;
- run one controller over several fixed scenarios without retuning each case;
- report both successful evidence and remaining limitations.

## Before Day 4

Open Command Prompt in the extracted Day 4 package:

```bat
py -3.12 -m pip install -r requirements.txt
py -3.12 setup_check.py
```

The final line should be:

```text
Day 4 environment is ready.
```

The package is organized as follows:

| Directory | Purpose |
|---|---|
| `day_4\demos` | Prepared demonstrations for Lesson 2 |
| `day_4\student` | Editable metric and batch-test exercises |
| `day_4\solutions` | Worked Python solutions |
| `day_4\gui` | Guided PyQt robustness laboratory |
| `simulator` | Numerical robustness, faults and evaluation modules |
| `python_3d_adas_day4` | Cumulative simulator for Lessons 5–6 |

!!! note "Windows commands"
    The commands use ordinary Windows Command Prompt. Run them from the
    extracted package root unless an instruction first changes directory.

## Cumulative controller architecture

Day 4 adds supervision around the controller developed earlier:

```text
driver cruise setting ─┐
lead-vehicle sensing ──┼─> Day 3 ACC target ─> Day 1 PI ─> throttle/brake
road constraints ──────┘

reference lane ───────────> Day 2 Pure Pursuit ───────────> steering

sensor-health status ─────> Day 4 supervisor ─────────────> safe override
```

The supervisor does not replace PI, Pure Pursuit or ACC. It checks whether
their required information is trustworthy. If the forward range sensor becomes
invalid, continuing to use the cruise target would be unsafe because a stopped
lead vehicle may be present but invisible. The Day 4 fallback therefore:

1. detects unhealthy or stale range information;
2. selects a zero-speed target;
3. enters `SAFE_STOP`;
4. requests at least a defined braking action;
5. permits the normal ACC layer to resume after the measurement becomes valid.

## Repeatable 3D-like scenarios

| Scenario | Change from nominal |
|---|---|
| `nominal` | No injected fault |
| `radar_dropout` | Range data unavailable from 34 to 43 s |
| `lateral_push` | One 1.35 m lateral displacement at 26 s |
| `brake_fade` | Only 58% of requested braking authority is applied |
| `combined` | Radar dropout, 65% braking authority and a 1.10 m push |

The lead vehicle follows the same deterministic stop-and-go schedule in every
case. This lets you compare controller changes under identical conditions.

## Core evidence

For cross-track-error samples $e_{y,k}$:

$$
\operatorname{MAE}_{e_y}
=\frac{1}{N}\sum_{k=1}^{N}|e_{y,k}|,
\qquad
e_{y,\max}=\max_k|e_{y,k}|.
$$

For speed errors $e_{v,k}$:

$$
\operatorname{RMSE}_v
=\sqrt{\frac{1}{N}\sum_{k=1}^{N}e_{v,k}^{2}}.
$$

When the ego vehicle closes on the lead vehicle:

$$
\mathrm{TTC}=\frac{d}{v-v_L}.
$$

These averages and extrema must be interpreted together with:

- collision count;
- road-departure percentage;
- minimum true gap;
- scenario completion;
- supervisor activation;
- peak deceleration and jerk.

## Evidence pack

Create:

```text
day4_results/
├── python_demo_table.md
├── numerical_exercises.md
├── pyqt_robustness_table.md
├── radar_dropout_baseline.csv
├── radar_dropout_supervised.csv
├── day4_results.csv
├── student_controller.py
└── day4_conclusion.md
```

Your final conclusion should answer:

1. Which fault made the unmodified Day 3 controller unsafe?
2. What condition activates `SAFE_STOP`?
3. Which metrics show that the supervisor worked?
4. What performance or comfort trade-off did the fallback introduce?
5. Which unmodelled effect prevents a real-vehicle safety claim?

## Safety boundary

This package is an educational deterministic simulation. The fault injections
are deliberately clear and repeatable; they are not a complete automotive
safety analysis. The simulator does not establish:

- production radar diagnostic coverage;
- functional-safety integrity;
- reliable object association or false-positive handling;
- tire-force limits on low-friction roads;
- real braking-system response;
- processor, network or power-supply failures;
- legal compliance or type approval.

Passing the Day 4 suite means that one controller passed five defined software
tests. It does not certify an ADAS product.
