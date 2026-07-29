# Day 1 — Longitudinal vehicle control

**Central engineering question:** How can software make a vehicle reach and
maintain a requested speed when the vehicle, road and actuator are not ideal?

Day 1 develops one complete control-engineering workflow. We begin with physical
forces and measurable requirements, compare open-loop and feedback behaviour,
calculate controller responses by hand, test them in graphical simulators, and
finish with an individual first-straight controller project against explicit
acceptance criteria.

The day contains six lessons of 45 minutes. Each lesson has a core activity,
a concrete output and an optional extension. Work through the core task first;
use the extension if you finish early.

## Day plan

| Lesson | Format | Main activity | Required output |
|---:|---|---|---|
| 1 | Theory overview | Connect the autonomous-driving stack, vehicle physics, feedback, P/PI control and performance metrics | Five-question concept check |
| 2 | Guided Python demonstrations | Compare open loop, feedback, drag, P/PI control and windup | Completed evidence table |
| 3 | Numerical exercises | Solve four quantitative control problems and check the worked solutions | Calculation sheet |
| 4 | Guided PyQt laboratory | Calculate, predict, run and measure four controller experiments | Results sheet and engineering conclusion |
| 5 | Guided 3D-like simulation | Implement and test P/PI cruise control in the Python highway simulator | Tested controller and results table |
| 6 | Individual simulator project | Drive the 3D-like car along the first straight using a tested longitudinal controller | Controller, baseline and final CSV files, and completed scorecard |

The detailed lesson pages provide a minute-by-minute route:

1. [Theory overview](lesson1.md)
2. [Four practical demonstrations](lesson2.md)
3. [Four guided engineering exercises](lesson3.md)
4. [Guided exercises with the PyQt simulator](lesson4.md)
5. [Guided longitudinal control in the Python 3D simulator](lesson5.md)
6. [Individual project: drive the first straight](lesson6.md)

## What you should be able to do by the end

By the end of Day 1, you should be able to:

- distinguish open-loop control from closed-loop feedback;
- identify the reference, measured output, error, controller, actuator, plant
  and disturbance in a speed-control loop;
- explain the simplified longitudinal force balance;
- calculate equilibrium speed and required actuator command;
- implement and evaluate proportional and PI control;
- calculate rise time, overshoot, settling time and final error;
- explain why actuator saturation limits the benefit of higher gain;
- explain persistent error, integral action and integral windup;
- transfer a longitudinal controller from a simple model to a 3D-like highway
  simulator;
- evaluate controller behaviour on flat, uphill and downhill road sections;
- justify a controller using evidence rather than visual preference.

## Before Day 1

Complete the [software setup](../setup.md), then open Command Prompt or a
terminal in the repository root.

```bash
python -m pip install -r requirements.txt
python setup_check.py
```

The setup check should finish without an exception. The Day 1 files are
organized as follows:

| Directory | Purpose |
|---|---|
| `day_1_longitudinal/demos` | Complete Python demonstrations |
| `day_1_longitudinal/student` | Editable controller investigations |
| `day_1_longitudinal/solutions` | Reference solutions |
| `simulator` | Shared vehicle model, controllers, plotting and metrics |
| `python_3d_adas` | 3D-like highway simulator and editable student controller |
| `docs/assets/images/day1` | Prepared figures used on the course website |

Run one headless demonstration before the first lesson:

```bash
python day_1_longitudinal/demos/lesson1_feedback_preview.py --no-show
```

If the command prints a metrics table, the numerical environment is ready.
Matplotlib windows can then be tested by running the same command without
`--no-show`.

!!! note "Commands on Windows"
    The commands on these pages work in ordinary Windows Command Prompt when
    executed from the repository root. Anaconda and PowerShell are not
    required.

## Day 1 numerical model

The model is deliberately compact enough to inspect and calculate by hand:

$$
m a =
F_\mathrm{actuator}
- F_\mathrm{rolling}
- c_\mathrm{drag}v^2
- F_\mathrm{hill}.
$$

The default parameters are:

| Quantity | Symbol | Default value |
|---|---:|---:|
| Vehicle mass | $m$ | $1200\ \mathrm{kg}$ |
| Maximum drive force | $F_{\max}$ | $4500\ \mathrm{N}$ |
| Maximum brake force | $F_{\mathrm{brake,max}}$ | $7000\ \mathrm{N}$ |
| Rolling resistance | $F_\mathrm{rolling}$ | $180\ \mathrm{N}$ |
| Aerodynamic-drag coefficient | $c_\mathrm{drag}$ | $4\ \mathrm{N/(m/s)^2}$ |
| Actuator time constant | $\tau$ | $0.35\ \mathrm{s}$ |
| Simulation step | $\Delta t$ | $0.05\ \mathrm{s}$ |

These values are not intended to reproduce one production vehicle. They create
a transparent teaching model containing inertia, resistance, quadratic drag,
unequal drive/brake authority, actuator lag and command saturation.

## Working method used throughout the day

Every practical activity follows the same engineering cycle:

1. **State a prediction.** Write what should change and why.
2. **Run a controlled comparison.** Change one factor at a time.
3. **Measure.** Record numerical metrics, not only plot appearance.
4. **Explain.** Connect the observation to a force, equation or controller term.
5. **Challenge the conclusion.** Test another road, gain, mass or disturbance.

Do not treat the Python files as black boxes, but you are not expected to write
a graphical simulator from scratch. Editable parameters are grouped near the
top of each starter file.

## Evidence pack for the final presentation

Create a results folder such as:

```text
day1_results/
├── straight_baseline.csv
├── straight_final.csv
├── student_controller.py
├── day1_notes.md
└── day1_selected_plot.png
```

At the end of Day 1, the evidence pack should contain:

- your baseline configuration and result;
- the controller configurations you tested;
- one table of quantitative results;
- one legible plot;
- the selected controller parameters;
- a short explanation of one trade-off;
- one limitation or unanswered question.

## Suggested team roles

During paired or group work, rotate these roles:

| Role | Responsibility |
|---|---|
| Driver | Runs the command and edits the marked parameter block |
| Recorder | Writes predictions, configurations and numerical results |
| Analyst | Checks calculations and connects results to theory |
| Reporter | Summarizes the evidence during the class discussion |

In Lesson 6, everyone edits and validates an individual controller, even if you
discuss ideas with your group.

## Core work, extensions and solutions

Each lesson page distinguishes between:

- **core work**, which everyone should complete;
- **engineering challenge**, which raises the difficulty without introducing an
  unrelated topic;
- **reserve activity**, which you can use if you finish early.

The additional tasks deepen the same learning objective through derivations,
sensitivity analysis, uncertainty bounds and disturbance tests.

!!! tip "Using the worked solutions"
    Worked solutions are intentionally included in the numerical and guided
    exercises. Attempt each task first, record your reasoning, and only then
    expand the solution to check your method and units.
