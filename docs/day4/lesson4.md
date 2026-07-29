# Lesson 4 — Guided PyQt robustness exercises


- **Format:** Guided laboratory
- **Tool:** `day_4\gui\day4_vehicle_simulator.py`
- **Main outcome:** Compare nominal, sensor, actuator and disturbance cases
  using the same controller

The PyQt simulator exposes more fault types and parameter controls than the
cumulative 3D-like simulator. Use it to understand the fault mechanisms before
implementing the focused range-sensor supervisor in Lesson 5.

## Start and identify the views

Run:

```bat
py -3.12 day_4\gui\day4_vehicle_simulator.py
```

Identify:

- true vehicle pose;
- measured pose;
- reference road;
- lead vehicle;
- state/mode indicator;
- fault controls;
- tracking, speed, gap and actuation plots;
- current metric cards.

Select:

```text
Lesson 1 — nominal success
```

Press **Apply and reset scenario**, then run.

Create:

| Scenario | Pass? | Mean path error | Max path error | Speed RMSE | Min gap | Main observation |
|---|---|---:|---:|---:|---:|---|
| Nominal | | | | | | |
| Sensor delay | | | | | | |
| Weak braking | | | | | | |
| Lateral push | | | | | | |
| Combined | | | | | | |

## Experiment 1: nominal reference

Before running, state the expected:

- path error;
- speed error;
- safety result;
- completion.

Run long enough to observe straight and curved road sections. Record the
metrics. Then answer:

1. Which controller layers are active?
2. Does a visually smooth run guarantee a small worst-case error?
3. Which result will be used as the comparison baseline?

The nominal case is not the final goal. It is the reference against which
disturbed cases are compared.

## Experiment 2: delay

Select:

```text
Lesson 1 — sensor and actuator delay
```

Before running, predict:

- whether the measured and true traces separate;
- whether steering begins earlier or later;
- whether maximum error changes more than mean error;
- whether increasing controller aggressiveness would always help.

Run and record the table.

Inspect the delay controls. If time permits, compare two values while leaving
everything else unchanged:

| Sensor delay | Actuator delay | Max path error | Steering activity | Pass/fail |
|---:|---:|---:|---:|---|
| supplied | supplied | | | |
| changed | unchanged | | | |

Explain delay using control-loop timing:

1. the vehicle moves;
2. the controller receives an old state;
3. it calculates a command for that old state;
4. the actuator applies the command later;
5. the real vehicle has already changed again.

## Experiment 3: weak braking

Select:

```text
Lesson 1 — weak braking
```

Predict:

- whether the requested brake changes;
- whether applied braking is equally strong;
- whether minimum gap or path error is more affected;
- which earlier parameter could provide additional margin.

Run and record.

The important comparison is requested versus applied authority. A controller
may correctly request heavy braking while the model applies only a fraction.

Calculate a simple check. If requested normalized brake is $u_b=0.80$ and
braking efficiency is $\eta_b=0.58$:

$$
u_{b,\mathrm{applied}}
=\eta_bu_b
=0.58(0.80)
=0.464.
$$

This does not mean the actual deceleration is exactly 46.4% of a fixed value;
the full vehicle dynamics still matter. It does show why the requested command
alone is insufficient evidence.

## Experiment 4: lateral push

Select:

```text
Lesson 3 — lateral push
```

Record:

- time of disturbance;
- maximum true path error;
- recovery time;
- whether road departure occurs;
- steering peak after the push.

Answer:

1. Why may maximum error increase strongly while mean error changes only
   slightly?
2. Does a successful recovery prove tolerance to every lateral disturbance?
3. Which Day 2 controller performs the recovery?

??? success "Expected reasoning"
    The push produces a short transient, so it dominates the maximum but has
    less influence on the average. The result applies only to this magnitude,
    direction, timing, road and speed. Pure Pursuit provides the steering
    recovery.

## Experiment 5: combined disturbance

Select:

```text
Lesson 3 — combined disturbance
```

Before running, choose one prediction:

- the combined result is approximately the sum of individual effects;
- one fault dominates;
- interaction creates a new failure.

Run and support or reject your prediction using at least three metrics.

| Evidence type | Metric | Result | Interpretation |
|---|---|---:|---|
| Safety | | | |
| Tracking | | | |
| Actuation/comfort | | | |

Fault effects are not necessarily additive. Delay may cause a larger steering
command exactly when authority is reduced, or braking loss may become important
only after traffic closes the gap.

## Guided conclusion

Complete:

```text
The nominal run __________.
The most influential single fault was __________ because __________.
The combined case was / was not predictable from the individual cases because
__________.
The safety decision relied primarily on __________.
In Lesson 5 I expect the range dropout to require __________.
```

## Troubleshooting

### The GUI is too large

Maximize the window or reduce Windows display scaling temporarily.

### The simulator appears to use old settings

Press **Apply and reset scenario** after selecting a preset or changing values.

### A metric is blank

Run the scenario for longer. Some traffic metrics appear only after a valid
lead interaction.

## Fast-finisher extension

Use the noise preset and compare two fixed random seeds. Then keep one seed
fixed and compare two controller configurations. Explain which comparison is
scientifically fairer.
