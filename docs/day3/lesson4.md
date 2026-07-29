# Lesson 4 — Guided PyQt simulator exercises

- **Format:** Guided visual laboratory
- **Simulator:** `day_3\gui\day3_vehicle_simulator.py`
- **Main outcome:** Select and justify ACC spacing and safety parameters using
  repeatable evidence

The PyQt simulator gives you direct control over:

- driver/global speed;
- curve-aware road speed;
- lead-vehicle scenario;
- standstill gap;
- time headway;
- emergency TTC and gap;
- continuous ACC and behaviour-state supervision.

For every run:

1. choose or reset a preset;
2. predict the result;
3. change only one parameter;
4. apply and reset;
5. run the complete scenario;
6. record metrics and explain the change.

## Start and identify the system

From the Day 3 package root:

```bat
py -3.12 day_3\gui\day3_vehicle_simulator.py
```

Select:

```text
Lesson 4 — balanced ACC
```

Find:

- ego and lead vehicles;
- road, traffic and selected targets;
- actual and desired gap;
- TTC;
- Cruise, Follow, Brake and Emergency state;
- speed, gap, acceleration and cross-track plots;
- completion and safety metrics.

Answer:

1. Which colour represents the ego vehicle?
2. Where can you see the selected target?
3. Which trace shows the desired gap?
4. Which metric would reveal a collision?

## Establish a no-traffic reference

Select:

```text
Lesson 1 — integrated baseline
```

This preset disables traffic. Run it and record:

| Quantity | No-traffic result |
|---|---:|
| Requested speed | |
| Minimum gap | not applicable |
| Peak longitudinal acceleration | |
| Peak lateral acceleration | |
| Path completion | |
| Road departure | |

The purpose is not to tune anything. Observe how the vehicle behaves when only
the speed and path controllers are active.

Before continuing, predict what will happen if the same controller encounters
a stopped lead vehicle without ACC.

## Short versus balanced headway

Compare these presets:

```text
Lesson 4 — short-headway ACC
Lesson 4 — balanced ACC
```

Run each from reset. Record:

| Metric | Short headway | Balanced headway |
|---|---:|---:|
| Time headway | | |
| Standstill gap | | |
| Minimum gap | | |
| Minimum finite TTC | | |
| Collision samples | | |
| Completion | | |
| Peak deceleration | | |
| State transitions | | |

Answer:

1. Which starts responding to the lead vehicle earlier?
2. Which maintains more space during the stop?
3. Does the safer-looking run complete less distance?
4. Is the short-headway setting acceptable merely because no collision occurs
   in one run?

Notice: The short-headway preset is deliberately aggressive. Treat an unsafe result as evidence, not as a failed software installation.

## Tune time headway systematically

Return to:

```text
Lesson 4 — balanced ACC
```

Keep:

- the same lead scenario;
- the same standstill gap;
- the same road and path settings;
- the same emergency thresholds.

Test:

$$
T_h\in\{1.0,\ 1.5,\ 2.0\}\ \mathrm s.
$$

Press **Apply and reset scenario** after every change.

Before running, calculate the desired gap at $v=12\ \mathrm{m/s}$ for each
candidate.

| $T_h$ | Calculated $d_{\mathrm{des}}$ at 12 m/s | Minimum gap | Minimum TTC | Completion | Peak deceleration | Decision |
|---:|---:|---:|---:|---:|---:|---|
| 1.0 s | | | | | | |
| 1.5 s | | | | | | |
| 2.0 s | | | | | | |

Selection order:

1. reject collision;
2. reject unacceptably small gap or TTC;
3. compare deceleration and state changes;
4. compare completion among the remaining candidates.

Do not select a controller using only completion.

??? success "Desired-gap check"
    With $d_0=5\ \mathrm m$ and $v=12\ \mathrm{m/s}$:

    | $T_h$ | $d_{\mathrm{des}}$ |
    |---:|---:|
    | 1.0 s | 17 m |
    | 1.5 s | 23 m |
    | 2.0 s | 29 m |

## Investigate standstill gap

Keep your chosen time headway. Compare:

$$
d_0\in\{3,\ 5,\ 8\}\ \mathrm m.
$$

Focus on the period when the lead vehicle is stopped.

| $d_0$ | Gap while stopped | Selected target | State | Restart behaviour |
|---:|---:|---:|---|---|
| 3 m | | | | |
| 5 m | | | | |
| 8 m | | | | |

Explain:

- why changing $d_0$ shifts the desired-gap curve at every speed;
- why $d_0$ matters most at low speed and standstill;
- why a larger $d_0$ can reduce road capacity.

## Safety-state thresholds

Select:

```text
Lesson 5 — late-braking states
```

Record the baseline times at which Brake and Emergency activate.

Then change only the emergency TTC:

| Emergency TTC | Brake time | Emergency time | Minimum gap | Collision | Interpretation |
|---:|---:|---:|---:|---:|---|
| 0.8 s | | | | | |
| 1.25 s | | | | | |
| 2.0 s | | | | | |

Predict before applying:

- a larger TTC threshold should activate Emergency earlier;
- earlier intervention may increase minimum gap;
- an excessively large threshold may cause unnecessary harsh braking.

The threshold is not a universal safety constant. It is meaningful only with
the assumed delays, braking capability and sensor behaviour.

## Checkpoint

Complete:

```text
Selected time headway:
Selected standstill gap:
Minimum gap:
Minimum finite TTC:
Collision samples:
Completion:
One safety benefit:
One efficiency or comfort cost:
One simulator limitation:
```

## Troubleshooting

### The Apply button appears to do nothing

Change the value, then select **Apply and reset scenario**. The experiment must
restart for a fair comparison.

### The vehicle stops early

Check the selected lead scenario and whether traffic is enabled. In
stop-and-go traffic, stopping can be correct controller behaviour.

### The selected target is lower than both the cruise setting and lead speed

The ego vehicle may be closing too quickly or the gap may be below the desired
value. The closing-speed correction can make the temporary target lower than
lead speed.

### Many settings changed at once

Reload a named preset. Repeat the experiment while changing one value only.

## Fast-finisher extension

Find two configurations that are not clearly better than each other:

- one with a larger safety margin but lower completion;
- one with higher completion but smaller safety margin.

Explain why this is a multi-objective design problem rather than a single
parameter-optimization exercise.
