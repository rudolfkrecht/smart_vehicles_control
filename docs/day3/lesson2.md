# Lesson 2 — Hands-on Python demonstrations

- **Format:** Guided Python experiments
- **Main outcome:** Connect ACC equations to plots and state changes

In this lesson, do not begin by changing parameters. For every experiment:

1. read the scenario;
2. predict the result;
3. run the script;
4. record evidence;
5. change one value;
6. explain the difference.

Run all commands from the extracted Day 3 package root.

## Prepare the evidence table

Create a table in your notes:

| Experiment | Prediction | Main observation | Metric or transition | Explanation |
|---|---|---|---|---|
| Headway baseline | | | | |
| Short headway | | | | |
| Long headway | | | | |
| Behaviour states | | | | |

Check the setup:

```bat
python setup_check.py
```

## Desired following distance

Open:

```text
day_3\student\exercise_acc_headway.py
```

Find:

```python
TIME_HEADWAY_CANDIDATES_S = (0.7, 1.5, 2.2)
STANDSTILL_GAP_M = 5.0
```

Before running, calculate $d_{\mathrm{des}}$ at $v=12\ \mathrm{m/s}$:

$$
d_{\mathrm{des}}=d_0+T_hv.
$$

| $T_h$ | Your calculated desired gap |
|---:|---:|
| 0.7 s | |
| 1.5 s | |
| 2.2 s | |

Run:

```bat
python courses\day_3\student\exercise_acc_headway.py
```

The script compares the candidates in the same lead-vehicle scenario. Record:

| $T_h$ | Minimum gap | Minimum finite TTC | Collision samples | Completion | Decision |
|---:|---:|---:|---:|---:|---|
| 0.7 s | | | | | |
| 1.5 s | | | | | |
| 2.2 s | | | | | |

Do not choose the setting that merely completes the greatest distance. First
reject collision or unacceptably small safety margins.

??? success "Calculation check"
    At 12 m/s with a 5 m standstill gap:

    $$
    d_{\mathrm{des}}(0.7)=5+0.7(12)=13.4\ \mathrm m,
    $$

    $$
    d_{\mathrm{des}}(1.5)=5+1.5(12)=23.0\ \mathrm m,
    $$

    $$
    d_{\mathrm{des}}(2.2)=5+2.2(12)=31.4\ \mathrm m.
    $$

## Compare complete headway responses

Run the prepared demonstration:

```bat
python courses\day_3\demos\lesson4_acc_headway.py
```

The lead vehicle:

1. begins at a steady speed;
2. slows;
3. stops;
4. waits;
5. accelerates again.

Inspect:

- ego and lead speeds;
- selected speed target;
- actual and desired gap;
- TTC during closing;
- collision markers;
- completion distance.

Answer:

1. Which controller begins reducing speed earliest?
2. Which follows most closely?
3. Which produces the largest minimum gap?
4. Does the longest headway always provide the best completion?
5. Which two metrics should be reported together?

Now replace only one candidate in
`TIME_HEADWAY_CANDIDATES_S`. Rerun and decide whether the new value improves
the trade-off.

## Behaviour-state transitions

Run:

```bat
python courses\day_3\demos\lesson5_behaviour_states.py
```

The terminal prints state transitions. Match the printed times to the plot.

Record:

| Transition | Time | Gap | TTC | Why did it occur? |
|---|---:|---:|---:|---|
| Cruise → Follow | | | | |
| Follow → Brake | | | | |
| Brake → Emergency, if present | | | | |
| Brake/Follow → Cruise | | | | |

Open the demonstration and find:

```python
EMERGENCY_TTC_S = 1.25
BRAKE_ENTRY_RATIO = 0.78
```

Predict:

- lowering `EMERGENCY_TTC_S` makes Emergency earlier or later?
- increasing `BRAKE_ENTRY_RATIO` makes Brake earlier or later?

Change only one parameter and rerun.

??? success "Expected direction"
    Lowering the emergency TTC threshold delays the Emergency transition.
    Increasing the brake-entry ratio usually activates Brake at a larger gap
    relative to the desired distance, so it tends to occur earlier.

## Integrated preview

Run:

```bat
python courses\day_3\demos\lesson6_workshop_preview.py
```

This demonstration combines:

- the speed controller from Day 1;
- Pure Pursuit from Day 2;
- traffic target selection;
- curve-aware speed constraints;
- state supervision.

For one time instant, identify:

| Quantity | Value |
|---|---:|
| Driver/global target | |
| Road target | |
| Traffic target | |
| Selected target | |
| Ego speed | |
| Lead speed | |
| Behaviour state | |

Check:

$$
v_{\mathrm{selected}}
=\min(v_{\mathrm{road}},v_{\mathrm{traffic}}).
$$

## Engineering conclusion

Complete:

```text
Increasing time headway changed __________.
Gap and TTC are both needed because __________.
The most useful plotted evidence was __________.
One setting that appeared efficient but unsafe was __________.
The deterministic scenario cannot represent __________.
```

Keep your table. You will use the same quantities in the numerical and
simulator lessons.

## Troubleshooting

### A graph window does not appear

Confirm the packages are installed:

```bat
python -m pip install -r requirements.txt
```

Run the command from the package root, not from inside `day_3\demos`.

### A script finishes but you cannot find its figure

The scripts may save images in the current working directory. Check:

```bat
dir *.png
```

### Results change after several edits

Restore the supplied constants, then change one value at a time. A useful
experiment must keep all other conditions fixed.

## Fast-finisher extension

Create one additional headway candidate between 1.0 and 2.0 s. Write a
three-sentence engineering argument:

1. one safety metric;
2. one efficiency or completion metric;
3. one comfort or control-effort observation.
