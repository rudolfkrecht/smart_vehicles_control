# Lesson 2 — Hands-on Python demonstrations

- **Format:** Guided Python experiments
- **Main outcome:** Connect fault definitions, plots and objective metrics

For every experiment:

1. read the scenario;
2. predict the effect;
3. run the unchanged baseline;
4. record quantitative evidence;
5. change one value if instructed;
6. explain the result.

Run commands from the extracted Day 4 package root.

## Preparation

Check the environment:

```bat
python setup_check.py
```

Create this evidence table:

| Experiment | Prediction | Safety result | Main changed metric | Explanation |
|---|---|---|---|---|
| Nominal | | | | |
| Delay + weak steering | | | | |
| Quantitative scorecard | | | | |
| Repeatable batch | | | | |
| Combined challenge | | | | |

## Nominal success is not enough

Run:

```bat
python courses\day_4\demos\lesson1_nominal_is_not_enough.py
```

The script uses one controller in two cases:

- a nominal practice road;
- the same task with sensor delay, actuator delay, steering bias and reduced
  steering authority.

Before viewing the result, predict:

- which trace separates first;
- whether average or maximum path error changes more;
- whether the car remains on the road;
- whether speed control or steering is affected more strongly.

Record:

| Case | Mean $|e_y|$ | Maximum $|e_y|$ | Speed RMSE | Minimum gap | Pass/fail |
|---|---:|---:|---:|---:|---|
| Nominal | | | | | |
| Disturbed | | | | | |

Explain why one passing case does not prove the other.

## Read a quantitative scorecard

Run:

```bat
python courses\day_4\demos\lesson2_quantitative_evaluation.py
```

The terminal first checks:

```text
[0.2, -0.4, 0.1, 0.5, -0.3] m
```

Calculate before comparing:

$$
\operatorname{MAE}
=\frac{0.2+0.4+0.1+0.5+0.3}{5}
=0.30\ \mathrm m.
$$

The maximum absolute error is $0.50\ \mathrm m$, and:

$$
\operatorname{RMSE}
=\sqrt{\frac{0.2^2+(-0.4)^2+0.1^2+0.5^2+(-0.3)^2}{5}}
\approx0.332\ \mathrm m.
$$

In the complete scorecard, identify one metric from each category:

| Category | Selected metric | Why it matters |
|---|---|---|
| Safety | | |
| Tracking | | |
| Speed | | |
| Comfort | | |
| Completion | | |

Then answer: could a controller have a smaller mean path error but still be
rejected? Give one numerical example.

??? success "Expected reasoning"
    Yes. A controller could have $0.10$ m mean path error and one $4.2$ m
    road excursion. The average is good, but the road departure fails the
    safety gate.

## Repeatable batch testing

Run:

```bat
python courses\day_4\demos\lesson3_repeatable_batch_testing.py
```

The same controller is tested from:

- centred pose;
- positive and negative lateral offsets;
- positive and negative heading offsets.

Record:

| Test ID | Initial change | Pass/fail | Maximum path error | Important note |
|---|---|---|---:|---|
| 1 | centred | | | |
| 2 | left offset | | | |
| 3 | right offset | | | |
| 4 | positive heading | | | |
| 5 | negative heading | | | |

Identify the worst case. Do not choose only by average error. Use:

1. failed case before passed case;
2. collision;
3. road departure;
4. largest maximum error.

Now open:

```text
day_4\student\exercise_batch_testing.py
```

Change:

```python
ENABLE_NOISE = False
```

to:

```python
ENABLE_NOISE = True
```

Run:

```bat
python courses\day_4\student\exercise_batch_testing.py --no-show
```

Why is `RANDOM_SEED = 220` important when two controllers are compared?

## Observe combined effects

Run:

```bat
python courses\day_4\demos\lesson4_challenge_preparation.py
```

Compare the aggressive and balanced configurations.

Record:

| Configuration | Practice passes | Collision/departure? | Strong point | Weak point |
|---|---:|---|---|---|
| Aggressive | | | | |
| Balanced | | | | |

An aggressive configuration may:

- complete more distance;
- follow smaller gaps;
- react more strongly;
- amplify noise or delay;
- fail a safety gate.

A balanced configuration may sacrifice progress or comfort to increase
robustness margin. Neither label is enough by itself; use the metrics.

## Engineering conclusion

Complete:

```text
The nominal demonstration showed __________.
The disturbed test changed __________ most strongly.
I selected __________ as the worst-case metric because __________.
A fixed seed is needed because __________.
The safest-looking plot was / was not the best scored result because
__________.
```

## Troubleshooting

### No plot window appears

Install the packages:

```bat
python -m pip install -r requirements.txt
```

### A command reports that a module is missing

Run the command from the package root. Confirm that `day_4` and `simulator`
are visible when you run:

```bat
dir
```

### You want to run without plots

Most demonstration and student scripts accept:

```bat
--no-show
```

## Fast-finisher extension

In `exercise_batch_testing.py`, change only `POSITION_NOISE_STD` from
$0.12$ m to $0.24$ m while keeping the seed fixed. Predict which metric will
change most, run the experiment and write a three-sentence explanation.
