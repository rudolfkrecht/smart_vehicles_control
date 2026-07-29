# Lesson 6 — Independent cruise-control project

- **Duration:** 45 minutes
- **Format:** Independent team milestone
- **Editable file:** `student_controller.py`
- **Required output:** Controller, CSV evidence, results table and two slides

## Project brief

Develop a longitudinal controller that drives the simulated car around the
closed highway while tracking a 15 m/s reference. The track contains flat
sections, a 5-degree ascent, an elevated straight and a 5-degree descent.
Steering is supplied for Day 1.

The controller must:

- reach at least 90% of the target within 10 seconds;
- have less than 10% overshoot;
- have less than 0.25 m/s mean speed error during the final two seconds;
- limit the maximum uphill speed error to 1.0 m/s after initial recovery;
- never apply throttle and brake simultaneously;
- keep every command within its defined range;
- include an explicit anti-windup method;
- complete the experiment without leaving the road.

## Rules

You may edit:

```text
student_controller.py
```

For the standard milestone, do not edit:

```text
simulator/model.py
simulator/track.py
simulator/simulation.py
```

The purpose is to improve the controller, not to make the car or road easier.

## 0–5 min — Define the method

Write:

```text
Control structure:
Candidate KP range:
Candidate KI range:
Anti-windup method:
Metrics:
Number of planned experiments:
```

Predict which track section will produce the largest positive speed error and
which will produce the largest negative speed error.

## 5–15 min — Establish a reproducible baseline

Run the unchanged starter:

```bash
python run_simulator.py \
  --headless \
  --controller student \
  --duration 60 \
  --csv baseline.csv
```

Record the printed metrics. Then run it graphically and identify when each
important event occurs:

- start from rest;
- beginning of the ascent;
- elevated section;
- descent;
- first curve.

## 15–30 min — Implement and tune

Modify the controller using a controlled experiment sequence.

Recommended approach:

1. Tune $K_P$ with $K_I=0$.
2. Keep the selected $K_P$ fixed.
3. Increase $K_I$ from a small positive value.
4. Add conditional integration or another justified anti-windup method.
5. Retest the complete 60-second route.

Use a table:

| Run | $K_P$ | $K_I$ | Anti-windup | Rise time | Overshoot | Final error | Max lane error | Pass? |
|---:|---:|---:|---|---:|---:|---:|---:|---|
| 1 | | | | | | | | |
| 2 | | | | | | | | |
| 3 | | | | | | | | |
| 4 | | | | | | | | |

Changing several parameters at the same time makes the result difficult to
interpret. Change one design decision per run.

## 30–38 min — Validate a final candidate

Run:

```bash
python run_simulator.py \
  --headless \
  --controller student \
  --duration 90 \
  --csv final_validation.csv
```

Then open the graphical simulator and observe the complete response. Verify:

```text
[ ] throttle is always in [0, 1]
[ ] brake is always in [0, 1]
[ ] steering is within its permitted range
[ ] throttle and brake are never positive together
[ ] speed does not become negative
[ ] car remains on the road
[ ] anti-windup is present and explained
```

## 38–43 min — Prepare two slides

### Slide 1 — Problem and method

Include:

- target and performance requirements;
- controller equation or block diagram;
- selected parameters;
- anti-windup method;
- test route.

### Slide 2 — Evidence and interpretation

Include:

- a screenshot or speed-history plot;
- baseline-versus-final results;
- one design trade-off;
- one remaining limitation.

## 43–45 min — Submit the checkpoint

Submit:

```text
student_controller.py
baseline.csv
final_validation.csv
two presentation slides
completed results table
three-sentence engineering conclusion
```

The conclusion must state:

1. what was changed;
2. what measurable result improved;
3. what remains uncertain or unsatisfactory.

## Optional variants for different teams

The instructor may assign one unseen condition:

| Variant | Change |
|---|---|
| A | Target speed 12 m/s |
| B | Target speed 20 m/s |
| C | Vehicle mass 1500 kg |
| D | Maximum drive force 3800 N |
| E | Drag coefficient 5.5 N/(m/s)^2 |
| F | Controller update interval doubled |

The final controller should be tested without retuning. The team must explain
why its performance changes.

