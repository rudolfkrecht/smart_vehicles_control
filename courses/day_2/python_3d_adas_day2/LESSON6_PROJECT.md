# Lesson 6 — Individual full-lap project

The complete student instructions and acceptance table are in:

```text
docs/day2/lesson6.md
```

## Project command

```bat
py -3.12 run_simulator.py --headless --controller student --duration 75 --target-speed 12 --csv results\final_full_lap.csv
```

Tune:

```python
self.BASE_LOOKAHEAD_M
self.SPEED_GAIN_S
```

using:

$$
L_d=L_{d,0}+K_vv.
$$

The final controller must complete at least one lap, remain on the road, and
satisfy the accuracy, steering-activity and lateral-acceleration criteria in the
lesson page.
