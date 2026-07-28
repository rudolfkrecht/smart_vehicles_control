# 5. Cruise-control workshop

## Challenge

Tune a proportional cruise controller systematically and evaluate it using quantitative metrics.

## Student file

```bash
python day_1_longitudinal/student/workshop_p_tuning.py
```

The starter runs immediately with three supplied gains. Students modify only:

```python
KP_VALUES = (0.10, 0.35, 0.70)
ENABLE_HILL = False
```

## Workflow

1. Run the unchanged flat-road baseline.
2. Record rise time, overshoot, final error and saturation.
3. Narrow the gain range and run again.
4. Select a gain and set `ENABLE_HILL = True`.
5. Explain why the hill creates a persistent error with P control.

Use the graphical simulator's **P + hill** preset to inspect each candidate
visually before recording its metrics with the student script.

| \(K_P\) | Rise time | Overshoot | Final error | Saturation | Suitable? |
|---:|---:|---:|---:|---:|---|
| | | | | | |
| | | | | | |
| | | | | | |

Advanced task:

```bash
python day_1_longitudinal/student/advanced_p_sweep.py
```
