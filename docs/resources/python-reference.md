# Python quick reference

This page will collect the small set of Python operations used repeatedly in the exercises: variables, functions, NumPy arrays, plotting and parameter sweeps.

```python
def speed_controller(target_speed, measured_speed, kp):
    error = target_speed - measured_speed
    command = kp * error
    return command
```

The workshops focus on modifying and evaluating short controller functions rather than writing large programs from scratch.

