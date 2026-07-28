# 3. Vehicle simulation

## Learning objective

Understand vehicle state, simulation time steps and a simplified longitudinal-motion model.

## Model

The simulator evaluates the longitudinal force balance:

\[
m a = F_\mathrm{actuator} - F_\mathrm{rolling}
      - c_\mathrm{drag}v^2 - F_\mathrm{hill}.
\]

It then updates speed and position with a discrete time step.

## Prepared demonstration

In the graphical simulator, select **Open loop** and change **Fixed command**
while the vehicle moves. The vehicle does not use measured speed to correct its
motion.

```bash
python day_1_longitudinal/demos/lesson3_open_loop.py
```

![Open-loop drag comparison](../assets/images/day1/lesson3_open_loop.png)

Safe modifications:

```python
OPEN_LOOP_COMMAND = 0.35
DRAG_VALUES = (2.0, 4.0, 8.0)
```

Predict which vehicle will be fastest before running. Notice that a constant
command does not create constant acceleration because drag increases with
speed.
