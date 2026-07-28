# 4. Proportional control

## Learning objective

Explain negative feedback and predict how proportional gain affects the response.

The proportional law is

\[
e(t)=v_\mathrm{target}(t)-v(t), \qquad
u(t)=K_P e(t).
\]

The simulator limits the command to:

\[
-1 \le u(t) \le 1.
\]

## Prepared demonstration

Open the graphical simulator, select **P + hill**, and change **Kp** while the
simulation runs. Reset before each formal comparison.

```bash
python day_1_longitudinal/demos/lesson4_p_control.py
```

![Effect of proportional gain](../assets/images/day1/lesson4_p_control.png)

The only live edit is:

```python
KP_VALUES = (0.08, 0.35, 1.20)
```

A larger gain reduces the steady-state error in this model, but also increases
command saturation and sensitivity to effects omitted from the simulation.
