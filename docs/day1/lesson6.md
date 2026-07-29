# 6. PI control challenge

## Challenge

Use integral action to reject a persistent disturbance while recognizing saturation and windup.

The PI controller uses accumulated speed error:

\[
I_k=I_{k-1}+e_k\Delta t,
\qquad
u_k=K_Pe_k+K_II_k.
\]

## Student challenge

```bash
python day_1_longitudinal/student/challenge_pi_control.py
```

The starter has `KI = 0.00`, so it initially behaves like the P baseline. Test
small positive values and meet both criteria:

- final error below 0.3 m/s;
- overshoot below 10%.

The expected behaviour of a balanced solution is:

![P and PI hill response](images/lesson6_pi_solution.png)

## Windup demonstration

In the graphical simulator, select **Windup case** and run once with
**Anti-windup** disabled. Then enable it, reset, and repeat the same scenario.

```bash
python day_1_longitudinal/demos/lesson6_windup.py
```

![Integral windup comparison](images/lesson6_windup.png)

The steep hill makes the target temporarily unreachable. Without anti-windup,
the integral term continues growing while the actuator is saturated, causing a
large response after the hill ends.
