# Lesson 2 — Exploring steering behaviour

## Learning objective

Separate the effects of steering angle, wheelbase and speed on path geometry
and lateral acceleration.

## Turning radius

For steady steering:

\[
R=\frac{L}{\tan\delta}
\]

Therefore:

- increasing \(|\delta|\) reduces \(|R|\);
- increasing \(L\) increases \(|R|\);
- speed does not appear in the geometric radius equation.

This last point is easy to misunderstand. At higher speed the vehicle covers a
larger part of the same circle in a fixed amount of time, so time-based plots
look different even though the radius is unchanged.

## Lateral acceleration

\[
a_y=\frac{v^2}{R}
=\frac{v^2}{L}\tan\delta
\]

Speed is squared. Doubling speed at the same radius quadruples lateral
acceleration. A geometrically valid kinematic trajectory may therefore be
uncomfortable or impossible when tire-force limits are considered.

## Prepared experiment

```bat
python day_2\demos\lesson2_steering_exploration.py
```

![Steering, wheelbase, speed and lateral acceleration](images/lesson2_steering_exploration.png)

## Calculation exercise

Use \(L=2.7\) m and \(\delta=12^\circ\).

1. Calculate \(R\).
2. Calculate \(a_y\) at 4 m/s.
3. Calculate \(a_y\) at 8 m/s.
4. Explain why the second value is four times the first.

## Engineering extension

The kinematic model assumes rolling without lateral slip. A real vehicle also
depends on tire friction, load, road surface, steering dynamics and body roll.
Day 4 will test the consequences of model mismatch.
