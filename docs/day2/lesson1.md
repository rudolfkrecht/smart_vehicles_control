# Lesson 1 — Vehicle motion in two dimensions

## Learning objective

Explain the state variables and input quantities of a kinematic bicycle model,
then predict the direction of motion for several steering commands.

## From speed control to steering

Day 1 used position \(s\) and speed \(v\). In a plane, position requires two
coordinates and vehicle orientation matters:

- \(x,y\): rear-axle position in the global frame;
- \(\psi\): vehicle heading;
- \(v\): forward speed;
- \(\delta\): front-wheel steering angle;
- \(L\): wheelbase.

The bicycle abstraction replaces the left and right wheels on each axle with
one virtual wheel. It is not a one-wheeled vehicle; it is a compact geometric
model.

## Model equations

\[
\dot{x}=v\cos\psi
\]

\[
\dot{y}=v\sin\psi
\]

\[
\dot{\psi}=\frac{v}{L}\tan\delta
\]

The first two equations project forward speed onto the global axes. The third
states that yaw rate grows with speed and steering angle, and decreases with
wheelbase.

## Discrete simulation loop

For time step \(\Delta t\):

```python
x += speed * cos(heading) * dt
y += speed * sin(heading) * dt
heading += speed / wheelbase * tan(steering) * dt
```

The supplied implementation also limits steering angle and steering rate.

## Predict before running

1. What path results from \(\delta=0\)?
2. Which signs of \(\delta\) turn left and right?
3. Does heading change instantly or gradually?
4. What happens if the same steering command is applied to a longer vehicle?

## Prepared demonstration

```bat
python day_2\demos\lesson1_bicycle_motion.py
```

![Three constant-steering trajectories](images/lesson1_bicycle_motion.png)

## Summary

A steering input changes yaw rate. The changing heading then changes how
forward velocity is divided between the global \(x\) and \(y\) directions.
