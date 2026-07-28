# Lesson 3 — Reference paths and tracking errors

## Learning objective

Distinguish waypoints, a continuous path, nearest point, look-ahead target,
cross-track error and heading error.

## Waypoints versus path

Waypoints are sparse points used to describe the desired route. A controller
needs local geometric information between them, so the supplied code creates a
dense cubic Hermite path:

```text
sparse waypoints → smooth interpolation → dense path samples
```

Directly switching from one waypoint to the next can produce abrupt changes in
target direction and oscillatory steering.

## Nearest point

The nearest sampled path point anchors the error calculation. To avoid jumping
backward, the implementation searches a local forward window and forces the
index to progress.

## Signed cross-track error

Let \(\mathbf p\) be the vehicle position, \(\mathbf p_r\) the nearest reference
point and \(\mathbf n_r\) the path's left normal:

\[
e_y=(\mathbf p-\mathbf p_r)^\mathsf T\mathbf n_r
\]

The sign tells us which side of the path the vehicle is on. The magnitude is
the lateral distance from the reference.

## Heading error

\[
e_\psi=\operatorname{wrap}(\psi-\psi_r)
\]

A vehicle can have \(e_y=0\) while facing across the road, so position error
alone is insufficient.

## Prepared visualisation

```bat
python day_2\demos\lesson3_path_errors.py
```

![Path, waypoints and tracking-error geometry](images/lesson3_path_errors.png)

## Predict before running

1. Which point is nearest to the vehicle?
2. Which path sample lies approximately one look-ahead distance ahead?
3. Is the signed cross-track error positive or negative?
4. If the vehicle is on the centreline but points 20° left, which error is zero?

## GUI activity

Choose **Lesson 3 — tracking geometry**, pause the simulator, and toggle the
geometry overlay. Move only the initial offset and heading error, then reset.
