# Lesson 1 — Theory: lateral motion and path following

- **Format:** Theory notes that support classroom presentation
- **Main outcome:** Explain how vehicle geometry, path errors and preview
  geometry produce a steering command

## Learning objectives

By the end of this lesson, you should be able to:

- describe the state and input of a kinematic bicycle model;
- calculate turning radius, yaw rate and lateral acceleration;
- distinguish waypoints, a reference path and a driven trajectory;
- define signed cross-track and heading error;
- explain the Pure Pursuit steering calculation;
- predict how look-ahead distance changes accuracy and smoothness;
- identify important limitations of the model.

## From longitudinal to lateral control

Day 1 considered motion along one coordinate. The controller measured speed and
produced throttle or brake. A vehicle driving on a road must also decide where
to steer.

The lateral-control loop contains:

| Element | Day 2 example |
|---|---|
| Reference | desired lane-centre path |
| Measurement | position $(x,y)$ and heading $\psi$ |
| Error | cross-track error $e_y$ and heading error $e_\psi$ |
| Controller | Pure Pursuit |
| Actuator command | steering angle $\delta$ |
| Plant | vehicle motion |
| Disturbance | initial offset, localization noise or lateral displacement |

Longitudinal and lateral control are separated for learning, but both move the
same car. Speed changes how quickly the vehicle reacts to a given steering
angle.

## Kinematic bicycle model

A passenger car has four wheels, but the left and right wheels can be replaced
by one virtual wheel at each axle when only planar geometry is required. This
is the **kinematic bicycle model**.

| Symbol | Meaning | Unit |
|---:|---|---:|
| $x,y$ | rear-axle position in the global frame | m |
| $\psi$ | vehicle heading | rad |
| $v$ | forward speed | m/s |
| $\delta$ | front-wheel steering angle | rad |
| $L$ | wheelbase | m |

The continuous model is:

$$
\dot{x}=v\cos\psi,
$$

$$
\dot{y}=v\sin\psi,
$$

$$
\dot{\psi}=\frac{v}{L}\tan\delta.
$$

Interpret the equations:

- $\dot{x}$ and $\dot{y}$ are the global components of forward velocity;
- positive $\delta$ produces a positive yaw rate in the model;
- greater speed produces faster heading change for the same steering;
- a longer wheelbase produces a slower heading change.

For a short time step $\Delta t$, explicit Euler integration gives:

```python
x += speed * math.cos(heading) * dt
y += speed * math.sin(heading) * dt
heading += speed / wheelbase * math.tan(steering) * dt
```

The supplied simulator also limits steering angle and steering rate. A command
can therefore change immediately while the applied steering changes gradually.

### Quick prediction

Without calculating:

1. What happens when $\delta=0$?
2. Does doubling $v$ double or halve yaw rate?
3. Does increasing $L$ make the same steering command more or less aggressive?

??? success "Check"
    1. The vehicle travels straight while its heading remains constant.
    2. Yaw rate doubles.
    3. The longer vehicle turns less aggressively because yaw rate decreases.

## Radius, yaw rate and lateral acceleration

For steady steering, the geometric turning radius is:

$$
R=\frac{L}{\tan\delta}.
$$

The yaw rate can also be written as:

$$
\dot{\psi}=\frac{v}{R}.
$$

The corresponding lateral acceleration is:

$$
a_y=\frac{v^2}{R}
=\frac{v^2}{L}\tan\delta.
$$

The radius equation contains no speed. At the same steering angle, an ideal
kinematic vehicle follows the same circle at 5 m/s and 10 m/s. However, lateral
acceleration contains $v^2$. Doubling speed multiplies $a_y$ by four.

This distinction is important:

- **geometric feasibility:** can the steering geometry describe the curve?
- **dynamic feasibility:** can the tires generate the required lateral force?

Day 2 mainly studies geometry. A small tracking error does not automatically
mean the motion is comfortable or physically feasible.

## From waypoints to a reference path

Sparse waypoints define the intended route, but a controller needs information
between them:

```text
sparse waypoints → smooth interpolation → dense reference path
```

Do not confuse these terms:

| Term | Meaning |
|---|---|
| Waypoint | one sparse point used to define the route |
| Reference path | continuous or densely sampled desired route |
| Driven trajectory | positions actually visited by the car |
| Nearest point | reference sample closest to the car |
| Preview point | target farther ahead on the reference path |

Directly aiming at each sparse waypoint can cause abrupt target changes. The
provided numerical model uses smooth cubic Hermite interpolation to create the
dense path used by the controller.

## Tracking errors

Let $\mathbf p$ be the vehicle position and $\mathbf p_r$ the nearest reference
point. The path heading at that point is $\psi_r$. Its left unit normal is:

$$
\mathbf n_r=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}.
$$

The signed cross-track error is:

$$
e_y=(\mathbf p-\mathbf p_r)^\mathsf T\mathbf n_r.
$$

In this sign convention:

- $e_y>0$: vehicle is left of the target path;
- $e_y<0$: vehicle is right of the target path;
- $e_y=0$: vehicle reference point is on the path.

Heading error is:

$$
e_\psi=\operatorname{wrap}(\psi-\psi_r).
$$

Angles must be wrapped into a consistent interval such as $[-\pi,\pi)$. Without
wrapping, headings of $179^\circ$ and $-179^\circ$ appear to differ by
$358^\circ$, although the shortest difference is only $2^\circ$.

A car can have zero cross-track error while pointing across the road. It can
also have zero heading error while travelling on a line parallel to the road.
Both errors therefore matter.

## Pure Pursuit

Pure Pursuit chooses a point $L_d$ metres ahead on the reference path. Let the
target bearing in the global frame be:

$$
\psi_t=\operatorname{atan2}(y_t-y,\;x_t-x).
$$

The target angle relative to the car is:

$$
\alpha=\operatorname{wrap}(\psi_t-\psi).
$$

The steering command is:

$$
\delta=
\tan^{-1}\left(
\frac{2L\sin\alpha}{L_d}
\right).
$$

The controller repeatedly:

1. finds the nearest path position;
2. selects a point ahead;
3. transforms the target direction relative to the car;
4. calculates steering;
5. applies steering limits;
6. repeats at the next time step.

### Look-ahead trade-off

| Look-ahead | Typical advantage | Typical risk |
|---|---|---|
| Short | fast correction, small geometric error | oscillation, high steering rate, noise sensitivity |
| Moderate | useful accuracy–smoothness compromise | must still match speed and curvature |
| Long | smooth and anticipatory | curve cutting, slower disturbance recovery |

One value rarely works equally well at every speed. A common extension is:

$$
L_d=L_{d,0}+K_vv,
$$

where $L_{d,0}$ is the base look-ahead and $K_v$ has units of seconds:

$$
[K_v]=\frac{\mathrm m}{\mathrm{m/s}}=\mathrm s.
$$

## Evaluation and concept check

You will use several metrics:

| Metric | Question answered |
|---|---|
| Mean $\|e_y\|$ | How accurate was ordinary tracking? |
| Maximum $\|e_y\|$ | What was the worst deviation? |
| Outside-road percentage | Did a safety boundary fail? |
| Steering-rate RMS | How active or oscillatory was steering? |
| Peak $a_y$ | How demanding was the curve dynamically? |
| Lap progress | Did the car complete the route? |

Answer before Lesson 2:

1. Why does speed not change ideal turning radius at fixed steering?
2. Why can higher speed still be unsafe?
3. Why are sparse waypoints insufficient for smooth steering?
4. Can $e_y=0$ while $e_\psi\ne0$?
5. Why can the configuration with the smallest mean error still be unsuitable?

??? success "Concept-check answers"
    1. Radius is determined by $R=L/\tan\delta$.
    2. Required lateral acceleration grows with $v^2$.
    3. Switching targets at sparse points can create discontinuous target
       directions.
    4. Yes. The car can be on the path while facing the wrong direction.
    5. It may have excessive peak error, steering activity, lateral
       acceleration or road departure.

## Model limitations

The Day 2 model does not simulate:

- lateral tire slip and saturation;
- changing friction;
- suspension, roll or load transfer;
- detailed steering-system compliance;
- localization delay or loss;
- combined tire-force limits during braking and cornering.

Use the model to study geometry and controller logic. Do not claim that a
controller is ready for a real vehicle solely because it succeeds here.
