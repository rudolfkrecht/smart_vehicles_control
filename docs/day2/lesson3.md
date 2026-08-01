# Lesson 3 — Numerical engineering exercises

- **Main outcome:** Calculate one complete chain from vehicle state to steering
  command

Use radians inside trigonometric calculations. Include units and sign
conventions. Attempt each exercise before opening its solution.

## Formula sheet

$$
\dot{x}=v\cos\psi,
\qquad
\dot{y}=v\sin\psi,
\qquad
\dot{\psi}=\frac{v}{L}\tan\delta
$$

$$
R=\frac{L}{\tan\delta},
\qquad
a_y=\frac{v^2}{R}
$$

$$
\mathbf n_r=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix},
\qquad
e_y=(\mathbf p-\mathbf p_r)^\mathsf T\mathbf n_r
$$

$$
e_\psi=\operatorname{wrap}(\psi-\psi_r)
$$

$$
\alpha=
\operatorname{wrap}\left[
\operatorname{atan2}(y_t-y,x_t-x)-\psi
\right]
$$

$$
\delta=
\tan^{-1}\left(
\frac{2L\sin\alpha}{L_d}
\right)
$$

## Exercise 1: one bicycle-model update

A car has:

| Quantity | Value |
|---|---:|
| Initial position | $(x_0,y_0)=(0,0)$ m |
| Initial heading | $\psi_0=15^\circ$ |
| Speed | $v=10$ m/s |
| Wheelbase | $L=2.8$ m |
| Steering angle | $\delta=8^\circ$ |
| Time step | $\Delta t=0.10$ s |

Calculate:

1. $\dot{x}$ and $\dot{y}$;
2. yaw rate $\dot{\psi}$ in rad/s and deg/s;
3. the next Euler state $(x_1,y_1,\psi_1)$;
4. the distance travelled during the step.

Write the signs before using a calculator.

??? success "Worked solution"
    Convert the angles:

    $$
    \psi_0=15^\circ=0.2618\ \mathrm{rad},
    \qquad
    \delta=8^\circ=0.1396\ \mathrm{rad}.
    $$

    Velocity components:

    $$
    \dot{x}=10\cos15^\circ=9.659\ \mathrm{m/s},
    $$

    $$
    \dot{y}=10\sin15^\circ=2.588\ \mathrm{m/s}.
    $$

    Yaw rate:

    $$
    \dot{\psi}
    =\frac{10}{2.8}\tan8^\circ
    =0.502\ \mathrm{rad/s}
    =28.76^\circ/\mathrm{s}.
    $$

    Euler update:

    $$
    x_1=0+9.659(0.10)=0.966\ \mathrm m,
    $$

    $$
    y_1=0+2.588(0.10)=0.259\ \mathrm m,
    $$

    $$
    \psi_1
    =15^\circ+28.76^\circ/\mathrm{s}(0.10\ \mathrm{s})
    =17.88^\circ.
    $$

    The travelled distance is approximately:

    $$
    v\Delta t=10(0.10)=1.00\ \mathrm m.
    $$

    The global displacement magnitude from the Euler update is also 1.00 m,
    apart from rounding.

## Exercise 2: radius and dynamic plausibility

For $L=2.8$ m and $\delta=12^\circ$:

1. calculate the ideal turning radius;
2. calculate $a_y$ at 8 m/s;
3. calculate $a_y$ at 12 m/s;
4. express both accelerations as fractions of $g=9.81$ m/s²;
5. if the simplified friction limit is $a_y\leq\mu g$ with $\mu=0.70$,
   calculate the maximum speed for this radius.

Explain why the same geometric path may be acceptable at one speed and
unacceptable at another.

??? success "Worked solution"
    Radius:

    $$
    R=\frac{2.8}{\tan12^\circ}=13.17\ \mathrm m.
    $$

    At 8 m/s:

    $$
    a_y=\frac{8^2}{13.17}=4.86\ \mathrm{m/s^2}
    \approx0.50g.
    $$

    At 12 m/s:

    $$
    a_y=\frac{12^2}{13.17}=10.94\ \mathrm{m/s^2}
    \approx1.12g.
    $$

    The speed increased by a factor of $12/8=1.5$, so the lateral
    acceleration increased by $1.5^2=2.25$.

    From:

    $$
    \frac{v^2}{R}\leq\mu g,
    $$

    the simplified maximum speed is:

    $$
    v_{\max}
    =\sqrt{\mu gR}
    =\sqrt{0.70(9.81)(13.17)}
    =9.51\ \mathrm{m/s}.
    $$

    This friction-circle calculation is still simplified, but it shows why a
    kinematically correct path is not automatically dynamically feasible.

## Exercise 3: signed tracking errors

At the nearest reference point:

$$
\mathbf p_r=
\begin{bmatrix}
20\\
5
\end{bmatrix}
\mathrm m,
\qquad
\psi_r=30^\circ.
$$

The vehicle state is:

$$
\mathbf p=
\begin{bmatrix}
19\\
7
\end{bmatrix}
\mathrm m,
\qquad
\psi=45^\circ.
$$

Calculate:

1. the displacement $\mathbf p-\mathbf p_r$;
2. the path's left normal $\mathbf n_r$;
3. signed cross-track error $e_y$;
4. heading error $e_\psi$;
5. which side of the oriented path the car occupies.

Then answer: what would happen to $e_y$ if the vehicle were reflected across
the reference line while retaining the same distance?

??? success "Worked solution"
    Displacement:

    $$
    \mathbf p-\mathbf p_r=
    \begin{bmatrix}
    -1\\
    2
    \end{bmatrix}
    \mathrm m.
    $$

    Left normal:

    $$
    \mathbf n_r=
    \begin{bmatrix}
    -\sin30^\circ\\
    \cos30^\circ
    \end{bmatrix}
    =
    \begin{bmatrix}
    -0.5\\
    0.866
    \end{bmatrix}.
    $$

    Cross-track error:

    $$
    e_y=
    \begin{bmatrix}
    -1 & 2
    \end{bmatrix}
    \begin{bmatrix}
    -0.5\\
    0.866
    \end{bmatrix}
    =0.5+1.732
    =2.232\ \mathrm m.
    $$

    The positive sign means that the vehicle lies to the left of the oriented
    reference path.

    Heading error:

    $$
    e_\psi=45^\circ-30^\circ=15^\circ=0.262\ \mathrm{rad}.
    $$

    Reflection to the other side preserves the magnitude but reverses the
    sign: $e_y=-2.232$ m.

## Exercise 4: Pure Pursuit steering

The rear-axle reference point is at $(0,0)$ and the vehicle heading is
$\psi=0^\circ$. The wheelbase is $L=2.8$ m.

### Case A

The preview target is at $(8,2)$ m.

Calculate:

1. target bearing;
2. relative angle $\alpha$;
3. geometric look-ahead distance;
4. steering command in radians and degrees;
5. turn direction.

### Case B

Move the preview target to $(14,2)$ m and repeat. Explain the change.

??? success "Worked solution"
    **Case A**

    $$
    \alpha
    =\operatorname{atan2}(2,8)-0
    =14.04^\circ.
    $$

    $$
    L_d=\sqrt{8^2+2^2}=8.246\ \mathrm m.
    $$

    $$
    \delta
    =\tan^{-1}\left(
    \frac{2(2.8)\sin14.04^\circ}{8.246}
    \right)
    =0.163\ \mathrm{rad}
    =9.35^\circ.
    $$

    The command is positive, so the model turns left.

    **Case B**

    $$
    \alpha=\operatorname{atan2}(2,14)=8.13^\circ,
    $$

    $$
    L_d=\sqrt{14^2+2^2}=14.142\ \mathrm m,
    $$

    $$
    \delta
    =\tan^{-1}\left(
    \frac{2(2.8)\sin8.13^\circ}{14.142}
    \right)
    =0.0559\ \mathrm{rad}
    =3.20^\circ.
    $$

    Moving the target farther ahead reduced both the relative target angle and
    the steering request. The response becomes smoother but may correct an
    offset more slowly or cut a curve.

## Engineering check

Complete:

| Exercise | Main result | Unit/sign check | Engineering meaning |
|---:|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |

Answer:

1. Which exercise connects speed to safety most clearly?
2. Which calculation would fail if degrees were passed to a function expecting
   radians?
3. Which result will you use when tuning look-ahead in Lesson 4?

## Reserve task

A speed-dependent look-ahead rule is:

$$
L_d=L_{d,0}+K_vv.
$$

For $L_{d,0}=4$ m and $K_v=0.35$ s, calculate $L_d$ at 6, 12 and 16 m/s.
Explain the unit of $K_v$.

??? success "Reserve-task solution"
    $$
    L_d(6)=4+0.35(6)=6.10\ \mathrm m,
    $$

    $$
    L_d(12)=4+0.35(12)=8.20\ \mathrm m,
    $$

    $$
    L_d(16)=4+0.35(16)=9.60\ \mathrm m.
    $$

    The gain has units of seconds because:

    $$
    [K_v]=\frac{\mathrm m}{\mathrm{m/s}}=\mathrm s.
    $$
