# Lesson 4 — Guided exercises with the PyQt simulator

- **Format:** Guided work in groups
- **Main tool:** Day 1 PyQt vehicle simulator
- **Required output:** One completed results sheet and a short engineering conclusion

This lesson connects the numerical work from Lesson 3 with an interactive
simulation. You will not simply change values and watch curves. For every
exercise, follow the same engineering sequence:

1. **Calculate** an expected value.
2. **Predict** the behaviour before starting the simulation.
3. **Run** a controlled experiment.
4. **Measure** the result.
5. **Explain** any difference between theory and simulation.

Work through each exercise with your partner or group, then compare your
calculation, prediction and measured result during the class discussion.

## Learning objectives

By the end of the lesson, you should be able to:

- calculate the open-loop command required to maintain a specified speed;
- explain why actuator command and vehicle acceleration are not the same thing;
- tune a proportional cruise controller against quantitative requirements;
- convert road angle into an opposing longitudinal force;
- explain why a P controller retains steady-state error on an uphill road;
- add integral action and justify a PI-controller configuration using evidence;
- recognize saturation and distinguish a physical limit from poor tuning.

## Start the simulator

From the root of the repository on Windows, run:

```bash
py -3.12 day_1_longitudinal/gui/day1_vehicle_simulator.py
```

If the `py` launcher is unavailable, use:

```bash
python day_1_longitudinal/gui/day1_vehicle_simulator.py
```

Keep the simulator open for the entire lesson. Before each new test:

1. press **Pause** if the simulation is running;
2. select the required controller and road scenario;
3. enter the specified parameters;
4. press **Reset**;
5. state your prediction;
6. press **Start**;
7. allow the experiment to reach the end before recording the metrics.

!!! warning "Reset between experiments"
    If the simulator is not reset, the next experiment may begin with the speed,
    actuator state or integral value left by the previous experiment. The
    comparison would then be invalid.

## Model used in the exercises

The simulator uses a simplified one-dimensional vehicle model:

$$
m\frac{dv}{dt}
=
F_\mathrm{drive}
-F_\mathrm{roll}
-F_\mathrm{drag}
-F_\mathrm{hill}.
$$

The drive force is determined by the dimensionless command $u$:

$$
F_\mathrm{drive}=uF_\mathrm{max},
\qquad -1\leq u\leq1.
$$

For positive $u$, the vehicle generates drive force. A negative value represents
braking. The Day 1 calculations use:

| Parameter | Symbol | Value | Unit |
|---|---:|---:|---|
| Vehicle mass | $m$ | $1200$ | $\mathrm{kg}$ |
| Maximum drive force | $F_\mathrm{max}$ | $4500$ | $\mathrm{N}$ |
| Rolling resistance | $F_\mathrm{roll}$ | $180$ | $\mathrm{N}$ |
| Quadratic drag coefficient | $c$ | $4$ | $\mathrm{N/(m/s)^2}$ |
| Actuator time constant | $\tau$ | $0.35$ | $\mathrm{s}$ |
| Target speed | $v_\mathrm{ref}$ | $15$ | $\mathrm{m/s}$ |

Aerodynamic resistance is represented by:

$$
F_\mathrm{drag}=cv^2.
$$

The unit of $c$ follows directly from this equation:

$$
[c]
=
\frac{\mathrm{N}}{(\mathrm{m/s})^2}
=
\mathrm{\frac{N\,s^2}{m^2}}
=
\mathrm{\frac{kg}{m}}.
$$

Therefore, the value used here can be written equivalently as:

$
c=4\ \mathrm{N/(m/s)^2}=4\ \mathrm{kg/m}.
$

Do not confuse $c$ with $C_D$. The parameter $c$ in this simplified model is **not** the dimensionless aerodynamic drag coefficient $C_D$. In a more detailed model, $F_\mathrm{drag}=\frac{1}{2}\rho C_D A v^2$, so the simplified coefficient combines several physical quantities:
$$
c=\frac{1}{2}\rho C_D A.
$$



## Exercise 1 — Find the open-loop operating point

- **Time:** 0–10 minutes
- **Scenario:** Open loop, flat road
- **Question:** What constant command should maintain $15\ \mathrm{m/s}$?

### Step 1: calculate before running

At constant speed, acceleration is zero:

$$
\frac{dv}{dt}=0.
$$

The net force must therefore also be zero:

$$
F_\mathrm{drive}
-F_\mathrm{roll}
-F_\mathrm{drag}=0.
$$

At $v=15\ \mathrm{m/s}$, calculate the drag force:

$$
F_\mathrm{drag}=cv^2
=4(15)^2
=900\ \mathrm{N}.
$$

The drive force required to balance rolling resistance and drag is:

$$
F_\mathrm{drive,eq}
=180+900
=1080\ \mathrm{N}.
$$

Since $F_\mathrm{drive}=uF_\mathrm{max}$:

$$
u_\mathrm{eq}
=\frac{1080}{4500}
=0.24.
$$

This means that the simplified vehicle needs a command of approximately
**24% of maximum drive force** to maintain $15\ \mathrm{m/s}$ on a flat road.

### Step 2: predict

Before starting the simulator, complete:

| Command | Predicted final speed | Reason |
|---:|---:|---|
| $u=0.18$ | | |
| $u=0.24$ | | |
| $u=0.30$ | | |

Do not calculate every final speed yet. First decide whether each speed should
be below, approximately equal to, or above $15\ \mathrm{m/s}$.

### Step 3: run the experiment

1. Select **Open loop**.
2. Select **Flat road** or set the hill disturbance to zero.
3. Set the initial speed to $15\ \mathrm{m/s}$ if that option is available.
   Otherwise, allow the vehicle to accelerate from rest.
4. Test $u=0.18$, $u=0.24$ and $u=0.30$.
5. Reset before every run.
6. Record the final speed and whether the vehicle was still accelerating at the
   end.

| Command $u$ | Predicted relation to $15\ \mathrm{m/s}$ | Measured final speed [$\mathrm{m/s}$] | Final acceleration approximately zero? |
|---:|---|---:|---|
| $0.18$ | | | |
| $0.24$ | | | |
| $0.30$ | | | |

### Step 4: interpret

Answer together:

1. Why does a constant positive command not produce constant acceleration
   forever?
2. Why is $u=0.24$ an equilibrium command rather than an acceleration command?
3. Does the actuator time constant change the theoretical equilibrium speed, or
   only how quickly the drive force develops?
4. Why would the required command change if the vehicle had a larger frontal
   area or a larger aerodynamic drag coefficient?

??? success "Expected check"
    The command $u=0.24$ produces:

    $$
    F_\mathrm{drive}=0.24(4500)=1080\ \mathrm{N},
    $$

    which balances $180\ \mathrm{N}$ rolling resistance and
    $900\ \mathrm{N}$ drag at $15\ \mathrm{m/s}$. A lower command produces a
    lower equilibrium speed; a higher command produces a higher equilibrium
    speed. The actuator time constant changes the transient response but not the
    final force balance.

---

## Exercise 2 — Tune a P controller on a flat road

- **Time:** 10–20 minutes
- **Scenario:** P controller, flat road
- **Question:** Which proportional gain gives the best response while respecting
  the requirements?

The P controller is:

$$
e(t)=v_\mathrm{ref}(t)-v(t),
$$

$$
u_\mathrm{raw}(t)=K_Pe(t),
$$

$$
u(t)=\operatorname{clip}\left(u_\mathrm{raw}(t),-1,1\right).
$$

Because $u$ is dimensionless and $e$ has unit $\mathrm{m/s}$:

$$
[K_P]=\frac{1}{\mathrm{m/s}}=\mathrm{s/m}.
$$

### Requirements

Use the following provisional commissioning requirements:

| Metric | Requirement |
|---|---:|
| Rise time to 90% of target | $<8\ \mathrm{s}$ |
| Overshoot | $<10\%$ |
| Final speed error | As small as possible |
| Saturation | No unnecessary prolonged saturation |

### Step 1: calculate the initial controller request

For a start from rest:

$$
e(0)=15-0=15\ \mathrm{m/s}.
$$

Calculate the raw initial command for the candidate gains:

| $K_P$ [$\mathrm{s/m}$] | $u_\mathrm{raw}(0)=K_Pe(0)$ | Applied $u(0)$ after clipping | Initially saturated? |
|---:|---:|---:|---|
| $0.03$ | | | |
| $0.10$ | | | |
| $0.35$ | | | |

The actuator cannot apply a command larger than $1$. Therefore, two different
gains may request very different raw commands but initially produce the same
physical command.

### Step 2: predict

Rank the gains from 1 to 3 for:

- fastest expected rise;
- smallest expected steady-state error;
- least expected saturation;
- best overall compromise.

Write the ranking before starting the runs.

### Step 3: run and measure

1. Select **P** and **Flat road**.
2. Set $v_\mathrm{ref}=15\ \mathrm{m/s}$.
3. Run $K_P=0.03$, $0.10$ and $0.35$.
4. Reset after changing the gain.
5. Record the displayed metrics or read them from the plots.

| $K_P$ | Rise time [$\mathrm{s}$] | Overshoot [%] | Final error [$\mathrm{m/s}$] | Saturation [% or duration] | Passes rise-time requirement? |
|---:|---:|---:|---:|---:|---|
| $0.03$ | | | | | |
| $0.10$ | | | | | |
| $0.35$ | | | | | |

If a response never reaches 90% of the target, write **not reached** rather than
inventing a rise time.

### Step 4: explain the steady-state error

At equilibrium, a nonzero command is still needed to balance resistance. For a
pure P controller:

$$
u_\mathrm{eq}=K_Pe_\mathrm{ss}.
$$

Therefore:

$$
e_\mathrm{ss}=\frac{u_\mathrm{eq}}{K_P}.
$$

This simplified expression explains the trend: increasing $K_P$ reduces the
error needed to generate the required command. It is only an approximation
because drag depends on the actual final speed.

Discuss:

1. Why does the P controller not reach exactly $15\ \mathrm{m/s}$?
2. Why does increasing $K_P$ reduce final error?
3. Why does increasing $K_P$ eventually give diminishing improvement?
4. Is the gain with the smallest final error automatically the best choice?

??? success "Expected trend"
    A very low gain should produce little or no saturation but a slow response
    and a large final error. A larger gain should respond faster and reduce the
    final error, but it requests saturation for longer. With the standard model,
    $K_P=0.35\ \mathrm{s/m}$ is a reasonable candidate for the next exercise,
    although it still retains a nonzero steady-state error.

---

## Exercise 3 — Test the P controller on a $5^\circ$ uphill road

- **Time:** 20–30 minutes
- **Scenario:** P + hill
- **Question:** Can the selected P controller maintain the reference speed under
  a constant road-load disturbance?

### What does a $5^\circ$ slope mean?

The weight of the vehicle acts vertically:

$$
F_g=mg.
$$

On an inclined road, the component of the weight parallel to the road is:

$$
F_\mathrm{hill}=mg\sin\theta.
$$

For an uphill road, this force opposes forward motion. It is therefore
subtracted in the vehicle equation.

For $m=1200\ \mathrm{kg}$ and $\theta=5^\circ$:

$$
F_\mathrm{hill}
=1200(9.81)\sin(5^\circ)
\approx1026\ \mathrm{N}.
$$

Road signs normally specify grade in percent:

$$
\text{grade}[\%]=100\tan\theta.
$$

Thus:

$$
100\tan(5^\circ)\approx8.75\%.
$$

A $5^\circ$ angle is therefore approximately an **8.75% road grade**.

!!! note "Force input or angle input"
    If the GUI requests **hill angle**, enter $5^\circ$. If it requests a
    constant **hill force**, enter $1026\ \mathrm{N}$. These describe the same
    slope for the $1200\ \mathrm{kg}$ vehicle used here.

### Step 1: calculate the required equilibrium command

At $15\ \mathrm{m/s}$ on the slope:

$$
F_\mathrm{required}
=F_\mathrm{roll}+F_\mathrm{drag}+F_\mathrm{hill}.
$$

Therefore:

$$
F_\mathrm{required}
=180+4(15)^2+1026
=2106\ \mathrm{N}.
$$

The required normalized command is:

$$
u_\mathrm{hill}
=\frac{2106}{4500}
\approx0.468.
$$

Compare this with the flat-road value:

$$
u_\mathrm{flat}=0.24.
$$

The slope nearly doubles the command required to maintain the target speed.

### Step 2: predict the P-controller response

Use your selected $K_P$ from Exercise 2. If the group selected another value,
also perform the calculation with $K_P=0.35\ \mathrm{s/m}$ for comparison.

Estimate the steady-state error:

$$
e_\mathrm{ss}\approx\frac{u_\mathrm{hill}}{K_P}.
$$

For $K_P=0.35\ \mathrm{s/m}$:

$$
e_\mathrm{ss}\approx\frac{0.468}{0.35}
\approx1.34\ \mathrm{m/s}.
$$

The approximate final speed is therefore:

$$
v_\mathrm{ss}\approx15-1.34=13.66\ \mathrm{m/s}.
$$

This is an approximation because the drag force at $13.66\ \mathrm{m/s}$ is
smaller than the drag force at $15\ \mathrm{m/s}$.

### Step 3: run and measure

1. Select **P + hill**.
2. Use the same $K_P$ selected in Exercise 2.
3. Set the hill to $5^\circ$ or $1026\ \mathrm{N}$.
4. If the simulator allows it, make the hill begin at $t=15\ \mathrm{s}$ so
   that flat-road and hill behaviour are visible in one run.
5. Record the speed just before the hill, the minimum speed after entering the
   hill, and the final speed.

| Quantity | Prediction | Measurement |
|---|---:|---:|
| Speed immediately before hill [$\mathrm{m/s}$] | | |
| Minimum speed after hill begins [$\mathrm{m/s}$] | | |
| Final speed [$\mathrm{m/s}$] | | |
| Final error [$\mathrm{m/s}$] | | |
| Final command $u$ | | |

Calculate the difference between predicted and measured final speed:

$$
\Delta v
=
v_\mathrm{measured}-v_\mathrm{predicted}.
$$

### Step 4: interpret

Answer:

1. Did the P controller notice and react to the slope? What is the evidence?
2. Why did the command increase when the speed decreased?
3. Why can the vehicle still retain a nonzero final error even though feedback
   is active?
4. Is the remaining error caused by insufficient maximum drive force?
5. How could you distinguish actuator saturation from the normal steady-state
   limitation of a P controller?

??? success "Expected check"
    The target is physically feasible because the required command is only
    approximately $0.468<1$. The P controller should increase its command after
    the speed falls, but it needs a nonzero error to produce that command. With
    $K_P=0.35\ \mathrm{s/m}$, the standard model settles near
    $13.75\ \mathrm{m/s}$, corresponding to a final error of approximately
    $1.25\ \mathrm{m/s}$. This is a controller-structure limitation, not a lack
    of available drive force.

---

## Exercise 4 — Add integral action

- **Time:** 30–40 minutes
- **Scenario:** PI + hill
- **Question:** Can integral action remove the persistent hill error without
  creating an unacceptable transient response?

The PI controller is:

$$
u_\mathrm{raw}(t)
=
K_Pe(t)
+
K_I\int_0^t e(\tau)\,d\tau.
$$

The integral of speed error has unit:

$$
\left[\int e\,dt\right]
=
\mathrm{\frac{m}{s}}\mathrm{s}
=
\mathrm{m}.
$$

Since the command is dimensionless:

$$
[K_I]=\mathrm{m^{-1}}.
$$

### Step 1: predict

Keep $K_P$ fixed at the value selected in Exercise 2. Test:

$$
K_I\in\{0.02,\ 0.05,\ 0.10\}\ \mathrm{m^{-1}}.
$$

Before running, predict what happens as $K_I$ increases:

| Behaviour | Expected to increase, decrease or remain similar? | Reason |
|---|---|---|
| Persistent final error | | |
| Time needed to remove hill error | | |
| Overshoot risk | | |
| Integral accumulation | | |
| Saturation risk | | |

### Step 2: run a controlled comparison

1. Select **PI + hill**.
2. Use the same road, target speed, hill angle and $K_P$ as in Exercise 3.
3. Enable anti-windup if the GUI provides the option.
4. Test $K_I=0.02$, $0.05$ and $0.10\ \mathrm{m^{-1}}$.
5. Reset between tests so that the integral begins at zero.
6. Record results only after the same simulation duration.

| $K_P$ [$\mathrm{s/m}$] | $K_I$ [$\mathrm{m^{-1}}$] | Final error [$\mathrm{m/s}$] | Overshoot [%] | Time to recover after hill [$\mathrm{s}$] | Saturation | Pass/fail |
|---:|---:|---:|---:|---:|---|---|
| | $0.02$ | | | | | |
| | $0.05$ | | | | | |
| | $0.10$ | | | | | |

Use these requirements:

- final error below $0.2\ \mathrm{m/s}$;
- overshoot below $10\%$;
- no sustained saturation after reaching the target;
- no excessive acceleration after the disturbance changes.

### Step 3: explain why PI can remove the error

At steady speed on the hill, the vehicle requires approximately:

$$
u_\mathrm{hill}=0.468.
$$

With PI control, the stored integral term can provide most or all of this
command even when the current error becomes very small:

$$
u
=
\underbrace{K_Pe}_{\text{current error}}
+
\underbrace{K_I I}_{\text{stored correction}}.
$$

This is why the speed error can approach zero while the controller continues to
produce the nonzero command needed to balance the road load.

### Step 4: select and defend one configuration

Complete:

```text
Selected Kp:
Selected Ki:
Final error:
Overshoot:
Saturation observation:

We selected this controller because:

The most important improvement over P control was:

The trade-off introduced by integral action was:

One condition that still requires testing is:
```

??? success "Expected trend"
    A small positive $K_I$ should gradually remove the persistent hill error.
    Increasing $K_I$ generally removes the error faster, but overly aggressive
    integral action can increase overshoot, oscillation and windup risk. The
    preferred controller is the smallest integral gain that meets the stated
    requirements with acceptable transient behaviour.

---

## 40–45 min — Compare results and write a conclusion

Prepare to report four values:

1. the calculated flat-road open-loop command;
2. the selected $K_P$;
3. the measured P-controller hill error;
4. the selected $K_I$ and resulting PI-controller hill error.

### Final results sheet

| Item | Group result |
|---|---|
| Calculated flat-road equilibrium command | |
| Measured command/speed agreement | |
| Selected $K_P$ | |
| Reason for selecting $K_P$ | |
| Hill angle and equivalent grade | |
| Equivalent hill force | |
| P-controller final hill error | |
| Selected $K_I$ | |
| PI-controller final hill error | |
| Main PI trade-off | |
| One remaining limitation | |

### Required engineering conclusion

Write three to five sentences using measured values:

> On a flat road, the calculated command required to maintain
> $15\ \mathrm{m/s}$ was ____, while the simulator showed ____. We selected
> $K_P=$ ____ because ____. On the $5^\circ$ slope, P control produced a final
> error of ____ $\mathrm{m/s}$. Adding $K_I=$ ____ reduced the error to ____
> $\mathrm{m/s}$, but caused ____.

Do not write only “PI was better.” State the measured improvement and its cost.

## If your group finishes early

Choose one extension. Do not change several variables simultaneously.

### Extension A — Find the slope limit

At $15\ \mathrm{m/s}$ and full drive:

$$
F_\mathrm{hill,max}
=F_\mathrm{max}-F_\mathrm{roll}-cv^2.
$$

Calculate the maximum opposing hill force and convert it into:

1. slope angle $\theta$;
2. road grade in percent.

Then test a slightly smaller and a slightly larger slope in the simulator. One
should be feasible; the other should be physically impossible at the target
speed.

### Extension B — Test model mismatch

Increase vehicle mass by 25% while keeping the road angle fixed.

Predict and test:

- whether the flat-road equilibrium command changes;
- whether the hill force changes;
- whether the rise time changes;
- whether the previously selected PI gains still meet the requirements.

Remember that, for a fixed road angle:

$$
F_\mathrm{hill}=mg\sin\theta,
$$

so changing mass also changes the gravitational road load.

### Extension C — Investigate anti-windup

Choose a slope steep enough that $15\ \mathrm{m/s}$ becomes temporarily
unreachable. Compare PI control with anti-windup enabled and disabled.

Observe:

- accumulated integral error during saturation;
- command immediately after the slope ends;
- peak speed after the slope;
- time required to return to the target band.

Explain why poor recovery after the hill can be a windup problem even when the
loss of speed during the hill is caused by a physical force limit.

## Troubleshooting

### The vehicle does not move

- Check that the simulation is running rather than paused.
- Confirm that the target speed is positive.
- For open loop, confirm that the command is positive.
- Press **Reset** after changing the scenario.

### Every gain initially gives the same response

Check the raw and applied commands. Several gains may all request $u>1$ at the
start, so the actuator applies the same saturated command $u=1$. The responses
separate only after the error decreases enough to leave saturation.

### PI behaves differently after repeated runs

The integral state may not have been cleared. Press **Reset** before every run.

### The hill is specified in newtons rather than degrees

Use:

$$
F_\mathrm{hill}=mg\sin\theta.
$$

For this lesson, enter approximately $1026\ \mathrm{N}$ for a $5^\circ$ uphill
slope and a $1200\ \mathrm{kg}$ vehicle.

### The measured value differs from the hand calculation

Check:

- whether the response actually reached steady state;
- whether the hill was active;
- whether you used actual speed rather than target speed in $cv^2$;
- whether actuator lag affected only the transient;
- whether the command was saturated;
- whether the GUI and calculation use the same vehicle parameters.

The purpose is not to force the simulation to match an incorrect calculation.
The purpose is to identify which assumption explains the difference.

## Takeaway

The four exercises establish a complete control-engineering chain:

- force balance predicts the open-loop operating point;
- proportional feedback improves the transient response but needs error to
  generate a nonzero steady command;
- a road slope adds the opposing force $mg\sin\theta$;
- integral action can supply the steady correction while the current error
  approaches zero;
- actuator limits still determine whether the target is physically feasible.

In Lesson 5, the same longitudinal-control ideas will be implemented in a more
realistic vehicle simulation environment.
