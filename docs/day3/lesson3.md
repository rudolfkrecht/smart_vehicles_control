# Lesson 3 — Numerical engineering exercises

- **Format:** Individual calculations followed by comparison
- **Main outcome:** Calculate the signals and commands used by the Day 3 ACC
  controller

Use SI units throughout. Show the equation, substitute values with units and
state what the result means physically.

## Data sheet

Unless an exercise changes a value, use:

| Parameter | Value |
|---|---:|
| Cruise target $v_{\mathrm{cruise}}$ | $15\ \mathrm{m/s}$ |
| Standstill gap $d_0$ | $5\ \mathrm m$ |
| Time headway $T_h$ | $1.5\ \mathrm s$ |
| Gap gain $K_d$ | $0.20\ \mathrm{s^{-1}}$ |
| Closing-speed gain $K_{\Delta v}$ | $0.50$ |
| PI proportional gain $K_P$ | $0.15\ \mathrm{s/m}$ |
| PI integral gain $K_I$ | $0.005\ \mathrm{1/m}$ |

The normalized PI output is positive for throttle and negative for brake.

## Exercise 1: desired following distance

Calculate the desired gap at:

1. $v=0\ \mathrm{m/s}$;
2. $v=5\ \mathrm{m/s}$;
3. $v=10\ \mathrm{m/s}$;
4. $v=15\ \mathrm{m/s}$;
5. $v=20\ \mathrm{m/s}$.

Use:

$$
d_{\mathrm{des}}=d_0+T_hv.
$$

| Ego speed | Desired gap |
|---:|---:|
| 0 m/s | |
| 5 m/s | |
| 10 m/s | |
| 15 m/s | |
| 20 m/s | |

Then explain why $T_h$ has units of seconds.

??? success "Worked solution"
    $$
    d_{\mathrm{des}}=5+1.5v.
    $$

    | Ego speed | Desired gap |
    |---:|---:|
    | 0 m/s | 5.0 m |
    | 5 m/s | 12.5 m |
    | 10 m/s | 20.0 m |
    | 15 m/s | 27.5 m |
    | 20 m/s | 35.0 m |

    Since $T_hv$ must be a distance:

    $$
    [T_h]=\frac{\mathrm m}{\mathrm{m/s}}=\mathrm s.
    $$

## Exercise 2: closing speed and TTC

For each case, calculate closing speed and TTC.

| Case | Gap $d$ | Ego $v$ | Lead $v_L$ |
|---|---:|---:|---:|
| A | 30 m | 15 m/s | 10 m/s |
| B | 30 m | 10 m/s | 10 m/s |
| C | 18 m | 12 m/s | 9 m/s |
| D | 12 m | 8 m/s | 11 m/s |

Use:

$$
v_{\mathrm{close}}=v-v_L,
$$

and, only when $v_{\mathrm{close}}>0$:

$$
\mathrm{TTC}=\frac{d}{v_{\mathrm{close}}}.
$$

??? success "Worked solution"
    | Case | Closing speed | TTC | Interpretation |
    |---|---:|---:|---|
    | A | $5\ \mathrm{m/s}$ | 6.0 s | Gap decreasing |
    | B | $0\ \mathrm{m/s}$ | $\infty$ | Constant gap |
    | C | $3\ \mathrm{m/s}$ | 6.0 s | Gap decreasing |
    | D | $-3\ \mathrm{m/s}$ | $\infty$ | Lead pulling away |

    Cases A and C have the same TTC even though their gaps and closing speeds
    differ.

## Exercise 3: ACC target

The ego vehicle has:

- $v=15\ \mathrm{m/s}$;
- $v_L=10\ \mathrm{m/s}$;
- $d=28\ \mathrm m$.

Calculate:

1. desired gap;
2. gap error $d-d_{\mathrm{des}}$;
3. closing speed;
4. the unclipped ACC target;
5. the selected target after clipping to $[0,15]\ \mathrm{m/s}$.

Use:

$$
v_{\mathrm{ACC}}
=v_L+K_d(d-d_{\mathrm{des}})
-K_{\Delta v}\max(v-v_L,0).
$$

Check the unit of the gap correction:

$$
\mathrm{s^{-1}}\cdot\mathrm m=\mathrm{m/s}.
$$

??? success "Worked solution"
    Desired gap:

    $$
    d_{\mathrm{des}}=5+1.5(15)=27.5\ \mathrm m.
    $$

    Gap error:

    $$
    28-27.5=0.5\ \mathrm m.
    $$

    Closing speed:

    $$
    15-10=5\ \mathrm{m/s}.
    $$

    ACC target:

    $$
    v_{\mathrm{ACC}}
    =10+0.20(0.5)-0.50(5)
    =7.6\ \mathrm{m/s}.
    $$

    The value is already inside the allowed range, so the selected target is
    $7.6\ \mathrm{m/s}$. The target is below lead speed because the ego car is
    still closing rapidly.

## Exercise 4: target arbitration

Select the minimum active target:

$$
v_{\mathrm{selected}}
=\min(v_{\mathrm{cruise}},v_{\mathrm{road}},v_{\mathrm{ACC}}).
$$

| Case | Cruise | Road | ACC | Selected |
|---|---:|---:|---:|---:|
| A | 15 | 15 | 7.6 | |
| B | 15 | 8 | 12 | |
| C | 15 | 15 | 15 | |
| D | 12 | 18 | 14 | |

All speeds are in m/s.

??? success "Worked solution"
    | Case | Selected target | Active constraint |
    |---|---:|---|
    | A | 7.6 m/s | Traffic |
    | B | 8 m/s | Road |
    | C | 15 m/s | All equal/inactive |
    | D | 12 m/s | Cruise setting |

## Exercise 5: PI and actuator split

Continue Exercise 3. The ego speed is $15\ \mathrm{m/s}$ and the selected
target is $7.6\ \mathrm{m/s}$. Assume the accumulated speed error is:

$$
\int e_v\,dt=6\ \mathrm{m}.
$$

Calculate:

1. speed error;
2. raw PI output;
3. saturated signed command;
4. throttle and brake commands.

Use:

$$
u=K_Pe_v+K_I\int e_v\,dt.
$$

??? success "Worked solution"
    Speed error:

    $$
    e_v=7.6-15=-7.4\ \mathrm{m/s}.
    $$

    Raw command:

    $$
    u=0.15(-7.4)+0.005(6)=-1.08.
    $$

    After saturation:

    $$
    u_{\mathrm{sat}}=-1.
    $$

    Therefore:

    ```text
    throttle = 0
    brake = 1
    ```

    The vehicle must never apply positive throttle and brake simultaneously.

## Exercise 6: emergency braking margin

A lead vehicle is stopped. The ego speed is $12\ \mathrm{m/s}$. Assume the
effective maximum deceleration is $6\ \mathrm{m/s^2}$ and ignore delay.

Estimate ideal braking distance:

$$
d_{\mathrm{brake}}=\frac{v^2}{2a}.
$$

Then add:

- 0.5 s of sensing and actuation delay;
- a 3 m residual gap.

Use:

$$
d_{\mathrm{total}}
=vT_{\mathrm{delay}}+\frac{v^2}{2a}+d_{\mathrm{residual}}.
$$

Compare the result with a 15 m measured gap.

??? success "Worked solution"
    Ideal braking distance:

    $$
    d_{\mathrm{brake}}
    =\frac{12^2}{2(6)}
    =12\ \mathrm m.
    $$

    Delay distance:

    $$
    12(0.5)=6\ \mathrm m.
    $$

    Total:

    $$
    d_{\mathrm{total}}=6+12+3=21\ \mathrm m.
    $$

    A 15 m gap is insufficient under these assumptions. This shows why a
    fixed TTC or gap threshold cannot be selected without considering delay
    and braking capability.

## Engineering conclusion

Complete:

```text
The desired gap at 15 m/s is __________.
The ACC target depends on both gap error and __________.
The lowest of road and traffic targets is selected because __________.
The emergency-distance calculation is optimistic because it omits __________.
```

## Additional challenge

Find a pair $(d,v-v_L)$ that gives TTC $=5$ s. Then find a different pair with
the same TTC. Explain why equal TTC does not imply equal desired gap, required
braking or comfort.

# Block 3 — Advanced worked examples


- **Main outcome:** Analyse the complete controller at a difficult operating
  point and justify a design decision from more than one metric


## Reference equations

$$
d_{\mathrm{des}}=d_0+T_hv
$$

$$
\mathrm{TTC}=\frac{d}{v-v_L}\qquad(v>v_L)
$$

$$
v_{\mathrm{ACC}}=
\operatorname{clip}\left[
v_L+K_d(d-d_{\mathrm{des}})
-K_{\Delta v}\max(v-v_L,0),0,v_{\mathrm{cruise}}
\right]
$$

$$
v_{\mathrm{curve}}=
\sqrt{\frac{a_{y,\max}}{\max(|\kappa|,\varepsilon)}}
$$

$$
v_{\mathrm{selected}}
=\min(v_{\mathrm{cruise}},v_{\mathrm{curve}},v_{\mathrm{ACC}})
$$

## Case 1 — One sample, three constraints and a saturated PI controller

At the current sample:

| Quantity | Value |
|---|---:|
| Ego speed $v$ | $16.0\ \mathrm{m/s}$ |
| Lead speed $v_L$ | $11.0\ \mathrm{m/s}$ |
| Gap $d$ | $32.0\ \mathrm m$ |
| Cruise request | $18.0\ \mathrm{m/s}$ |
| Previewed curvature $\|\kappa\|$ | $0.0225\ \mathrm{m^{-1}}$ |
| $d_0$, $T_h$ | $5.0\ \mathrm m$, $1.6\ \mathrm s$ |
| $K_d$, $K_{\Delta v}$ | $0.22\ \mathrm{s^{-1}}$, $0.65$ |
| $a_{y,\max}$ | $2.5\ \mathrm{m/s^2}$ |
| PI gains | $K_P=0.15$, $K_I=0.005\ \mathrm{s^{-1}}$ |
| Previous integral | $I_{k-1}=12.0\ \mathrm{m}$ |
| Sample time | $\Delta t=0.10\ \mathrm s$ |

Calculate:

1. desired gap;
2. closing speed and TTC;
3. ACC target;
4. curve-speed target;
5. selected speed and active limiting constraint;
6. tentative integral $I_k$;
7. raw and clipped signed PI command;
8. expected behaviour state.

Then answer: if the curve target were calculated from the current curvature
rather than previewed curvature, which physical response could become late?

??? success "Case 1 — worked solution"
    Desired gap:

    $$
    d_{\mathrm{des}}=5+1.6(16)=30.6\ \mathrm m.
    $$

    Closing speed and TTC:

    $$
    v-v_L=5.0\ \mathrm{m/s},\qquad
    \mathrm{TTC}=\frac{32}{5}=6.4\ \mathrm s.
    $$

    ACC target:

    $$
    v_{\mathrm{ACC}}
    =11+0.22(32-30.6)-0.65(5)
    =8.058\ \mathrm{m/s}.
    $$

    Curve target:

    $$
    v_{\mathrm{curve}}
    =\sqrt{\frac{2.5}{0.0225}}
    =10.54\ \mathrm{m/s}.
    $$

    Therefore traffic is the active constraint and

    $$
    v_{\mathrm{selected}}=8.058\ \mathrm{m/s}.
    $$

    The speed error is $-7.942\ \mathrm{m/s}$ and

    $$
    I_k=12+(-7.942)(0.1)=11.206\ \mathrm m.
    $$

    $$
    u_{\mathrm{raw}}
    =0.15(-7.942)+0.005(11.206)
    \approx-1.135.
    $$

    The normalized command clips to $-1.0$, requesting full braking. The state
    should be `BRAKE` because the selected target is more than $1\ \mathrm{m/s}$
    below the current speed, even though TTC is not yet critical. Without
    curvature preview, braking for the bend would begin later.

## Case 2 — Is collision avoidance physically feasible after brake fade?

Do not start with the ACC target. First check whether the available distance is
consistent with the assumed latency and braking authority.

Assume:

- $v=18\ \mathrm{m/s}$;
- $v_L=10\ \mathrm{m/s}$;
- measured true gap $d=42\ \mathrm m$;
- sensing + computation + actuation latency $\tau=0.35\ \mathrm s$;
- nominal ego deceleration $a_E=6.0\ \mathrm{m/s^2}$;
- braking efficiency $\eta_B=0.60$;
- lead deceleration $a_L=5.0\ \mathrm{m/s^2}$;
- final residual margin $d_0=5\ \mathrm m$.

Use the simplified worst-case requirement:

$$
d_{\mathrm{req}}
=v\tau
+\frac{v^2}{2\eta_Ba_E}
-\frac{v_L^2}{2a_L}
+d_0.
$$

Tasks:

1. calculate the effective ego deceleration;
2. calculate latency distance and both braking distances;
3. determine whether $42$ m is sufficient;
4. solve the same expression for the maximum feasible ego speed;
5. name three physical effects omitted by this check.

??? success "Case 2 — worked solution"
    Effective ego deceleration is

    $$
    \eta_Ba_E=0.60(6.0)=3.6\ \mathrm{m/s^2}.
    $$

    The four distance terms are:

    $$
    v\tau=6.3\ \mathrm m,
    \quad \frac{18^2}{2(3.6)}=45.0\ \mathrm m,
    \quad \frac{10^2}{2(5)}=10.0\ \mathrm m,
    \quad d_0=5.0\ \mathrm m.
    $$

    Thus

    $$
    d_{\mathrm{req}}=6.3+45-10+5=46.3\ \mathrm m.
    $$

    The observed $42$ m gap is short by $4.3$ m under these assumptions.

    To find the maximum feasible speed, solve

    $$
    42=0.35v+\frac{v^2}{7.2}-10+5.
    $$

    The positive root is approximately

    $$
    v_{\max}=17.18\ \mathrm{m/s}.
    $$

    Omitted effects include braking build-up dynamics, slope, tire friction,
    aerodynamic/rolling forces, sensor error and variation in lead braking.

## Case 3 — Trace a stateful supervisor

The supervisor uses these priorities:

1. invalid/stale range data overrides all normal modes with `SAFE_STOP`;
2. `EMERGENCY` if $d\leq3.0$ m or TTC $\leq1.2$ s;
3. `BRAKE` if TTC $<2.5$ s, gap $<0.72d_{\mathrm{des}}$, or the selected target
   is more than $1\ \mathrm{m/s}$ below ego speed;
4. otherwise `FOLLOW` or `CRUISE`;
5. after a radar fault clears, remain in `SAFE_STOP` until health has been
   continuously valid for $1.0$ s.

Fill the last two columns:

| Sample | Health | Lead | $v$ | $v_L$ | $d$ | TTC | Healthy recovery time | State |
|---:|---|---|---:|---:|---:|---:|---:|---|
| A | valid | yes | 14 | 11 | 30 | 10.0 | — | |
| B | valid | yes | 13 | 8 | 20 | 4.0 | — | |
| C | valid | yes | 10 | 6 | 5 | 1.25 | — | |
| D | valid | yes | 8 | 5 | 2.8 | 0.93 | — | |
| E | failed | no | 6 | — | — | — | 0.0 | |
| F | valid | yes | 4 | 4 | 14 | $\infty$ | 0.4 | |
| G | valid | yes | 4 | 4 | 14 | $\infty$ | 1.1 | |

Explain why `lead_detected == False` is not a valid radar-failure test.

??? success "Case 3 — worked interpretation"
    A and B are `BRAKE`: their continuous ACC targets are far below the ego
    speeds. C is still `BRAKE` because TTC is just above the emergency
    threshold. D is `EMERGENCY`. E is `SAFE_STOP` and latches the supervisor.
    F remains `SAFE_STOP` because the confirmation interval is incomplete. At
    G the latch may clear and normal ACC gives `FOLLOW`.

    A healthy radar can report no lead because the road is genuinely empty or
    the target is outside the sensor range. Detection status and health status
    answer different questions.

## Case 4 — Diagnose candidates without hiding failures in a score

| Candidate | Cases passed | Minimum gap | Maximum path error | RMS jerk | Completion time |
|---|---:|---:|---:|---:|---:|
| A | 6/6 | 4.8 m | 1.20 m | 2.8 m/s³ | 39 s |
| B | 5/6 | 2.4 m | 0.60 m | 1.4 m/s³ | 34 s |
| C | 6/6 | 7.5 m | 0.85 m | 3.6 m/s³ | 43 s |

Requirements are: all cases pass, minimum gap $>3$ m, maximum path error
$\leq1.60$ m.

Tasks:

1. apply the safety gates before ranking;
2. determine whether A or C dominates the other;
3. select a candidate for a comfort-critical shuttle;
4. select a candidate for a time-critical closed-course demonstrator;
5. state which additional evidence could reverse each decision.

??? success "Case 4 — worked interpretation"
    Candidate B is rejected before scoring. Neither A nor C dominates the
    other: A is faster and smoother in jerk, while C has better gap and lateral
    margins. C is defensible for a safety-margin-focused shuttle, although its
    higher jerk needs investigation. A is defensible for the closed-course
    demonstrator if its smaller margins remain acceptable in further tests.

## Case 5 — Design the test that could disprove your conclusion

Choose the candidate selected in Case 4. Propose a four-run campaign that is
specifically likely to expose its weakness. Include:

- a baseline;
- one changed initial condition;
- one sensor or actuator fault;
- one combined case;
- a fixed random seed for pairwise comparison;
- one metric expected to fail first.

There is no single numerical solution. A strong answer creates a plausible
counterexample rather than another easy nominal run.

## Computational verification

Run the supplied checker after completing the calculations:

```bat
python courses\day_3\demos\lesson3_integrated_case_study.py
```

It prints the Case 1 constraint calculation, the Case 2 stopping envelope and
the positive root for maximum feasible speed. If your values differ, compare
units and signs before changing the equations.

## Fast-team investigation

Derive the maximum feasible ego-speed formula for Case 2 symbolically. Then
plot $v_{\max}$ for braking efficiency from 0.4 to 1.0 and latency from 0 to
0.6 s. Identify the region where the original 18 m/s approach is feasible.
