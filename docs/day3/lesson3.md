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
