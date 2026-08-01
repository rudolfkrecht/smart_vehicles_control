# Lesson 3 — Numerical engineering exercises

- **Format:** Individual calculations followed by pair checking
- **Main outcome:** Calculate robustness metrics and justify pass/fail decisions

Use units in every answer. A number without a unit is incomplete unless the
quantity is dimensionless.

## Exercise 1: classify the fault

Match each observation to the most appropriate fault type.

| Observation | Noise, bias, delay, dropout or authority loss? |
|---|---|
| Range varies by approximately $\pm0.3$ m around the true value | |
| Steering is always about $1.5^\circ$ to the right of the request | |
| Correct pose arrives 0.25 s late | |
| No range samples arrive for 9 s | |
| A brake request of 0.8 produces only 58% of nominal force | |

??? success "Solution"
    | Observation | Type |
    |---|---|
    | range variation | noise |
    | persistent steering offset | bias |
    | old but correct pose | delay |
    | absent range samples | dropout |
    | weak physical response | authority loss |

## Exercise 2: path-error metrics

For:

$$
e_y=[0.20,\ -0.40,\ 0.10,\ 0.50,\ -0.30]\ \mathrm m,
$$

calculate:

1. mean signed error;
2. mean absolute error;
3. maximum absolute error;
4. RMSE.

Use:

$$
\bar e_y=\frac{1}{N}\sum e_{y,k},
$$

$$
\operatorname{MAE}=\frac{1}{N}\sum |e_{y,k}|,
$$

$$
\operatorname{RMSE}
=\sqrt{\frac{1}{N}\sum e_{y,k}^2}.
$$

Then run:

```bat
python courses\day_4\student\exercise_metrics.py
```

??? success "Solution"
    The signed sum is $0.10$ m:

    $$
    \bar e_y=0.10/5=0.020\ \mathrm m.
    $$

    The absolute sum is $1.50$ m:

    $$
    \operatorname{MAE}=1.50/5=0.300\ \mathrm m.
    $$

    $$
    e_{y,\max}=0.500\ \mathrm m.
    $$

    The squared sum is $0.55\ \mathrm{m^2}$:

    $$
    \operatorname{RMSE}
    =\sqrt{0.55/5}
    \approx0.332\ \mathrm m.
    $$

    The small signed mean is not evidence of accurate tracking because
    positive and negative errors cancel.

## Exercise 3: TTC and interpretation

For each case, calculate:

$$
v_{\mathrm{close}}=v-v_L,
\qquad
\mathrm{TTC}=\frac{d}{v_{\mathrm{close}}}
$$

when $v_{\mathrm{close}}>0$.

| Case | Gap $d$ | Ego speed $v$ | Lead speed $v_L$ | Closing speed | TTC |
|---|---:|---:|---:|---:|---:|
| A | 30 m | 14 m/s | 10 m/s | | |
| B | 20 m | 12 m/s | 8 m/s | | |
| C | 12 m | 8 m/s | 8 m/s | | |
| D | 15 m | 7 m/s | 9 m/s | | |

Which case has the smallest gap? Which case has the most urgent collision
trend? Why are those not necessarily the same question?

??? success "Solution"
    | Case | Closing speed | TTC |
    |---|---:|---:|
    | A | $4\ \mathrm{m/s}$ | $7.5$ s |
    | B | $4\ \mathrm{m/s}$ | $5.0$ s |
    | C | $0\ \mathrm{m/s}$ | infinite/not closing |
    | D | $-2\ \mathrm{m/s}$ | infinite/separating |

    Case C has the smallest gap. Case B has the smallest finite TTC and is
    the most urgent closing situation. Gap describes distance; TTC also
    includes relative motion.

## Exercise 4: braking-authority loss

The nominal braking model can provide approximately
$a_{\mathrm{nom}}=5.0\ \mathrm{m/s^2}$. Estimate the applied deceleration and
idealized stopping distance from $14\ \mathrm{m/s}$ for:

| Efficiency $\eta_b$ | Applied deceleration | Stopping distance |
|---:|---:|---:|
| 1.00 | | |
| 0.65 | | |
| 0.58 | | |

Use:

$$
a_{\mathrm{applied}}=\eta_ba_{\mathrm{nom}},
\qquad
d_{\mathrm{stop}}=\frac{v^2}{2a_{\mathrm{applied}}}.
$$

Ignore actuator delay and rolling resistance for this hand calculation.

??? success "Solution"
    | $\eta_b$ | $a_{\mathrm{applied}}$ | $d_{\mathrm{stop}}$ |
    |---:|---:|---:|
    | 1.00 | $5.00\ \mathrm{m/s^2}$ | $19.6$ m |
    | 0.65 | $3.25\ \mathrm{m/s^2}$ | $30.2$ m |
    | 0.58 | $2.90\ \mathrm{m/s^2}$ | $33.8$ m |

    Authority loss increases stopping distance nonlinearly because distance
    depends on $1/a$. A real result would also include response delay and a
    more detailed brake/tire model.

## Exercise 5: sensor-age supervisor

The latest valid range sample arrived at $t_{\mathrm{last}}=34.00$ s. The
controller accepts a measurement only while:

$$
t-t_{\mathrm{last}}\le0.25\ \mathrm s.
$$

Complete:

| Current time | Age | Healthy for normal ACC? | Requested mode |
|---:|---:|---|---|
| 34.10 s | | | |
| 34.25 s | | | |
| 34.27 s | | | |
| 38.00 s | | | |

The Day 4 simulator additionally provides a direct health flag. The supervisor
activates when either the flag is false or the age is too large:

```python
not range_sensor_healthy or range_measurement_age > 0.25
```

??? success "Solution"
    | Time | Age | Normal ACC? | Mode |
    |---:|---:|---|---|
    | 34.10 s | 0.10 s | yes | ACC mode |
    | 34.25 s | 0.25 s | yes | ACC mode |
    | 34.27 s | 0.27 s | no | `SAFE_STOP` |
    | 38.00 s | 4.00 s | no | `SAFE_STOP` |

    In the packaged dropout scenario the health flag becomes false
    immediately at 34 s, so the fallback does not wait 0.25 s.

## Exercise 6: jerk and comfort

Acceleration samples are:

$$
a=[0.0,\ 0.8,\ 1.0,\ 0.3,\ -0.5]\ \mathrm{m/s^2}
$$

with $\Delta t=0.5$ s. Calculate:

$$
j_k=\frac{a_k-a_{k-1}}{\Delta t}.
$$

Then calculate RMS jerk:

$$
j_{\mathrm{RMS}}
=\sqrt{\frac{1}{N-1}\sum_{k=2}^{N}j_k^2}.
$$

??? success "Solution"
    The jerk sequence is:

    $$
    j=[1.6,\ 0.4,\ -1.4,\ -1.6]\ \mathrm{m/s^3}.
    $$

    $$
    j_{\mathrm{RMS}}
    =\sqrt{\frac{1.6^2+0.4^2+(-1.4)^2+(-1.6)^2}{4}}
    \approx1.35\ \mathrm{m/s^3}.
    $$

    A fault response may accept temporarily higher jerk to preserve safety,
    but the trade-off should still be measured and reported.

## Exercise 7: pass/fail hierarchy

Evaluate the following candidates:

| Candidate | Collision | Road departure | Min gap | Max lane error | Progress |
|---|---:|---:|---:|---:|---:|
| A | 0 | 0% | 2.5 m | 0.20 m | 112% |
| B | 0 | 0% | 7.0 m | 1.45 m | 101% |
| C | 0 | 0% | 8.0 m | 0.30 m | 88% |
| D | 1 sample | 0% | $-0.1$ m | 0.10 m | 130% |

Course criteria:

- collision samples $=0$;
- road departure $=0\%$;
- minimum gap $>3.0$ m;
- maximum lane error $\le1.60$ m;
- progress $\ge95\%$.

Which pass? Which appears fastest but must be rejected?

??? success "Solution"
    - A fails minimum gap.
    - B passes.
    - C fails progress.
    - D fails collision and minimum gap.

    D has the greatest progress but must be rejected first. Safety gates
    prevent an unsafe result from winning through efficiency.

## Submission

Keep:

1. completed tables;
2. units for every calculation;
3. one sentence explaining why averages and extrema are both required;
4. one sentence explaining why a supervisor can reduce nominal performance.

## Fast-finisher extension

At $14\ \mathrm{m/s}$, compare desired ACC gaps for $d_0=6$ m and
$T_h\in\{1.0,1.7,2.2\}$ s. Discuss whether a larger headway can compensate for
all possible braking faults.
