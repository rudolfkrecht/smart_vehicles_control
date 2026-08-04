# Lesson 1 — Theory: Adaptive Cruise Control

- **Format:** Theory with short concept checks
- **Main outcome:** Explain how lead-vehicle measurements modify the speed
  target of an already working vehicle controller

## 1. From cruise control to ACC

Conventional cruise control compares one requested speed with the measured ego
speed:

$$
e_v=v_{\mathrm{ref}}-v.
$$

A PI controller then produces a longitudinal command:

$$
u=K_Pe_v+K_I\int e_v\,dt.
$$

This works on an empty road, but it cannot react correctly to a slower lead
vehicle. If the requested speed remains $15\ \mathrm{m/s}$ while a car ahead
travels at $8\ \mathrm{m/s}$, ordinary cruise control continues trying to
accelerate.

Adaptive Cruise Control adds a target-selection layer before the PI
controller:

1. maintain the driver-selected speed when the road ahead is clear;
2. reduce the selected speed when a relevant lead vehicle is detected;
3. preserve a speed-dependent following distance;
4. brake more strongly when the gap or TTC becomes unsafe.

The existing PI controller is still useful. ACC changes its reference; it does
not need to replace the complete actuator-control layer.

## 2. Measurements

Use the following symbols:

| Symbol | Meaning | Unit |
|---|---|---:|
| $v$ | ego speed | m/s |
| $v_L$ | lead-vehicle speed | m/s |
| $d$ | bumper-to-bumper gap | m |
| $d_0$ | standstill gap | m |
| $T_h$ | requested time headway | s |
| $v-v_L$ | closing speed | m/s |
| TTC | time to collision when closing | s |

The sign of closing speed matters:

- $v-v_L>0$: the ego car is closing the gap;
- $v-v_L=0$: both vehicles have the same speed;
- $v-v_L<0$: the gap is increasing.

### Concept check

The ego car travels at $14\ \mathrm{m/s}$ and the lead car at
$10\ \mathrm{m/s}$.

$$
v_{\mathrm{close}}=14-10=4\ \mathrm{m/s}.
$$

The ego car reduces the gap by approximately 4 m every second unless one of the
vehicles changes speed.

## 3. Constant-time-headway spacing

A fixed 10 m gap has very different meaning at 3 m/s and 25 m/s. The Day 3
controller therefore uses:

$$
d_{\mathrm{des}}=d_0+T_hv.
$$

The desired gap contains:

- a fixed standstill part $d_0$;
- a speed-dependent part $T_hv$.

For $d_0=5\ \mathrm m$ and $T_h=1.5\ \mathrm s$:

| Ego speed | Desired gap |
|---:|---:|
| 0 m/s | 5.0 m |
| 5 m/s | 12.5 m |
| 10 m/s | 20.0 m |
| 15 m/s | 27.5 m |

Unit check:

$$
[T_hv]=\mathrm{s}\frac{\mathrm m}{\mathrm s}=\mathrm m.
$$

Therefore, the two terms in the desired-gap equation both have units of
metres.

## 4. Time to collision

When the ego car is closing:

$$
\mathrm{TTC}=\frac{d}{v-v_L}.
$$

For a gap of 24 m and a closing speed of 4 m/s:

$$
\mathrm{TTC}=\frac{24}{4}=6\ \mathrm s.
$$

If $v\leq v_L$, the gap is not closing and the simulator reports infinite TTC.

TTC is useful, but it is not a complete safety measure:

- a short but constant gap gives infinite TTC;
- TTC does not directly include the available braking force;
- noisy relative-speed measurement can make TTC vary strongly;
- a safe threshold depends on delay, road friction and vehicle dynamics.

For this reason, the teaching controller uses gap, desired gap and TTC
together.

## 5. Simplified ACC target

The continuous teaching rule is:

$$
v_{\mathrm{ACC}}=
\operatorname{clip}\left[
v_L+K_d(d-d_{\mathrm{des}})
-K_{\Delta v}\max(v-v_L,0),
0,\ v_{\mathrm{cruise}}
\right].
$$

Interpret each term:

- $v_L$ gives the controller a lead-speed baseline;
- $K_d(d-d_{\mathrm{des}})$ raises the target when extra space is available
  and lowers it when the gap is too small;
- $K_{\Delta v}\max(v-v_L,0)$ reduces the target when the ego vehicle is
  closing;
- clipping prevents negative targets and targets above the cruise setting.

Units:

$$
[K_d]=\mathrm{s^{-1}}
$$

because $K_d$ multiplies distance and must produce speed. The closing-speed
gain $K_{\Delta v}$ is dimensionless.

### Example

Suppose:

- $v=15\ \mathrm{m/s}$;
- $v_L=10\ \mathrm{m/s}$;
- $d=28\ \mathrm m$;
- $d_0=5\ \mathrm m$;
- $T_h=1.5\ \mathrm s$;
- $K_d=0.20\ \mathrm{s^{-1}}$;
- $K_{\Delta v}=0.50$.

First:

$$
d_{\mathrm{des}}=5+1.5(15)=27.5\ \mathrm m.
$$

Then:

$$
v_{\mathrm{ACC}}
=10+0.20(28-27.5)-0.50(15-10)
=7.6\ \mathrm{m/s}.
$$

The controller requests a speed below the lead speed because the ego car is
still closing at 5 m/s.

## 6. Selecting among constraints

The driver, road and traffic can all impose a target:

$$
v_{\mathrm{selected}}
=\min(v_{\mathrm{cruise}},v_{\mathrm{road}},v_{\mathrm{ACC}}).
$$

Examples:

| Cruise target | Road target | ACC target | Selected |
|---:|---:|---:|---:|
| 15 | 15 | 12 | 12 m/s |
| 15 | 8 | 13 | 8 m/s |
| 15 | 15 | 15 | 15 m/s |

The lowest active constraint wins. A clear lane ahead does not cancel a
low-speed curve, and a straight road does not cancel a close lead vehicle.

## 7. Behaviour labels and overrides

The simulator displays four labels:

| Mode | Meaning |
|---|---|
| Cruise | No relevant lead vehicle; use the driver/road target |
| Follow | A lead vehicle affects the selected target |
| Brake | The target has dropped strongly or the situation is becoming unsafe |
| Emergency | A critical gap or TTC threshold has been crossed |

The labels make decisions visible. The continuous ACC target still performs
most of the following action.

An emergency label does not guarantee collision avoidance. It means the
controller requests the strongest available braking in this educational model.

## 8. Complete update order

At every simulation step:

1. measure ego pose and speed;
2. locate the Pure Pursuit preview point;
3. measure the lead-vehicle gap and speed;
4. calculate closing speed and TTC;
5. calculate desired gap;
6. calculate and limit the ACC target;
7. apply safety-state overrides;
8. pass the selected speed to the Day 1 PI controller;
9. calculate Day 2 steering;
10. update the shared vehicle model and save metrics.

Longitudinal and lateral control are separate calculations, but their commands
act on the same vehicle.

## 9. End-of-lesson concept check

Answer before opening the solution:

1. Why does desired gap increase with ego speed?
2. Why is a 30 m gap not automatically safe?
3. What does negative closing speed mean?
4. Why can TTC be infinite at an uncomfortably short gap?
5. Why is the ACC layer placed before the PI controller?
6. Why is a collision-free deterministic simulation not enough to certify ACC?

??? success "Worked answers"
    1. A larger speed covers more distance during the same response time.
    2. Safety also depends on relative speed, delay, braking capability and
       road conditions.
    3. The lead vehicle is pulling away from the ego vehicle.
    4. TTC is infinite whenever the gap is not decreasing, even if the
       constant gap is too short.
    5. ACC selects an appropriate reference; the PI controller already knows
       how to convert a speed error into throttle or brake.
    6. The scenario omits sensor failures, cut-ins, variable friction, delay
       and many other real-world conditions.

## Summary

- Cruise control regulates speed; ACC also regulates traffic spacing.
- A time-headway policy makes desired distance grow with speed.
- Gap and closing speed provide different information.
- TTC is useful but insufficient by itself.
- ACC selects a speed target that the existing longitudinal controller tracks.
- Safety conclusions must remain limited to the tested model and scenario.
