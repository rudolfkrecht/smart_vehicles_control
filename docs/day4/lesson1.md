# Lesson 1 — Theory: robustness and safety supervision

- **Format:** Instructor-led theory with short predictions
- **Main outcome:** Explain why one successful run is insufficient and how a
  supervisory fallback protects the cumulative controller

## From capability to evidence

On Days 1–3 the controller completed a defined highway and traffic scenario.
The correct conclusion was:

> The controller can complete this stated nominal simulation.

That result does not show what happens if:

- the sensor stops updating;
- the measured pose is noisy or biased;
- a command arrives late;
- the brakes produce less force than requested;
- the car is displaced from the lane;
- several changes occur together.

Robustness is not an absolute label. A defensible claim always names:

1. the variation;
2. its magnitude and unit;
3. when it occurs;
4. the acceptance criteria;
5. the observed result.

“Robust to a 0.25 s sensor delay in five fixed test cases” is testable.
“Robust controller” is incomplete.

## Classify disturbances

| Type | Meaning | Example | Typical effect |
|---|---|---|---|
| Noise | sample-to-sample variation | range $\pm0.3$ m | active or jittery command |
| Bias | persistent offset | steering $+1.5^\circ$ | continuous correction |
| Delay | correct value arrives late | pose delayed 0.25 s | oscillation and phase lag |
| Dropout | measurement unavailable | radar absent for 9 s | controller loses traffic evidence |
| Authority loss | applied actuation is weaker | 58% braking | longer stopping distance |
| External disturbance | state is changed externally | lateral push 1.35 m | recovery transient |

Predict:

1. Which fault may be difficult to identify from one instantaneous sample?
2. Which fault can make a numerically correct command physically ineffective?
3. Which fault is central in the Day 4 3D-like implementation?

??? success "Expected reasoning"
    A bias can resemble a real offset in one sample and usually requires
    consistency checking. Authority loss can make a correct command
    ineffective. The implementation task focuses on range-sensor dropout.

## Truth and measurement

The controller only receives its observation. The simulator also retains the
true state.

For example:

$$
d_{\mathrm{measured}}=d_{\mathrm{true}}+n_d+b_d,
$$

where $n_d$ is measurement noise and $b_d$ is bias. During dropout, there may
be no valid $d_{\mathrm{measured}}$ at all.

This separation is essential:

- the controller must act using available measurements;
- the evaluator should judge collision, true gap and road departure using
  simulated truth;
- otherwise a failed sensor could incorrectly report that the experiment was
  safe.

The Day 4 observation therefore includes:

| Signal | Meaning |
|---|---|
| `range_sensor_healthy` | diagnostic status |
| `range_measurement_age` | age of the unavailable/stale information |
| `lead_detected` | valid relevant lead measurement exists |
| `lead_distance` | measured gap when valid |
| `active_fault` | scenario annotation for analysis |

`lead_detected = False` is not enough by itself. It can mean either “the sensor
is healthy and no lead vehicle is in range” or “the sensor is unable to
observe the road.” The health flag distinguishes these cases.

## Supervisory control

A safety supervisor watches the assumptions required by the normal controller.
Its task is not necessarily to improve nominal tracking. Its task is to prevent
unsafe use of a controller outside its valid information state.

The Day 4 logic is:

```python
supervisor_active = (
    not observation.range_sensor_healthy
    or observation.range_measurement_age > 0.25
)

if supervisor_active:
    selected_target = 0.0
    mode = "SAFE_STOP"
```

The longitudinal layer then requests a controlled braking action. Pure Pursuit
continues to steer because the lane-reference information remains available in
this particular fault model.

This is an example of **graceful degradation**:

- full operation: PI + Pure Pursuit + ACC;
- degraded operation: lane keeping plus controlled stopping;
- recovery: normal ACC resumes after valid range data returns.

!!! important
    A fallback is valid only for the faults it was designed to handle. If the
    steering estimate and range sensor both fail, continuing lane keeping would
    require a different strategy.

## Why stopping distance matters

If a vehicle brakes with approximately constant deceleration magnitude $a$,
the idealized stopping distance is:

$$
d_{\mathrm{stop}}=\frac{v^2}{2a}.
$$

At $v=14\ \mathrm{m/s}$:

| Available deceleration | Idealized stopping distance |
|---:|---:|
| $5.0\ \mathrm{m/s^2}$ | $19.6\ \mathrm m$ |
| $3.0\ \mathrm{m/s^2}$ | $32.7\ \mathrm m$ |
| $2.0\ \mathrm{m/s^2}$ | $49.0\ \mathrm m$ |

If braking authority is reduced to $\eta_b$, a simple approximation is:

$$
a_{\mathrm{applied}}\approx \eta_b a_{\mathrm{nominal}}.
$$

With $\eta_b=0.58$, the same brake request produces much less deceleration.
This is why a supervisor should react before the remaining gap becomes
critical.

## Metrics and acceptance hierarchy

Use both typical and worst-case metrics.

Mean absolute path error:

$$
\operatorname{MAE}_{e_y}
=\frac{1}{N}\sum_{k=1}^{N}|e_{y,k}|.
$$

Maximum path error:

$$
e_{y,\max}=\max_k |e_{y,k}|.
$$

Speed RMSE:

$$
\operatorname{RMSE}_v
=\sqrt{\frac{1}{N}\sum_{k=1}^{N}
(v_{\mathrm{selected},k}-v_k)^2}.
$$

Discrete jerk:

$$
j_k=\frac{a_k-a_{k-1}}{\Delta t}.
$$

Evaluate in this order:

1. collision;
2. road departure;
3. minimum true gap;
4. required supervisor activation;
5. scenario completion;
6. worst and typical tracking;
7. comfort and control activity.

An excellent average cannot compensate for one collision.

## Repeatable test cases

A complete test case records:

- scenario name;
- vehicle and controller configuration;
- initial state;
- road and lead-vehicle schedule;
- fault type, magnitude and timing;
- random seed when noise is used;
- duration and sample time;
- pass/fail criteria.

Fixed seeds support fair comparisons. If Controller A and Controller B receive
different random noise, the experiment changed two things at once.

Keep practice and final evaluation separate:

- practice cases support tuning and diagnosis;
- evaluation cases test the frozen result;
- repeatedly tuning on evaluation cases turns them into practice data.

## Concept check

Answer in one sentence each:

1. Why is `lead_detected = False` ambiguous without sensor health?
2. Why can a safe fallback reduce performance?
3. Why is minimum gap evaluated using truth rather than measurement?
4. What does passing all five Day 4 scenarios prove?

??? success "Suggested answers"
    1. It may mean no target or no functioning sensor.
    2. Conservative braking sacrifices speed and completion to preserve
       margin.
    3. A faulty measurement must not be allowed to declare itself safe.
    4. It proves success for those documented simulated conditions, not
       general real-vehicle safety.

## Lesson summary

- Robustness claims require named disturbances and magnitudes.
- The evaluator separates truth from controller measurements.
- A supervisor checks whether the normal controller assumptions remain valid.
- Day 4 uses a controlled safe stop during range-sensor dropout.
- Safety gates are evaluated before accuracy or efficiency.
