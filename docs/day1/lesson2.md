# 2. Requirements and performance

## Learning objective

Translate driving requests into measurable targets, constraints and performance criteria.

## Prepared demonstration

```bash
python day_1_longitudinal/demos/lesson2_response_metrics.py
```

The demonstration compares a slow P controller, a tuned P controller and a PI
controller.

![Response metrics comparison](../assets/images/day1/lesson2_response_metrics.png)

Read the graph and printed table using these requirements:

- target speed: 15 m/s;
- overshoot below 10%;
- settling time below 10 seconds;
- command within \(-1 \le u \le 1\).

Ask:

1. Which controller reaches 90% of the target first?
2. Which controller retains a steady-state error?
3. Is the lowest error automatically the best engineering choice?
