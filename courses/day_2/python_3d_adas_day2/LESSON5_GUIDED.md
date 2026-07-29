# Lesson 5 — Guided Pure Pursuit implementation

The complete student instructions are in:

```text
docs/day2/lesson5.md
```

## 45-minute route

1. Run the Reference controller and identify the preview target.
2. Run the Student baseline with zero steering.
3. Implement target vector, target bearing and wrapped relative angle.
4. Implement the Pure Pursuit equation.
5. Apply the steering limit.
6. Compare fixed look-ahead values of 4, 6 and 10 m.
7. Save the selected controller and CSV evidence.

Start:

```bat
py -3.12 run_simulator.py
```

Edit:

```text
student_controller.py
```

Reset the graphical simulator after every saved code change.
