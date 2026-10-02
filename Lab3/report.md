# Lab 3 — Line Following with PID Control

**Student:** Yelbek Abdumajitov  
**Student ID:** 240066  
**Software:** TRIK Studio 2026.2

## 1. Objective and scope

The objective is to follow a black line using two light sensors and differential motor control. The controller changes the difference between the wheel powers to correct the robot's position relative to the line.

This report documents `line_sim_240066.qrs` and `line_real_240066.qrs`, including the available simulation validation results and a link to the recorded real-robot demonstration. The required comparison of P, PI, PD, PID and a nonlinear controller is **not yet complete**. The five simulation runs reported below are repetitions of the same controller, not five different controllers.

## 2. Program structure and setup

Both programs contain six blocks: Start, initialization function, controller function, M3 motor, M4 motor and Timer. Initialization runs once. The remaining four blocks repeat in a loop.

The light sensors use A3 and A4. M3 drives the left wheel and M4 drives the right wheel. Both programs apply the same motor-mixing principle:

```text
left motor power  = forward power + steering correction
right motor power = forward power - steering correction
```

The simulation track consists of five straight black segments with sharp corners and a width of 6 pixels. Its saved sensor positions are A3 at `(25, 21)` and A4 at `(25, 29)` in robot-local coordinates. Realistic sensor, motor and physics options are disabled for the reported simulation tests.

| Parameter | Simulation | Real robot |
|---|---:|---:|
| Proportional gain | 0.65 | 1.3 |
| Integral gain | 0.01 | 0.01 |
| Derivative gain | 0.8 | 15 |
| Base motor power | 22 | 30 |
| Speed-reduction coefficient | 0.25 | 0.35 |
| Integral limit | ±100 | ±200 |
| Timer block delay | 10 ms | 1 ms |

The coefficients belong to the exact discrete formulas below. In particular, the derivative calculations differ between the two programs. The timer delay is not a measurement of the complete loop execution time.

## 3. Sensor references and calibration

The simulation computes its error directly:

```text
err = sensorA4 - sensorA3
```

The real program records both sensor readings at startup, then subtracts these reference values:

```text
leftReference = sensorA4
rightReference = sensorA3
lineError = (sensorA4 - leftReference) - (sensorA3 - rightReference)
```

The robot must therefore be placed at the intended centred starting position before the real program starts. The reference-variable names refer to the assignments above; they should not be used to infer the physical sensor mounting sides.

This startup reference subtraction compensates for the initial difference between the sensor readings. Neither file implements full normalization using separately measured black and white values.


## 4. Implemented controllers

### Simulation

During normal following, the simulation uses:

```text
integral = clamp(integral + err * 0.01, -100, 100)
p = 0.65 * err
i = 0.01 * integral
d = 0.8 * (err - old)
u = clamp(p + i + d, -40, 40)
v = max(8, 22 - 0.25 * abs(err))
old = err
```

Here, `clamp(x, a, b)` is shorthand for the file's `max(a, min(b, x))` expression. Each final motor command is limited to −100…100.

When both sensor readings are at most 4, the controller treats the line as lost, resets the integral and increments a loss counter. It first moves forward at power 15. At 100 consecutive lost-line iterations, it pivots right with M3 at +20 and M4 at −20. Detecting the line resets the counter and resumes normal following. At 360 consecutive lost-line iterations, both motor commands become zero.

The rightward search is tuned for the saved track and starting direction. The program has no separate finish marker: it stops after prolonged line loss at the endpoint. Its block loop continues until execution is stopped manually.

### Real robot

The real program uses the following calculations:

```text
pTerm = 1.3 * lineError
errorSum = errorSum + lineError * 0.01
errorSum = max(-200, min(200, errorSum))
iTerm = 0.01 * errorSum
dTerm = 15 * (lineError - previousError) / 10
turnPower = pTerm + iTerm + dTerm
previousError = lineError
drivePower = 30 - 0.35 * abs(lineError)

M3 power = drivePower + turnPower
M4 power = drivePower - turnPower
```

The real file retains the uploaded program's formulas, constants and 1 ms timer delay. Only the block placement and variable names were changed. It has no explicit motor-power clamp, corner-search routine or automatic finish stop.

Both current implementations combine PID correction with error-dependent speed reduction. The simulation also includes nonlinear recovery logic. These are not isolated pure-PID baselines for comparing controller types.

## 5. Simulation validation results

Five runs were executed in the TRIK Studio 2026.2 simulator. Besides the saved start, the tests used small vertical-position and heading offsets. Validation checked that the robot reached five checkpoints in route order, finished near the endpoint, and remained stationary through 150 simulated seconds.

| Run | Vertical start offset | Heading offset | Full route completed | Time until stationary |
|---|---:|---:|---|---:|
| 1 | 0 px | 0° | Yes | 110.88 s |
| 2 | +2 px | +2° | Yes | 110.58 s |
| 3 | −2 px | −2° | Yes | 111.04 s |
| 4 | +2 px | −2° | Yes | 110.40 s |
| 5 | −2 px | +2° | Yes | 111.00 s |

The mean time until stationary was **110.78 simulated seconds**. All five runs completed the route and remained stopped. These times include the endpoint search before stopping; they are not separately measured finish-line crossing times or lap times.

The tests recorded position and heading trajectories. They did **not** record the sensor error on every control cycle. Therefore, RMS sensor error, error-versus-time plots and the number of line-loss episodes cannot be reported from these validation results. Successful completion does not mean there were zero temporary line losses; recovery is part of the controller.



## 7. Real-robot demonstration

[Watch the recorded demonstration on YouTube](https://youtube.com/shorts/FdpGy3EBK5M)

The link is provided for the real-robot demonstration associated with `line_real_240066.qrs`. Numerical lap time, RMS error and line-loss counts have not been extracted from the real run.

## 8. Conclusions

The implemented simulation controller completed the track in all five validation runs, including small changes to the starting position and heading. Its mean time until stationary was 110.78 simulated seconds. The earlier controller lost the line at the first sharp corner, while the revised sensor placement and recovery logic allowed the saved route to be completed. The real-robot program retains the original control logic, and the demonstration link is included above. A comparison winner and the effect of increasing base power cannot yet be determined because the five-controller measurements and speed experiment are still pending.
