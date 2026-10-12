# CS413 Robotics — Lab 4: Line Following

**Author:** Yelbek Abdumajitov  
**Program:** `line_real_race.qrs` — TRIK Studio visual blocks

## Controller and calibration

The robot uses a filtered PD controller with sensors `A4` and `A3`, and motors `M3` and `M4`. At startup, it captures `left = sensorA4` and `right = sensorA3` with the robot aligned on the line, and initialises `old = 0` and `d = 0`.

**Calibration readings:** `left = ____`, `right = ____`.

**Parameters:** `kp = 0.8`, `kd = 0.010`, derivative filter `0.02 s`, derivative limits `±8`, steering limits `±100`, Timer `5 ms`.

Each cycle uses the following calculations, where `clamp(x, a, b) = max(a, min(b, x))`:

```text
dt = max(0.001, elapsed_seconds_since_previous_cycle)
err = (sensorA4 - left) - (sensorA3 - right)
d = clamp((0.02*d + kd*(err-old)) / (0.02+dt), -8, 8)
old = err
u = clamp(kp*err + d, -100, 100)
v = 100 - max(u, -u)
M3 = v + u
M4 = v - u
wait 5 ms
```

This motor mixing keeps one calculated wheel command at **100% PWM** each cycle and the other within **−100% to 100%**. Here, `u` is half the wheel-power difference, equivalent to the handout's steering command `U = 2*u`.

## Lap results and video

| Run | Time |
| --- | --- |
| Earlier controller: best reported practice lap | **43 seconds** |
| Updated controller: best practice lap | ____ seconds |
| Official race | ____ seconds |

[Watch the successful practice lap](https://youtube.com/shorts/IHw1LTUnUpw?si=WjfQaxU4Y5vmMqBd). The video and 43-second result refer to the earlier controller, before the full-speed update.

## Error over time

**Best practice lap error plot:** ____________________  
Plot measured `err` against elapsed time in seconds for the same run reported above.

## Changes and lap-time factors

The fixed base command of 30 was replaced with `v = 100 - abs(u)` to keep one wheel at full power. The steering limit increased from ±30 to ±100, while the gains, derivative filter, Timer, and six-block layout were retained. The previous base command limited straight-line drive power. For the updated controller, stable cornering and steering corrections will influence lap time, with the existing gains serving as starting settings for testing.
