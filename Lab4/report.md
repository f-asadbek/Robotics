# Lab 4: Big Line Following (Real Robot)

## Overview
The goal was to make the TRIK robot follow a black line on the competition track as fast as possible. The program is `big_line_real.qrs`, built from TRIK Studio blocks. The video `1015.mp4` shows the robot following the line.

## Controller
A PD controller using the two line sensors (A4 and A3).

1. **Calibration.** Each sensor is normalized to a 0–100 scale using measured white/black readings:
   - A4: min 3, max 45
   - A3: min 2, max 95

   Normalized values are clamped to 0–100.
2. **Error.** `err = normA4 - normA3`. It is 0 when the line is centred between the sensors.
3. **Control law.**
   ```
   derivative = err - lastErr
   uRaw = kp * err + kd * derivative
   u    = clamp(uRaw, -umax, umax)
   ```
4. **Motor mixing.** Base speed `v = 27`:
   ```
   M3 = v + u
   M4 = v - u
   ```
5. **Loop.** The program repeats every 5 ms timer delay.