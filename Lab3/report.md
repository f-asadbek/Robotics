# Lab 3: Line Following Report

## Goal
Follow the track line with a signed-error controller on the simulator (`line_sim.qrs`) and on the real TRIK robot (`line_real.qrs`), and compare controller types (P, PI, PD, PID, one nonlinear).

## Sensor calibration and error
Both programs use two line sensors (A3, A4). At program start the current readings are stored as the reference (`calL`, `calR`), so the error is the change from that baseline:

```
delta = (sensorA4 - calL/calR) - (sensorA3 - calR/calL)
```

`delta` is 0 when both sensors see the same thing, and its sign tells which side the line is on. Steering is `u` added to the M3 power and subtracted from the M4 power on top of a base speed.

## Controllers implemented

### Real robot (`line_real.qrs`): PID with speed reduction
```
acc   = clamp(acc + delta * 0.01, -iLim, iLim)     # anti-windup
dE    = (delta - prevE) / 10
u     = kP*delta + kI*acc + kD*dE
speed = base - kS * |delta|                         # slow down in turns
M3 = speed + u,  M4 = speed - u
```
| Parameter | Value |
|---|---|
| base | 30 |
| kP | 1.3 |
| kI | 0.01 |
| kD | 15 (applied to `delta/10`, i.e. effective 1.5) |
| kS | 0.35 |
| iLim | 200 |
| timer | 1 ms |

### Simulator (`line_sim.qrs`): nonlinear gain with line-loss recovery
```
gain = kP + kN * delta^2                            # gain grows with error
u    = gain*delta + kI*sumE + kD*(delta - prevE)
M3 = speed + u,  M4 = speed - u
```
| Parameter | Value |
|---|---|
| speed | 20 |
| kP | 0.5 |
| kI, kD | 0, 0 (set per test run) |
| kN | 0.0005 |
| lostLvl | 5 |
| turnG | 4 |
| timer | 30 ms |

If both sensors drop below `lostLvl`, the robot turns toward the side where the line was last seen (`lastSide`, gain `turnG`) until the line is found again.

## Tuning approach
Start from a P-only controller (Ziegler–Nichols step 1: raise `kP` until sustained oscillation), back off, then add `kD` to damp oscillation and a small `kI` to remove steady offset. Each controller (P, PI, PD, PID, nonlinear) is selected by setting the unused gains to 0, so the track, base speed and calibration stay identical and the controller is the only variable.