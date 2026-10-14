# Lab 2 — PID Trajectory Report

## Controller design

Movement is now driven by encoder feedback instead of fixed timers. Each straight segment and turn calls one of three reusable PID subprograms (`PID straight`, `PID turn CCW`, `PID turn CW`), each running a position-PID loop per wheel: `power = Kp*error + Ki*integral + Kd*derivative`, where `error = target − encoder reading`, looping until both wheels reach their target encoder count.

## Final Kp, Ki, Kd

_TODO: fill in after tuning. Shipped starting point (Ziegler–Nichols step 1: raise Kp with Ki = Kd = 0) is **Kp = 0.5, Ki = 0, Kd = 0**, set in the "Expression" block at the start of each PID subprogram body — edit it there to retune (one edit applies to every call of that subprogram)._

_TODO: describe how you arrived at the final values (Ziegler–Nichols starting point → manual adjustment, and what each change did)._

## Step-response data

_TODO: table or plot for at least 3 coefficient sets, including one deliberately badly tuned (e.g. Kp too high → oscillation, or Ki too high → overshoot/windup). Encoder values are visible live in the Watch List panel while running in Debug mode — record encoder3/encoder4 vs. time for each set._

## Lab 1 vs. Lab 2 comparison

_TODO (3–6 sentences): compare the timer-based Lab 1 trajectory to this encoder-based PID version — what got better (e.g. consistency, repeatability, resistance to surface/battery variation), and what PID did **not** fix (e.g. it doesn't fix space constraints, or wheel-diameter/track calibration errors, or doesn't add dynamic obstacle avoidance)._

## Notes

- The Timer-based targets from Lab 1 were converted into approximate placeholder encoder-tick targets (proportionally scaled from the old millisecond values) inside `pid_sim.qrs`. **These have not been empirically tuned** — like the Lab 1 timer values, they need to be calibrated by running in the simulator and reading real `encoder3`/`encoder4` values in the Watch List, then adjusting the `target` value on each "PID straight" / "PID turn CCW" / "PID turn CW" call block.
- `pid_real.qrs` starts as an identical copy of `pid_sim.qrs`. If the real robot ends up needing different Kp/Ki/Kd (very likely, due to friction/battery differences not modeled in simulation), update `pid_real.qrs`'s PID subprogram bodies and note the difference here.
