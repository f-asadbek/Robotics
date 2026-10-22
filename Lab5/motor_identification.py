"""Lab 5 - motor step-response data collector (runs on the TRIK robot).

For every drive wheel in WHEELS and every PWM level from -100 % to +100 % in
5 % steps (0 excluded) the program:

  1. resets the encoder and waits for the wheel to be still,
  2. reads the battery voltage at rest,
  3. applies the PWM step (t = 0) and samples (time stamp, encoder angle)
     into a Python list for RUN_TIME seconds - no file I/O while sampling,
  4. reads the battery voltage under load, switches the motor off,
  5. only then writes motor_data/wheel_<N>_v_<PWM>.txt,
  6. rests REST_TIME seconds so the wheel stops before the next level.

Lift the robot so the wheels spin freely (or set LOAD accordingly).
Total run time: 2 wheels x 40 levels x ~3.5 s = about 5 minutes.

File format (lines starting with '#' are header lines):
    # key: value
    ...
    # format: time_ms angle_deg
    <time_ms> <angle_deg>
"""
import os
import time

# ---- configuration ----------------------------------------------------------
# (wheel id used in the file name, motor port, encoder port)
WHEELS = [
    (1, "M3", "E3"),
    (2, "M4", "E4"),
]
LOAD = "free_spin"      # free_spin | on_floor   (state it in the report)
RUN_TIME = 2.0          # s recorded per PWM level
DT_MS = 5               # ms, target sampling period (real stamps are logged)
SETTLE_MS = 500         # ms after encoder reset, wheel at rest
REST_MS = 1000          # ms between levels, wheel coasts to a stop
PAUSE_MS = 3000         # ms between the two wheels
OUT_DIR = "motor_data"
# -----------------------------------------------------------------------------

LEVELS = list(range(-100, 0, 5)) + list(range(5, 105, 5))


def show(line1, line2=""):
    brick.display().clear()
    brick.display().addLabel(line1, 1, 1)
    brick.display().addLabel(line2, 1, 20)
    brick.display().redraw()


def beep(freq, ms):
    # playTone is not in every firmware's API; a missing beep must not stop the run
    try:
        brick.playTone(freq, ms)
    except Exception:
        pass


def battery_volts():
    # the 2D simulator has no battery (brick.battery() is None); log nan there
    try:
        return float(brick.battery().readVoltage())
    except Exception:
        return float("nan")


def record_step(motor, encoder, power):
    """Apply a PWM step from rest. Returns (samples, battery_rest, battery_load)."""
    encoder.reset()
    script.wait(SETTLE_MS)
    battery_rest = battery_volts()

    samples = []
    t0 = time.time()
    motor.setPower(power)                       # the step: t = 0
    while True:
        t = time.time() - t0
        if t > RUN_TIME:
            break
        samples.append((int(t * 1000), encoder.read()))
        script.wait(DT_MS)

    battery_load = battery_volts()
    motor.powerOff()
    return samples, battery_rest, battery_load


def save(wheel_id, motor_port, encoder_port, power, battery_rest, battery_load,
         samples):
    name = "%s/wheel_%d_v_%d.txt" % (OUT_DIR, wheel_id, power)
    lines = [
        "# wheel_id: %d" % wheel_id,
        "# motor_port: %s" % motor_port,
        "# encoder_port: %s" % encoder_port,
        "# pwm_percent: %d" % power,
        "# battery_rest_v: %s" % battery_rest,
        "# battery_load_v: %s" % battery_load,
        "# load: %s" % LOAD,
        "# run_time_s: %s" % RUN_TIME,
        "# target_dt_ms: %d" % DT_MS,
        "# recorded_unix_time: %d" % int(time.time()),
        "# format: time_ms angle_deg",
    ]
    for t_ms, angle in samples:
        lines.append("%d %s" % (t_ms, angle))
    f = open(name, "w")
    f.write("\n".join(lines) + "\n")
    f.close()


def run_wheel(wheel_id, motor_port, encoder_port):
    motor = brick.motor(motor_port)
    encoder = brick.encoder(encoder_port)
    print("Wheel %d (%s/%s), load %s" % (wheel_id, motor_port, encoder_port, LOAD))

    for power in LEVELS:
        show("Wheel %d  %s" % (wheel_id, motor_port), "PWM %d %%" % power)
        try:
            samples, b_rest, b_load = record_step(motor, encoder, power)
        finally:
            motor.powerOff()
        save(wheel_id, motor_port, encoder_port, power, b_rest, b_load, samples)

        n = len(samples)
        dt = (samples[-1][0] - samples[0][0]) / float(n - 1) if n > 1 else 0
        print("  PWM %4d %%: %3d samples, mean dt %.1f ms, final %s deg, "
              "battery %.2f / %.2f V"
              % (power, n, dt, samples[-1][1], b_rest, b_load))
        script.wait(REST_MS)


def main():
    if not os.path.exists(OUT_DIR):
        os.makedirs(OUT_DIR)

    for i, (wheel_id, motor_port, encoder_port) in enumerate(WHEELS):
        if i > 0:
            show("Next: wheel %d" % wheel_id, "port %s" % motor_port)
            beep(600, 200)
            script.wait(PAUSE_MS)
        run_wheel(wheel_id, motor_port, encoder_port)

    show("Done", "%d files" % (len(WHEELS) * len(LEVELS)))
    beep(1000, 300)                             # audible "done"
    brick.stop()


main()
