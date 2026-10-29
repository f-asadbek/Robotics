"""Lab 6 - draw "47" with encoder-controlled moves and log odometry (TRIK, Python).

The same program runs in the TRIK Studio 2D simulator and on the real robot.
It detects where it runs (the simulator has no battery) and writes:
    odometry_data_sim.txt   or   odometry_data_real.txt     x,y,time  (m, m, ms)
    odometry_debug_sim.txt  or   odometry_debug_real.txt    + heading and encoders

Odometry (differential drive, every loop, midpoint integration):
    dL = r * dTheta_L,  dR = r * dTheta_R        (wheel angles in radians)
    d = (dL + dR) / 2,  dPhi = (dR - dL) / B
    x += d cos(phi + dPhi/2),  y += d sin(phi + dPhi/2),  phi += dPhi

Start pose = (0, 0), heading along +x. The path (A = stroke length):
    "4": down A, right A, up A, down 2A      "7": top bar A, down 2A
Overall about 0.8 m x 0.6 m with the default sizes.

Samples are kept in memory and written after the robot stops, so file
writing does not disturb the control loop.
"""
import math
import time

# ---- robot geometry: MEASURE these on your robot (metres) --------------------
WHEEL_RADIUS = 0.042        # r: half the wheel diameter
TRACK_WIDTH = 0.194         # B: centre of left tyre to centre of right tyre
LEFT_MOTOR, LEFT_ENCODER = "M4", "E4"
RIGHT_MOTOR, RIGHT_ENCODER = "M3", "E3"

# ---- trajectory size ---------------------------------------------------------
A = 0.30                    # m, stroke length; each digit is A wide and 2A tall
GAP = 0.20                  # m, space between the two digits

# ---- motion control ----------------------------------------------------------
MAX_POWER = 40              # % on straights
TURN_POWER = 30             # % when turning on the spot
MIN_POWER = 12              # %, enough to keep the wheels turning
K_DIST = 150.0              # % power per metre still to go
K_TURN = 1.0                # % power per degree of heading still to go
K_HEAD = 2.0                # % steering per degree of heading error on straights
K_CROSS = 3.0               # degrees of heading correction per cm off the line
MAX_CROSS = 20.0            # degrees, limit of that correction
RAMP = 120                  # % per second, soft start against wheel slip
DIST_TOL = 0.003            # m
TURN_TOL = 1.0              # degrees
HOLD_LOOPS = 5              # loops inside the tolerance before a move is done
MOVE_TIMEOUT = 20.0         # s, safety stop if a wheel is blocked
SETTLE_MS = 300             # keep logging while the robot comes to rest
LOOP_MS = 10
# ------------------------------------------------------------------------------


def beep(freq, ms):
    try:
        brick.playTone(freq, ms)
    except Exception:
        pass


def show(text):
    try:
        brick.display().clear()
        brick.display().addLabel(text, 1, 1)
        brick.display().redraw()
    except Exception:
        pass


def on_real_robot():
    # the 2D simulator has no battery model: brick.battery() returns nothing
    try:
        return brick.battery().readVoltage() > 1.0
    except Exception:
        return False


class Odometry:
    def __init__(self):
        self.enc_l = brick.encoder(LEFT_ENCODER)
        self.enc_r = brick.encoder(RIGHT_ENCODER)
        self.enc_l.reset()
        self.enc_r.reset()
        script.wait(100)
        self.last_l = self.enc_l.read()
        self.last_r = self.enc_r.read()
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        self.t0 = time.time()
        self.log = []
        self.update()

    def update(self):
        """Read the encoders, integrate the pose, store a sample."""
        l = self.enc_l.read()
        r = self.enc_r.read()
        d_l = math.radians(l - self.last_l) * WHEEL_RADIUS
        d_r = math.radians(r - self.last_r) * WHEEL_RADIUS
        self.last_l, self.last_r = l, r

        d = (d_l + d_r) / 2.0
        d_phi = (d_r - d_l) / TRACK_WIDTH
        mid = self.phi + d_phi / 2.0
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.phi += d_phi

        t_ms = int((time.time() - self.t0) * 1000)
        self.log.append((self.x, self.y, t_ms, self.phi, l, r))
        return l, r


def set_power(left, right):
    brick.motor(LEFT_MOTOR).setPower(int(round(left)))
    brick.motor(RIGHT_MOTOR).setPower(int(round(right)))


def wait_logging(odo, ms):
    end = time.time() + ms / 1000.0
    while time.time() < end:
        odo.update()
        script.wait(LOOP_MS)


def wrap(a):
    """Angle in degrees to (-180, 180]."""
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


def drive_power(remaining, gain, max_power, started):
    """Signed power: proportional to what is left, soft start, never below MIN_POWER."""
    p = min(max_power, gain * abs(remaining), MIN_POWER + RAMP * (time.time() - started))
    return math.copysign(max(p, MIN_POWER), remaining)


class Pilot:
    """Moves relative to a commanded pose, so turn and distance errors do not add up."""

    def __init__(self, odo):
        self.odo = odo
        self.heading = 0.0          # commanded heading, degrees
        self.cx = 0.0               # commanded position, m
        self.cy = 0.0

    def turn(self, angle_deg):
        """Turn on the spot to the commanded heading; positive = left (counter-clockwise)."""
        self.heading += angle_deg
        start, inside = time.time(), 0
        while time.time() - start < MOVE_TIMEOUT and inside < HOLD_LOOPS:
            self.odo.update()
            err = wrap(self.heading - math.degrees(self.odo.phi))
            if abs(err) <= TURN_TOL:
                inside += 1
                set_power(0, 0)
            else:
                inside = 0
                p = drive_power(err, K_TURN, TURN_POWER, start)
                set_power(-p, p)
            script.wait(LOOP_MS)
        set_power(0, 0)
        wait_logging(self.odo, SETTLE_MS)

    def forward(self, dist):
        """Drive dist metres along the commanded line, steering back onto it."""
        x0, y0 = self.cx, self.cy
        h = math.radians(self.heading)
        start, inside = time.time(), 0
        while time.time() - start < MOVE_TIMEOUT and inside < HOLD_LOOPS:
            self.odo.update()
            dx, dy = self.odo.x - x0, self.odo.y - y0
            along = dx * math.cos(h) + dy * math.sin(h)
            cross = -dx * math.sin(h) + dy * math.cos(h)     # m, left of the line
            remaining = dist - along
            if abs(remaining) <= DIST_TOL:
                inside += 1
                set_power(0, 0)
            else:
                inside = 0
                p = drive_power(remaining, K_DIST, MAX_POWER, start)
                back = max(-MAX_CROSS, min(MAX_CROSS, K_CROSS * cross * 100))
                steer = K_HEAD * wrap(self.heading - back - math.degrees(self.odo.phi))
                set_power(p - steer, p + steer)
            script.wait(LOOP_MS)
        set_power(0, 0)
        wait_logging(self.odo, SETTLE_MS)
        self.cx += dist * math.cos(h)
        self.cy += dist * math.sin(h)


def save(odo, tag):
    with open("odometry_data_%s.txt" % tag, "w") as f:
        for x, y, t_ms, phi, l, r in odo.log:
            f.write("%.4f,%.4f,%d\n" % (x, y, t_ms))
    with open("odometry_debug_%s.txt" % tag, "w") as f:
        f.write("time_ms,x,y,phi_deg,enc_left,enc_right\n")
        for x, y, t_ms, phi, l, r in odo.log:
            f.write("%d,%.4f,%.4f,%.2f,%s,%s\n" % (t_ms, x, y, math.degrees(phi), l, r))


def main():
    tag = "real" if on_real_robot() else "sim"
    print("Lab 6 odometry, run: %s, r = %.4f m, B = %.4f m" % (tag, WHEEL_RADIUS, TRACK_WIDTH))
    odo = Odometry()
    go = Pilot(odo)
    try:
        show("4")
        go.turn(-90)                 # face down the page
        go.forward(A)                # left stroke of the 4
        go.turn(90)
        go.forward(A)                # middle bar
        go.turn(90)
        go.forward(A)                # up to the top of the right stroke
        go.turn(180)
        go.forward(2 * A)            # full right stroke
        show("move to 7")
        go.turn(180)
        go.forward(2 * A)            # back up the right stroke
        go.turn(-90)
        go.forward(GAP)              # gap between the digits
        beep(800, 200)
        show("7")
        wait_logging(odo, 500)
        go.forward(A)                # top bar of the 7
        go.turn(-90)
        go.forward(2 * A)            # stroke of the 7
    finally:
        set_power(0, 0)
        save(odo, tag)
    x, y, t_ms, phi, l, r = odo.log[-1]
    print("End: x = %.3f m, y = %.3f m, heading = %.1f deg, %d samples, %.1f s"
          % (x, y, wrap(math.degrees(phi)), len(odo.log), t_ms / 1000.0))
    show("done %.2f %.2f" % (x, y))
    beep(1000, 300)


main()
