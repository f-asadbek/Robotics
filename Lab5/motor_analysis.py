"""Lab 5 - offline identification of the TRIK drive motors.

Reads the raw step-response files written by motor_identification.py
(motor_data/wheel_<N>_v_<PWM>.txt), and for every wheel:

  * fits  theta(t) = w_ss * (t - T_m + T_m * exp(-t / T_m))  to every PWM level,
  * converts PWM to voltage, U = PWM/100 * U_battery (under load),
  * fits the steady-state line  w_ss = (U - sgn(U) * U_0) / k_e,
    per direction and for both directions together,
  * finds the measured dead zone (levels at which the wheel did not turn),
  * simulates the identified model with odeint next to a measured step.

Outputs: results/*.csv|json, figures/*.png, and the tables inside report.md
(between <!-- AUTO:name --> markers; the rest of the report is left alone).

    py motor_analysis.py                     # motor_data/ -> report.md
    py motor_analysis.py --data other_dir --report none
Needs numpy, scipy, matplotlib (all available in Colab).
"""
import argparse
import glob
import json
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import optimize
from scipy.integrate import odeint

MOVE_DEG = 5.0          # a run with less total rotation counts as "did not move"
MOVE_W = 0.2            # rad/s; creeping slower than this (stick-slip) also counts as still
TM_BOUNDS = (1e-3, 3.0)  # s, search range of the time constant
TM_LEVEL = 30           # % PWM; recommended T_m = median over |PWM| >= this
COLORS = {1: "#1f77b4", 2: "#d62728", 3: "#2ca02c", 4: "#9467bd", 5: "#8c564b"}
FILE_RE = re.compile(r"wheel_(\d+)_v_(-?\d+)\.txt$")


# ---------------------------------------------------------------- loading ---
def read_run(path):
    """Return (header dict, t [s], theta [rad]) of one raw file."""
    header, rows = {}, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if ":" in line:
                    key, value = line[1:].split(":", 1)
                    header[key.strip()] = value.strip()
                else:
                    header[line[1:].strip()] = ""
                continue
            rows.append([float(v) for v in line.split()[:2]])
    fmt = header.get("format", "time_ms angle_deg").split()
    data = np.array(rows)
    if fmt[:2] == ["time_ms", "angle_deg"]:
        t_ms, deg = data[:, 0], data[:, 1]
    elif fmt[:2] in (["angle_deg", "time_ms"], ["position_deg", "time_ms"]):
        deg, t_ms = data[:, 0], data[:, 1]
    else:
        raise ValueError("%s: unknown column format %r" % (path, header.get("format")))
    return header, (t_ms - t_ms[0]) / 1000.0, np.radians(deg)


def battery_of(header):
    for key in ("battery_load_v", "battery_rest_v", "battery_voltage", "battery_voltage_mv"):
        if key in header and np.isfinite(float(header[key])):
            return float(header[key]), key
    raise SystemExit("no battery voltage in the file header (recorded in the 2D simulator?); "
                     "U = PWM/100 * U_battery needs a real robot run")


def load_all(data_dir):
    runs = {}
    for path in sorted(glob.glob(os.path.join(data_dir, "wheel_*_v_*.txt"))):
        m = FILE_RE.search(os.path.basename(path))
        if not m:
            continue
        wheel, pwm = int(m.group(1)), int(m.group(2))
        header, t, theta = read_run(path)
        runs.setdefault(wheel, {})[pwm] = {"header": header, "t": t, "theta": theta}
    if not runs:
        raise SystemExit("no wheel_<N>_v_<PWM>.txt files in %s" % data_dir)
    return runs


# ---------------------------------------------------------------- fitting ---
def theta_model(t, w_ss, t_m):
    return w_ss * (t - t_m + t_m * np.exp(-t / t_m))


def fit_step(t, theta):
    """Fit one step response. Returns dict with w_ss, T_m and 1-sigma errors."""
    n = len(t)
    tail = slice(int(n * 0.6), n)
    w_lin = np.polyfit(t[tail], theta[tail], 1)[0] if n >= 5 else np.nan
    out = {"w_lin": w_lin, "w_ss": np.nan, "w_sd": np.nan, "T_m": np.nan,
           "T_sd": np.nan, "rmse_deg": np.nan, "tm_flag": ""}
    try:
        p0 = [w_lin if w_lin else 1.0, 0.1]
        lo, hi = [-200.0, TM_BOUNDS[0]], [200.0, TM_BOUNDS[1]]
        (w, tm), cov = optimize.curve_fit(theta_model, t, theta, p0=p0,
                                          bounds=(lo, hi), maxfev=20000)
        sd = np.sqrt(np.diag(cov))
        resid = theta - theta_model(t, w, tm)
        out.update(w_ss=w, w_sd=sd[0], T_m=tm, T_sd=sd[1],
                   rmse_deg=np.degrees(np.sqrt(np.mean(resid ** 2))))
        dt = np.median(np.diff(t))
        if tm <= TM_BOUNDS[0] * 1.01 or tm >= TM_BOUNDS[1] * 0.99:
            out["tm_flag"] = "at bound"
        elif tm < 2 * dt:
            out["tm_flag"] = "< 2 samples"
        elif not np.isfinite(sd[1]) or sd[1] > 0.5 * tm:
            out["tm_flag"] = "poorly determined"
    except (RuntimeError, ValueError) as exc:
        out["tm_flag"] = "fit failed: %s" % exc
    return out


def encoder_sign(levels):
    """+1 if positive PWM gives a growing angle, -1 if motor and encoder disagree."""
    s = sum(np.sign(p) * r["theta"][-1] for p, r in levels.items())
    return -1 if s < 0 else 1


def analyse_wheel(wheel, levels, fit_min_pwm=0):
    sign = encoder_sign(levels)
    rows = []
    for pwm in sorted(levels):
        r = levels[pwm]
        t, theta = r["t"], sign * r["theta"]
        u_bat, u_key = battery_of(r["header"])
        dts = np.diff(t) * 1000
        tail = slice(len(t) // 2, len(t))
        slope = np.polyfit(t[tail], theta[tail], 1)[0] if len(t) >= 6 else 0.0
        row = {"wheel": wheel, "pwm": pwm, "U": pwm / 100.0 * u_bat, "U_bat": u_bat,
               "U_bat_source": u_key, "n": len(t),
               "dt_med_ms": float(np.median(dts)), "dt_max_ms": float(dts.max()),
               "rot_deg": float(np.degrees(theta[-1])),
               "moving": abs(np.degrees(theta[-1])) >= MOVE_DEG and abs(slope) >= MOVE_W}
        if "battery_rest_v" in r["header"]:
            row["U_rest"] = float(r["header"]["battery_rest_v"])
        if row["moving"]:
            row.update(fit_step(t, theta))
        else:
            crept = abs(row["rot_deg"]) >= MOVE_DEG
            row.update(w_ss=slope if crept else 0.0, w_sd=np.nan, T_m=np.nan, T_sd=np.nan,
                       w_lin=slope, rmse_deg=np.nan, tm_flag="creep" if crept else "no motion")
        rows.append(row)

    mov = [r for r in rows if r["moving"] and abs(r["pwm"]) >= fit_min_pwm]
    U = np.array([r["U"] for r in mov])
    W = np.array([r["w_ss"] for r in mov])

    def line_fit(mask):
        if mask.sum() < 3:
            return None
        a, b = np.polyfit(U[mask], W[mask], 1)
        pred = a * U[mask] + b
        ss = np.sum((W[mask] - W[mask].mean()) ** 2)
        r2 = 1 - np.sum((W[mask] - pred) ** 2) / ss if ss > 0 else np.nan
        return {"k_e": 1.0 / a, "U0": -b / a, "slope": a, "intercept": b, "r2": r2,
                "n": int(mask.sum())}

    fwd, rev = line_fit(U > 0), line_fit(U < 0)
    # joint odd model: w = U/k_e - sgn(U) U0/k_e
    A = np.column_stack([U, np.sign(U)])
    (a, c), *_ = np.linalg.lstsq(A, W, rcond=None)
    pred = A @ np.array([a, c])
    joint = {"k_e": 1.0 / a, "U0": -c / a, "r2": 1 - np.sum((W - pred) ** 2) / np.sum((W - W.mean()) ** 2),
             "rmse": float(np.sqrt(np.mean((W - pred) ** 2))), "n": len(W)}

    def dead_zone(direction):
        side = [r for r in rows if np.sign(r["pwm"]) == direction]
        still = [abs(r["pwm"]) for r in side if not r["moving"]]
        moved = [abs(r["pwm"]) for r in side if r["moving"]]
        return {"max_still_pwm": max(still) if still else 0,
                "min_moving_pwm": min(moved) if moved else None}

    tm_pool = [r["T_m"] for r in mov if abs(r["pwm"]) >= TM_LEVEL and not r["tm_flag"]]
    params = {
        "wheel": wheel,
        "motor_port": levels[min(levels)]["header"].get("motor_port", "?"),
        "encoder_port": levels[min(levels)]["header"].get("encoder_port", "?"),
        "load": levels[min(levels)]["header"].get("load", "?"),
        "encoder_sign": sign,
        "levels": len(rows),
        "fwd": fwd, "rev": rev, "joint": joint,
        "dead_zone_fwd": dead_zone(1), "dead_zone_rev": dead_zone(-1),
        "T_m_recommended": float(np.median(tm_pool)) if tm_pool else float("nan"),
        "T_m_range": [float(min(tm_pool)), float(max(tm_pool))] if tm_pool else None,
        "T_m_pool_n": len(tm_pool),
        "fit_min_pwm": fit_min_pwm,
        "synthetic": any("SYNTHETIC" in k for k in levels[min(levels)]["header"]),
    }
    return rows, params


# ------------------------------------------------------------- simulation ---
def simulate(params, u_volts, t):
    """Integrate T_m dw/dt + w = U_eff / k_e with the identified joint model."""
    k_e, u0, t_m = params["joint"]["k_e"], params["joint"]["U0"], params["T_m_recommended"]
    u_eff = np.sign(u_volts) * max(abs(u_volts) - u0, 0.0) if u0 > 0 else u_volts - np.sign(u_volts) * u0

    def motor(state, _t):
        w, _theta = state
        return [(u_eff / k_e - w) / t_m, w]

    y = odeint(motor, [0.0, 0.0], t)
    return y[:, 1], y[:, 0]


def smooth_speed(t, theta, half=2):
    """Central-difference speed over +-half samples (quantised encoder)."""
    w = np.full_like(theta, np.nan)
    for i in range(half, len(t) - half):
        w[i] = (theta[i + half] - theta[i - half]) / (t[i + half] - t[i - half])
    return w


# ---------------------------------------------------------------- figures ---
def watermark(fig, synthetic):
    if synthetic:
        fig.text(0.5, 0.5, "SYNTHETIC DRY-RUN DATA\nnot robot measurements",
                 ha="center", va="center", fontsize=28, color="red", alpha=0.18,
                 rotation=20)


def fig_omega_vs_u(rows_by_wheel, params_by_wheel, path, synthetic):
    fig, ax = plt.subplots(figsize=(10, 6))
    for wheel, rows in rows_by_wheel.items():
        p, c = params_by_wheel[wheel], COLORS.get(wheel, None)
        U = np.array([r["U"] for r in rows])
        W = np.array([r["w_ss"] for r in rows])
        mov = np.array([r["moving"] for r in rows])
        ax.scatter(U[mov], W[mov], s=22, color=c, zorder=3,
                   label="wheel %d (%s) measured" % (wheel, p["motor_port"]))
        ax.scatter(U[~mov], W[~mov], s=40, facecolors="none", edgecolors=c, zorder=3,
                   label="wheel %d did not turn / crept" % wheel)
        j = p["joint"]
        for direction in (1, -1):
            u0 = max(j["U0"], 0.0)
            xs = np.linspace(direction * u0, direction * np.abs(U).max(), 50)
            ax.plot(xs, (xs - direction * j["U0"]) / j["k_e"], "--", color=c, lw=1.4,
                    label=("wheel %d fit: k_e = %.3f V·s/rad, U$_0$ = %.2f V"
                           % (wheel, j["k_e"], j["U0"])) if direction == 1 else None)
        for direction, key in ((1, "dead_zone_fwd"), (-1, "dead_zone_rev")):
            dz = p[key]["max_still_pwm"]
            if dz:
                u_dz = dz / 100.0 * np.mean([r["U_bat"] for r in rows])
                ax.axvspan(0, direction * u_dz, color=c, alpha=0.10)
    ax.axhline(0, color="gray", lw=0.7)
    ax.axvline(0, color="gray", lw=0.7)
    ax.set_xlabel("motor voltage U = PWM/100 · U$_{battery}$  [V]")
    ax.set_ylabel("steady-state speed ω$_{ss}$  [rad/s]")
    ax.set_title("Steady-state speed vs voltage (shaded: measured dead zone)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")
    watermark(fig, synthetic)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_tm(rows_by_wheel, params_by_wheel, path, synthetic):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for wheel, rows in rows_by_wheel.items():
        c = COLORS.get(wheel, None)
        good = [r for r in rows if r["moving"] and not r["tm_flag"]]
        # fits stuck at a search bound carry no information; they stay in the table only
        bad = [r for r in rows if r["moving"] and r["tm_flag"] in ("< 2 samples", "poorly determined")]
        ax.errorbar([r["pwm"] for r in good], [r["T_m"] * 1000 for r in good],
                    yerr=[r["T_sd"] * 1000 for r in good], fmt="o", ms=4, color=c,
                    capsize=2, label="wheel %d" % wheel)
        if bad:
            ax.scatter([r["pwm"] for r in bad], [r["T_m"] * 1000 for r in bad], marker="x",
                       color=c, label="wheel %d unreliable fit" % wheel)
        tm = params_by_wheel[wheel]["T_m_recommended"]
        if np.isfinite(tm):
            ax.axhline(tm * 1000, color=c, ls=":", lw=1)
    dt = np.median([r["dt_med_ms"] for rows in rows_by_wheel.values() for r in rows])
    ax.axhline(2 * dt, color="gray", ls="--", lw=1, label="2 × sampling period")
    ax.set_xlabel("PWM [%]")
    ax.set_ylabel("T$_m$ [ms]")
    ax.set_ylim(bottom=0)
    ax.set_title("Fitted time constant per level (dotted: recommended value)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    watermark(fig, synthetic)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_step(runs, rows_by_wheel, params_by_wheel, pwm, path, synthetic):
    wheels = sorted(rows_by_wheel)
    fig, axs = plt.subplots(2, len(wheels), figsize=(6 * len(wheels), 8), squeeze=False)
    stats = {}
    for col, wheel in enumerate(wheels):
        p = params_by_wheel[wheel]
        level = min(runs[wheel], key=lambda q: abs(q - pwm))
        run = runs[wheel][level]
        row = next(r for r in rows_by_wheel[wheel] if r["pwm"] == level)
        t, theta = run["t"], p["encoder_sign"] * run["theta"]
        t_fine = np.linspace(0, t[-1], 400)
        th_sim, w_sim = simulate(p, row["U"], t_fine)
        th_sim_at = np.interp(t, t_fine, th_sim)
        rmse = np.degrees(np.sqrt(np.mean((theta - th_sim_at) ** 2)))
        stats[wheel] = {"pwm": level, "U": row["U"], "rmse_deg": float(rmse),
                        "end_err_pct": float(100 * (th_sim_at[-1] - theta[-1]) / theta[-1])}
        c = COLORS.get(wheel, None)
        ax = axs[0][col]
        ax.plot(t, theta, ".", ms=4, color=c, label="measured")
        ax.plot(t_fine, th_sim, "k-", lw=1.5, label="simulated, identified model (odeint)")
        if np.isfinite(row["T_m"]):
            ax.plot(t_fine, theta_model(t_fine, row["w_ss"], row["T_m"]), "--", color="orange",
                    lw=1, label="this run's own fit")
        ax.set_title("Wheel %d (%s), PWM %+d %%, U = %.2f V\nRMS error %.1f°"
                     % (wheel, p["motor_port"], level, row["U"], rmse))
        ax.set_xlabel("t [s]")
        ax.set_ylabel("θ [rad]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax = axs[1][col]
        ax.plot(t, smooth_speed(t, theta), ".", ms=4, color=c, label="measured (finite difference)")
        ax.plot(t_fine, w_sim, "k-", lw=1.5, label="simulated")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("ω [rad/s]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    watermark(fig, synthetic)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return stats


def fig_all_steps(runs, params_by_wheel, path, synthetic):
    wheels = sorted(runs)
    fig, axs = plt.subplots(1, len(wheels), figsize=(6 * len(wheels), 4.5), squeeze=False)
    cmap = plt.get_cmap("coolwarm")
    for col, wheel in enumerate(wheels):
        ax = axs[0][col]
        sign = params_by_wheel[wheel]["encoder_sign"]
        for pwm, r in sorted(runs[wheel].items()):
            ax.plot(r["t"], sign * r["theta"], lw=0.9, color=cmap((pwm + 100) / 200))
        ax.set_title("Wheel %d - all %d step responses" % (wheel, len(runs[wheel])))
        ax.set_xlabel("t [s]")
        ax.set_ylabel("θ [rad]")
        ax.grid(alpha=0.3)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(-100, 100))
    fig.colorbar(sm, ax=axs[0].tolist(), label="PWM [%]")
    watermark(fig, synthetic)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------- report ----
def f(x, fmt="%.2f"):
    return "—" if x is None or (isinstance(x, float) and not np.isfinite(x)) else fmt % x


def table_levels(rows_by_wheel, params_by_wheel):
    wheels = sorted(rows_by_wheel)
    head = "| PWM % |" + "".join(
        " U%d [V] | ω%d [rad/s] | T_m%d [ms] |" % (w, w, w) for w in wheels)
    sep = "|---:|" + "---:|---:|---:|" * len(wheels)
    lines = [head, sep]
    by = {w: {r["pwm"]: r for r in rows_by_wheel[w]} for w in wheels}
    for pwm in sorted(set().union(*[set(b) for b in by.values()])):
        cells = []
        for w in wheels:
            r = by[w].get(pwm)
            if r is None:
                cells += ["missing"] * 3
                continue
            tm = f(r["T_m"] * 1000 if np.isfinite(r["T_m"]) else None, "%.0f")
            if np.isfinite(r["T_sd"]) and tm != "—":
                tm += " ± %.0f" % (r["T_sd"] * 1000)
            if r["moving"] and r["tm_flag"]:
                tm += " ⚠"
            speed = f(r["w_ss"]) if r["moving"] else "%s (%s)" % (
                f(r["w_ss"]), "creep" if r["tm_flag"] == "creep" else "still")
            cells += [f(r["U"]), speed, tm]
        lines.append("| %+d | %s |" % (pwm, " | ".join(cells)))
    lines.append("")
    lines.append("ω is the fitted steady-state speed; T_m ± 1σ from the fit covariance. "
                 "⚠ = T_m not resolved (shorter than two sampling periods, at the search "
                 "bound, or σ > 50 %); those levels are excluded from the recommended T_m.")
    return "\n".join(lines)


def table_params(params_by_wheel):
    lines = ["| Wheel | Ports | k_e [V·s/rad] | U₀ [V] | R² | k_e fwd / rev | "
             "Dead zone fwd / rev (PWM %%) | T_m [ms] (median, |PWM| ≥ %d %%) | Encoder sign |" % TM_LEVEL,
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for w, p in sorted(params_by_wheel.items()):
        j, fw, rv = p["joint"], p["fwd"], p["rev"]
        dzf, dzr = p["dead_zone_fwd"]["max_still_pwm"], p["dead_zone_rev"]["max_still_pwm"]
        rng = p["T_m_range"]
        lines.append("| %d | %s/%s | **%.3f** | %.2f | %.3f | %s / %s | %s / %s | **%s** (%s–%s, n=%d) | %+d |" % (
            w, p["motor_port"], p["encoder_port"], j["k_e"], j["U0"], j["r2"],
            f(fw["k_e"] if fw else None, "%.3f"), f(rv["k_e"] if rv else None, "%.3f"),
            "≤ %d" % dzf if dzf else "none", "≤ %d" % dzr if dzr else "none",
            f(p["T_m_recommended"] * 1000, "%.0f"),
            f(rng[0] * 1000 if rng else None, "%.0f"), f(rng[1] * 1000 if rng else None, "%.0f"),
            p["T_m_pool_n"], p["encoder_sign"]))
    lines.append("")
    lines.append("k_e and U₀ come from one fit over both directions, "
                 "ω = (U − sgn(U)·U₀)/k_e, using every level at which the wheel turned. "
                 "A negative U₀ means the curve is steeper near zero than the line "
                 "(no dead zone in the linear fit). The measured dead zone is the largest "
                 "|PWM| at which the wheel did not turn (< %g° in the run) or only crept "
                 "(< %g rad/s, stick-slip)." % (MOVE_DEG, MOVE_W))
    return "\n".join(lines)


def table_quality(rows_by_wheel, params_by_wheel):
    lines = ["| Wheel | Files | Samples per run (min / median) | Sampling period median / max [ms] | "
             "Battery at rest [V] | Battery under load [V] | Load |",
             "|---|---:|---:|---:|---:|---:|---|"]
    for w, rows in sorted(rows_by_wheel.items()):
        n = [r["n"] for r in rows]
        rest = [r.get("U_rest") for r in rows if r.get("U_rest") is not None]
        load = [r["U_bat"] for r in rows]
        lines.append("| %d | %d | %d / %d | %.1f / %.0f | %s | %.2f – %.2f | %s |" % (
            w, len(rows), min(n), int(np.median(n)),
            np.median([r["dt_med_ms"] for r in rows]), max(r["dt_max_ms"] for r in rows),
            "%.2f – %.2f" % (min(rest), max(rest)) if rest else "—",
            min(load), max(load), params_by_wheel[w]["load"]))
    return "\n".join(lines)


def key_numbers(rows_by_wheel, params_by_wheel, step_stats, track_w):
    out = []
    wheels = sorted(params_by_wheel)
    for w in wheels:
        p = params_by_wheel[w]
        fw, rv = p["fwd"], p["rev"]
        if fw and rv:
            out.append("- Wheel %d: forward vs reverse k_e differ by %.1f %% (%.3f vs %.3f V·s/rad)."
                       % (w, 100 * (rv["k_e"] - fw["k_e"]) / fw["k_e"], fw["k_e"], rv["k_e"]))
        if p["T_m_range"]:
            out.append("- Wheel %d: T_m ranges %.0f–%.0f ms over |PWM| ≥ %d %% — the linear model "
                       "assumes one constant." % (w, p["T_m_range"][0] * 1000, p["T_m_range"][1] * 1000, TM_LEVEL))
        out.append("- Wheel %d: steady-state line fit R² = %.3f, RMS residual %.2f rad/s."
                   % (w, p["joint"]["r2"], p["joint"]["rmse"]))
    if len(wheels) == 2:
        a, b = wheels
        ra = {r["pwm"]: r for r in rows_by_wheel[a]}
        rb = {r["pwm"]: r for r in rows_by_wheel[b]}
        for pwm in (50, 100, -50, -100):
            if pwm in ra and pwm in rb and ra[pwm]["moving"] and rb[pwm]["moving"]:
                wa, wb = ra[pwm]["w_ss"], rb[pwm]["w_ss"]
                fast, slow = (a, b) if abs(wa) >= abs(wb) else (b, a)
                diff = 100 * abs(wa - wb) / max(abs(wa), abs(wb))
                # straight-line command: heading rate / forward speed = 2 dw / (W (wa + wb))
                drift = np.degrees(2 * (wb - wa) / (track_w * (wa + wb)))
                out.append("- PWM %+d %%: wheel %d %.2f rad/s, wheel %d %.2f rad/s; wheel %d is "
                           "%.1f %% slower than wheel %d. With the same PWM on both motors the "
                           "robot would turn ≈ %.1f° per metre (track width W = %.3f m)."
                           % (pwm, a, wa, b, wb, slow, diff, fast, abs(drift), track_w))
    for w, s in sorted(step_stats.items()):
        out.append("- Simulation check, wheel %d at PWM %+d %%: RMS angle error %.1f°, "
                   "angle after 2 s off by %+.1f %%." % (w, s["pwm"], s["rmse_deg"], s["end_err_pct"]))
    return "\n".join(out)


def handoff(params_by_wheel):
    """Values to paste into the Week 6 / Week 7 programs."""
    lines = ["```python", "# Lab 5 identified motor parameters (from motor_analysis.py)"]
    for w, p in sorted(params_by_wheel.items()):
        port = p["motor_port"]
        lines.append("K_E_%s = %.4f    # V*s/rad, wheel %d" % (port, p["joint"]["k_e"], w))
        lines.append("U0_%s  = %.3f     # V, offset of the steady-state line" % (port, p["joint"]["U0"]))
        lines.append("T_M_%s = %.4f    # s" % (port, p["T_m_recommended"]))
        lines.append("ENC_SIGN_%s = %+d  # multiply encoder readings by this" % (port, p["encoder_sign"]))
    lines.append("```")
    return "\n".join(lines)


def inject(report_path, blocks):
    text = open(report_path, encoding="utf-8").read()
    for name, body in blocks.items():
        pattern = re.compile(r"(<!-- AUTO:%s -->\n).*?(\n<!-- /AUTO:%s -->)" % (name, name), re.S)
        if not pattern.search(text):
            print("  report: marker AUTO:%s not found, skipped" % name)
            continue
        text = pattern.sub(lambda m: m.group(1) + body + m.group(2), text)
    open(report_path, "w", encoding="utf-8", newline="\n").write(text)


# ------------------------------------------------------------------- main ---
def main():
    ap = argparse.ArgumentParser(description="Identify T_m and k_e from Lab 5 step responses.")
    ap.add_argument("--data", default="motor_data")
    ap.add_argument("--out", default=".", help="folder for results/ and figures/")
    ap.add_argument("--report", default="report.md", help="report to fill, or 'none'")
    ap.add_argument("--fit-min-pwm", type=int, default=0,
                    help="use only |PWM| >= this for the k_e line fit (default: all moving levels)")
    ap.add_argument("--sim-pwm", type=int, default=50, help="PWM level of the simulated step")
    ap.add_argument("--track-width", type=float, default=0.194,
                    help="distance between the wheels [m], for the drift estimate")
    a = ap.parse_args()

    runs = load_all(a.data)
    rows_by_wheel, params_by_wheel = {}, {}
    for wheel, levels in sorted(runs.items()):
        rows_by_wheel[wheel], params_by_wheel[wheel] = analyse_wheel(wheel, levels, a.fit_min_pwm)
    synthetic = any(p["synthetic"] for p in params_by_wheel.values())
    if synthetic:
        print("WARNING: these files are marked SYNTHETIC (dry run) - not robot data")

    res_dir, fig_dir = os.path.join(a.out, "results"), os.path.join(a.out, "figures")
    os.makedirs(res_dir, exist_ok=True)
    os.makedirs(fig_dir, exist_ok=True)

    keys = ["wheel", "pwm", "U", "U_bat", "U_bat_source", "moving", "rot_deg", "w_ss", "w_sd",
            "w_lin", "T_m", "T_sd", "tm_flag", "rmse_deg", "n", "dt_med_ms", "dt_max_ms"]
    with open(os.path.join(res_dir, "step_fits.csv"), "w", encoding="utf-8") as fh:
        fh.write(",".join(keys) + "\n")
        for rows in rows_by_wheel.values():
            for r in rows:
                fh.write(",".join(str(r.get(k, "")) for k in keys) + "\n")
    with open(os.path.join(res_dir, "parameters.json"), "w", encoding="utf-8") as fh:
        json.dump(params_by_wheel, fh, indent=2, default=float)

    fig_omega_vs_u(rows_by_wheel, params_by_wheel, os.path.join(fig_dir, "omega_vs_U.png"), synthetic)
    fig_tm(rows_by_wheel, params_by_wheel, os.path.join(fig_dir, "Tm_vs_pwm.png"), synthetic)
    fig_all_steps(runs, params_by_wheel, os.path.join(fig_dir, "all_steps.png"), synthetic)
    step_stats = fig_step(runs, rows_by_wheel, params_by_wheel, a.sim_pwm,
                          os.path.join(fig_dir, "step_response_sim.png"), synthetic)

    for w, p in sorted(params_by_wheel.items()):
        print("wheel %d (%s): k_e = %.3f V*s/rad, U0 = %.2f V, R2 = %.3f, T_m = %.0f ms, "
              "encoder sign %+d, %d levels"
              % (w, p["motor_port"], p["joint"]["k_e"], p["joint"]["U0"], p["joint"]["r2"],
                 p["T_m_recommended"] * 1000, p["encoder_sign"], p["levels"]))

    if a.report.lower() != "none":
        inject(a.report, {
            "quality": table_quality(rows_by_wheel, params_by_wheel),
            "parameters": table_params(params_by_wheel),
            "levels": table_levels(rows_by_wheel, params_by_wheel),
            "key_numbers": key_numbers(rows_by_wheel, params_by_wheel, step_stats, a.track_width),
            "handoff": handoff(params_by_wheel),
        })
        print("report updated: %s" % a.report)


if __name__ == "__main__":
    main()
