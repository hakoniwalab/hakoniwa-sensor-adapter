#!/usr/bin/env python3
"""Identify the rotor-order IMU vibration model of Hakoniwa Drone PRO (components.propellerVibration) from real PX4
ULogs. Reference implementation of docs/design/propeller-vibration-identification.md.

Inputs: a fit log (amplitudes and phases) and, optionally, more logs of the same airframe for the speed exponents.
Every log needs sensor_gyro_fifo, sensor_accel_fifo (accelerometer amplitudes are NaN without it), esc_status with
esc_rpm, vehicle_land_detected and vehicle_local_position; vehicle_angular_velocity is used when present.

Output: the propellerVibration object (paste it under components of a drone config) and a report of every choice,
count and unresolved value.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.signal import butter, sosfiltfilt

WINDOW_S = 0.5          # spectral window
BAND_TOL = 0.03         # order band: k * f_rotor * (1 -/+ BAND_TOL), per rotor
TAKEOFF_SKIP_S = 10.0   # hover starts this long after take-off ...
LANDING_SKIP_S = 5.0    # ... and ends this long before landing
MAX_HSPEED = 1.0        # m/s
MAX_VSPEED = 0.5        # m/s
MAX_YAW_RATE = np.radians(10.0)
DEMOD_LOWPASS_HZ = 1.0  # phase: low-pass after demodulation
DEMOD_SEPARATION_HZ = 3.0  # phase: another rotor's same order must be at least this far away
MIN_SPEED_SPAN = 1.10   # exponent: max/min mean rotor speed needed in the windows of one log and sensor
TOPICS = ["sensor_gyro_fifo", "sensor_accel_fifo", "esc_status", "vehicle_land_detected",
          "vehicle_local_position", "vehicle_angular_velocity"]
SENSORS = {"gyro": "sensor_gyro_fifo", "accel": "sensor_accel_fifo"}


def load(path):
    ulog = ULog(str(path), TOPICS)
    data = {}
    for item in ulog.data_list:
        if item.multi_id == 0 and item.name not in data:
            data[item.name] = item.data
    for topic in ("sensor_gyro_fifo", "esc_status", "vehicle_land_detected", "vehicle_local_position"):
        if topic not in data:
            raise RuntimeError(f"{path}: missing {topic}")
    return data


def rotor_rpm(data, rotors):
    esc = data["esc_status"]
    keys = [f"esc[{r}].esc_rpm" for r in range(rotors)]
    missing = [k for k in keys if k not in esc]
    if missing:
        raise RuntimeError(f"esc_status has no {missing[0]}")
    return esc["timestamp"].astype(float), np.stack([np.abs(esc[k]).astype(float) for k in keys], axis=1)


def fifo_blocks(fifo):
    """Continuous raw FIFO blocks (time us, xyz, rate Hz). timestamp_sample is the last sample of a message."""
    count = fifo["samples"].astype(int)
    stamp = fifo["timestamp_sample"].astype(np.int64)
    usable = (count[1:] > 0) & (np.diff(stamp) > 0)
    period = float(np.median(np.diff(stamp)[usable] / count[1:][usable]))
    axes = [np.stack([fifo[f"{a}[{i}]"] for i in range(int(count.max()))], axis=1) for a in "xyz"]
    scale = np.asarray(fifo["scale"], float)
    breaks = np.where(np.abs(np.diff(stamp) - count[1:] * period) > 1.5 * period)[0] + 1
    for start, end in zip(np.r_[0, breaks], np.r_[breaks, len(stamp)]):
        if end - start < 10:
            continue
        xyz = np.stack([np.concatenate([v[i, :count[i]] * scale[i] for i in range(start, end)]) for v in axes])
        first = stamp[start] - (count[start] - 1) * period
        yield first + np.arange(xyz.shape[1]) * period, xyz, 1e6 / period


def hover_mask(data, half):
    """Hover samples on the vehicle_local_position clock; half = first/second splits them at their median time."""
    lp, ld = data["vehicle_local_position"], data["vehicle_land_detected"]
    air = ld["timestamp"][ld["landed"] == 0]
    if len(air) == 0:
        raise RuntimeError("the vehicle never left the ground")
    t = lp["timestamp"].astype(float)
    ok = (t > air[0] + TAKEOFF_SKIP_S * 1e6) & (t < air[-1] - LANDING_SKIP_S * 1e6)
    ok &= (np.hypot(lp["vx"], lp["vy"]) < MAX_HSPEED) & (np.abs(lp["vz"]) < MAX_VSPEED)
    av = data.get("vehicle_angular_velocity")
    if av is not None:
        ok &= np.interp(t, av["timestamp"], np.abs(av["xyz[2]"])) < MAX_YAW_RATE
    if half:
        middle = np.median(t[ok])
        ok &= (t < middle) if half == "first" else (t >= middle)
    return t, ok


def in_hover(hover, time):
    t, ok = hover
    return np.interp(time, t, ok.astype(float)) > 0.99


def order_windows(fifo, esc, hover, orders):
    """Per 0.5 s hover window: mean rotor rpm and per-order, per-axis RMS. Every frequency bin belongs to at most
    one order: the bin goes to the order whose nearest rotor line k * f_r is closest (no double counting where the
    bands of neighbouring orders overlap)."""
    etime, rpm = esc
    out = []
    for time, xyz, fs in fifo_blocks(fifo):
        length = int(WINDOW_S * fs)
        hann = np.hanning(length)
        power = np.mean(hann ** 2)
        freq = np.fft.rfftfreq(length, 1.0 / fs)
        for start in range(0, xyz.shape[1] - length + 1, length):
            tw = time[start:start + length]
            if not in_hover(hover, np.array([tw.mean()]))[0]:
                continue
            frot = np.array([np.interp(tw.mean(), etime, rpm[:, r]) for r in range(rpm.shape[1])]) / 60.0
            if frot.min() * 60.0 < 500.0:
                continue
            seg = xyz[:, start:start + length]
            spec = np.abs(np.fft.rfft((seg - seg.mean(axis=1, keepdims=True)) * hann, axis=1)) ** 2
            distance = np.full((len(orders), len(freq)), np.inf)
            for i, k in enumerate(orders):
                lines = k * frot
                near = np.min(np.abs(freq[:, None] - lines[None, :]), axis=1)
                inside = np.any(np.abs(freq[:, None] - lines[None, :]) <= BAND_TOL * lines[None, :], axis=1)
                distance[i, inside] = near[inside]
            owner = np.argmin(distance, axis=0)
            owned = np.isfinite(np.min(distance, axis=0))
            rms = np.full((len(orders), 3), np.nan)
            for i, k in enumerate(orders):
                mask = owned & (owner == i)
                if k * frot.max() * (1 + BAND_TOL) < fs / 2 and mask.any():
                    rms[i] = np.sqrt(2.0 * spec[:, mask].sum(axis=1) / (length * length * power))
            out.append({"time": tw.mean(), "rpm": frot * 60.0, "rms": rms})
    return out


def exponents(windows_by_log, orders, max_order, default):
    """Speed exponent per order: pooled log-log slope of the three-axis order RMS on the mean rotor speed, with a
    separate intercept for every (log, sensor), so units and airframe-to-airframe level differences do not enter the
    slope. Groups whose speed span is below MIN_SPEED_SPAN carry no slope information and are left out."""
    result, notes = {}, {}
    for i, k in enumerate(orders):
        if k > max_order:
            result[k], notes[k] = default, f"fixed at {default} (order above --exponent-max-order)"
            continue
        num = den = 0.0
        groups = 0
        for (_log, _sensor), windows in windows_by_log.items():
            pts = [(np.log(np.mean(w["rpm"])), np.log(np.linalg.norm(w["rms"][i])))
                   for w in windows if np.all(np.isfinite(w["rms"][i])) and np.linalg.norm(w["rms"][i]) > 0]
            if len(pts) < 10:
                continue
            p = np.asarray(pts)
            if np.exp(p[:, 0].max() - p[:, 0].min()) < MIN_SPEED_SPAN:
                continue
            dx, dy = p[:, 0] - p[:, 0].mean(), p[:, 1] - p[:, 1].mean()
            num += float(dx @ dy)
            den += float(dx @ dx)
            groups += 1
        if den > 0:
            result[k], notes[k] = max(0.0, num / den), f"pooled slope over {groups} log/sensor groups"
        else:
            result[k], notes[k] = default, f"fixed at {default} (no group with speed span >= {MIN_SPEED_SPAN})"
    return result, notes


def amplitudes(windows, orders, reference_rpm, exponent):
    """Per-rotor amplitude at referenceRpm, per axis: median over windows of RMS / sqrt(sum_r q_r^2 / 2), where
    q_r = (rpm_r / referenceRpm)^p_k is the generator's own scale for rotor r (a sine of amplitude a has RMS a/sqrt 2
    and the rotors' phases are independent)."""
    out = {}
    for i, k in enumerate(orders):
        values = []
        for w in windows:
            q = (w["rpm"] / reference_rpm) ** exponent[k]
            values.append(w["rms"][i] / np.sqrt(np.sum(q ** 2) / 2.0))
        values = np.asarray(values) if values else np.full((1, 3), np.nan)
        out[k] = [float(np.nanmedian(values[:, a])) if np.isfinite(values[:, a]).any() else float("nan")
                  for a in range(3)]
    return out


def phases(fifo, esc, hover, signs, orders):
    """Per rotor and order: the gyro y and z phase relative to x, by demodulating each axis with the rotor's own
    integrated phase (Drone PRO rotorAxisPhaseDeg), and the phase coherence R -> phaseJitterDeg = sqrt(-ln R).
    Also the rotor's own gyro amplitude per axis, 2 sqrt(mean |z|^2): only the part that follows the rotor angle
    within the low-pass, so it is reported (rotor-to-rotor differences), not used for the model amplitude."""
    etime, rpm = esc
    acc = {}
    for time, gyro, fs in fifo_blocks(fifo):
        hovering = in_hover(hover, time)
        if hovering.sum() < 3 * fs:
            continue
        frot = np.stack([np.interp(time, etime, rpm[:, r]) / 60.0 for r in range(len(signs))])
        sos = butter(4, DEMOD_LOWPASS_HZ, fs=fs, output="sos")
        edge = int(fs)
        for r in range(len(signs)):
            theta = 2.0 * np.pi * np.cumsum(frot[r]) / fs
            others = np.delete(frot, r, axis=0)
            for k in orders:
                z = [sosfiltfilt(sos, gyro[a] * np.exp(-1j * k * theta)) for a in range(3)]
                valid = hovering & (np.min(np.abs(k * (others - frot[r])), axis=0) >= DEMOD_SEPARATION_HZ)
                valid[:edge] = False
                valid[-edge:] = False
                if valid.sum() < fs:
                    continue
                pxy, pxz = (z[0] * np.conj(z[1]))[valid], (z[0] * np.conj(z[2]))[valid]
                a = acc.setdefault((r, k), [0j, 0j, 0.0, 0, np.zeros(3)])
                a[0] += pxy.sum()
                a[1] += pxz.sum()
                a[2] += np.abs(pxy).sum()
                a[3] += valid.sum()
                a[4] += np.array([np.sum(np.abs(z[ax][valid]) ** 2) for ax in range(3)])
    out = {}
    for k in orders:
        rows, coherence, amplitude = [], [], []
        for r, sign in enumerate(signs):
            a = acc.get((r, k))
            if a is None:
                rows.append(None)
                amplitude.append(None)
                continue
            amplitude.append([float(v) for v in 2.0 * np.sqrt(a[4] / a[3])])
            dy = -sign * np.degrees(np.angle(a[0]))
            dz = -sign * np.degrees(np.angle(a[1]))
            rows.append([0.0, round(float(dy), 1), round(float(dz), 1)])
            coherence.append(abs(a[0]) / a[2])
        r_med = float(np.median(coherence)) if coherence else float("nan")
        jitter = float(np.degrees(np.sqrt(max(0.0, -np.log(max(r_med, 1e-6)))))) if coherence else float("nan")
        out[k] = {"rotors": rows, "coherence": r_med, "phaseJitterDeg": round(jitter, 1),
                  "rotor_gyro_amplitude_radps": amplitude}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fit-log", type=Path, required=True, help="ULog for amplitudes and phases")
    ap.add_argument("--exponent-log", type=Path, action="append", default=[],
                    help="more ULogs of the same airframe for the speed exponents (repeatable); the fit log is "
                         "always included")
    ap.add_argument("--half", choices=["first", "second"], help="use only this half of the fit log's hover")
    ap.add_argument("--rotation-signs", default="1,1,-1,-1",
                    help="+1/-1 per esc_status index, as the drone config's rotor directions")
    ap.add_argument("--max-order", type=int, default=14)
    ap.add_argument("--exponent-max-order", type=int, default=6, help="orders above this use --default-exponent")
    ap.add_argument("--default-exponent", type=float, default=2.0)
    ap.add_argument("--phase-max-order", type=int, default=6, help="orders above this get no per-rotor phase")
    ap.add_argument("--reference-rpm", type=float,
                    help="default: median mean rotor rpm of the fit log's hover windows")
    ap.add_argument("--unresolved-phase", choices=["zero", "omit-order"], default="zero",
                    help="a rotor whose phase cannot be separated: 0,0,0, or no rotorAxisPhaseDeg for that order")
    ap.add_argument("--phase-jitter-bandwidth-hz", type=float, default=2.0)
    ap.add_argument("--alias-attenuation-db", type=float, default=40.0)
    ap.add_argument("--out", type=Path, required=True, help="propellerVibration JSON")
    ap.add_argument("--report", type=Path, help="choices, counts and unresolved values (default: <out>.report.json)")
    args = ap.parse_args()

    signs = [int(s) for s in args.rotation_signs.split(",")]
    orders = list(range(1, args.max_order + 1))
    windows_by_log = {}
    fit = None
    for index, path in enumerate([args.fit_log] + args.exponent_log):
        data = load(path)
        esc = rotor_rpm(data, len(signs))
        hover = hover_mask(data, args.half if index == 0 else None)
        for sensor, topic in SENSORS.items():
            if topic in data:
                windows_by_log[(str(path), sensor)] = order_windows(data[topic], esc, hover, orders)
        if index == 0:
            fit = (data, esc, hover)
        print(f"{path.name}: " + ", ".join(f"{s}={len(windows_by_log.get((str(path), s), []))}" for s in SENSORS)
              + " hover windows", file=sys.stderr)

    fit_windows = {s: windows_by_log.get((str(args.fit_log), s), []) for s in SENSORS}
    if not fit_windows["gyro"]:
        raise RuntimeError("the fit log has no gyro hover window")
    reference_rpm = args.reference_rpm or float(np.median([np.mean(w["rpm"]) for w in fit_windows["gyro"]]))
    exponent, exponent_notes = exponents(windows_by_log, orders, args.exponent_max_order, args.default_exponent)
    amp = {s: amplitudes(fit_windows[s], orders, reference_rpm, exponent) for s in SENSORS}
    phase_orders = [k for k in orders if k <= args.phase_max_order]
    phase = phases(fit[0]["sensor_gyro_fifo"], fit[1], fit[2], signs, phase_orders)

    harmonics, unresolved = [], []
    for k in orders:
        h = {"order": k, "exponent": exponent[k],
             "accelAmplitudeMps2": [round(v, 5) for v in amp["accel"][k]],
             "gyroAmplitudeRadps": [round(v, 6) for v in amp["gyro"][k]]}
        if k in phase:
            rows = phase[k]["rotors"]
            missing = [r for r, row in enumerate(rows) if row is None]
            unresolved += [{"order": k, "rotor": r} for r in missing]
            if not (missing and args.unresolved_phase == "omit-order"):
                h["rotorAxisPhaseDeg"] = [row if row is not None else [0.0, 0.0, 0.0] for row in rows]
            if np.isfinite(phase[k]["phaseJitterDeg"]):
                h["phaseJitterDeg"] = phase[k]["phaseJitterDeg"]
        harmonics.append(h)
    model = {"enabled": True, "model": "rpm-harmonics", "referenceRpm": round(reference_rpm, 1),
             "aliasAttenuationDb": args.alias_attenuation_db,
             "phaseJitterBandwidthHz": args.phase_jitter_bandwidth_hz, "axisPhase": "rotating",
             "harmonics": harmonics}
    args.out.write_text(json.dumps(model, indent=1) + "\n")

    report = {
        "fit_log": str(args.fit_log), "half": args.half, "exponent_logs": [str(p) for p in args.exponent_log],
        "rotation_signs": signs,
        "reference_rpm": {"value": reference_rpm, "source": "argument" if args.reference_rpm else
                          "median mean rotor rpm of the fit log's gyro hover windows"},
        "hover_windows": {f"{Path(log).name}:{s}": len(w) for (log, s), w in windows_by_log.items()},
        "exponent": {str(k): {"value": exponent[k], "how": exponent_notes[k]} for k in orders},
        "phase_coherence": {str(k): phase[k]["coherence"] for k in phase},
        "rotor_gyro_amplitude_radps": {
            "values": {str(k): phase[k]["rotor_gyro_amplitude_radps"] for k in phase},
            "note": "per rotor and axis, by demodulation: the part that follows the rotor angle. Shows how "
                    "different the rotors are; the model amplitude is common to all rotors and comes from the "
                    "band power (all rotors, including vibration spread around the lines)"},
        "unresolved_phase": {"handling": args.unresolved_phase, "rotors": unresolved,
                             "why": "the same order of another rotor stays within "
                                    f"{DEMOD_SEPARATION_HZ} Hz, so the two cannot be separated"},
        "not_identified": {"phaseJitterBandwidthHz": "argument (default 2 Hz)",
                           "aliasAttenuationDb": "argument (a simulation requirement, not a property of the log)",
                           "accel phase": "the accelerometer uses the gyro's per-rotor phase"},
    }
    (args.report or args.out.with_suffix(".report.json")).write_text(json.dumps(report, indent=1) + "\n")
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
