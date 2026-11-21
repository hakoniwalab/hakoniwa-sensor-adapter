#!/usr/bin/env python3
"""Measure the four X500 IMU evaluation cards and write their complete derived data as CSV."""
import argparse
import csv
import gc
from collections import defaultdict
from pathlib import Path

import numpy as np
from pyulog import ULog

HERE = Path(__file__).resolve().parent
DEFAULT_IDS = HERE.parents[2] / "docs/evaluation/imu-x500/data/flight_ids.csv"
WIN_S = 0.5
FMIN, FMAX = 40.0, 1000.0
TOL, KMAX = 0.03, 6
FIT_LOG = "407292d1-cb50-48b7-a13e-142a50245044"
METRICS = ("gyro_vibration_metric", "accel_vibration_metric", "delta_angle_coning_metric",
           "delta_velocity_coning_metric")
PARAMETERS = ("IMU_GYRO_CUTOFF", "IMU_ACCEL_CUTOFF", "IMU_DGYRO_CUTOFF", "IMU_DACC_CUTOFF",
              "IMU_GYRO_DNF_EN", "IMU_GYRO_DNF_MIN", "IMU_GYRO_DNF_BW", "IMU_GYRO_DNF_HMC")
MAP_X_EDGES = np.arange(20.0, 285.0, 5.0)
MAP_FREQ = np.arange(FMIN, FMAX + 0.1, 4.0)


def write_csv(path, rows, fields=None):
    rows = list(rows)
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {path} ({len(rows)} rows)")


def find_log(root, log_id):
    root = Path(root)
    for path in (root / f"{log_id}.ulg", root / "logs_vib" / f"{log_id}.ulg", root / "logs" / f"{log_id}.ulg"):
        if path.is_file():
            return path
    raise FileNotFoundError(f"{log_id}.ulg not found under {root}")


def datasets(ulog):
    """First (primary) instance of every requested topic."""
    out = {}
    for item in ulog.data_list:
        if item.multi_id == 0 and item.name not in out:
            out[item.name] = item.data
    return out


def fifo_blocks(data):
    """Yield continuous raw FIFO blocks; timestamp_sample is the last sample, in microseconds."""
    count = data["samples"].astype(int)
    stamp = data["timestamp_sample"].astype(np.int64)
    usable = (count[1:] > 0) & (np.diff(stamp) > 0)
    period = float(np.median(np.diff(stamp)[usable] / count[1:][usable]))
    maximum = int(count.max())
    axes = [np.stack([data[f"{axis}[{i}]"] for i in range(maximum)], axis=1) for axis in "xyz"]
    scale = np.asarray(data["scale"], float)
    breaks = np.where(np.abs(np.diff(stamp) - count[1:] * period) > 1.5 * period)[0] + 1
    for start, end in zip(np.r_[0, breaks], np.r_[breaks, len(stamp)]):
        if end - start < 10:
            continue
        xyz = []
        for values in axes:
            xyz.append(np.concatenate([values[i, :count[i]] * scale[i] for i in range(start, end)]))
        first = stamp[start] - (count[start] - 1) * period
        time = first + np.arange(len(xyz[0])) * period
        yield time, np.stack(xyz), 1e6 / period


def rpm_data(data):
    keys = sorted(key for key in data if key.endswith("esc_rpm"))
    rpm = np.stack([np.abs(data[key]) for key in keys], axis=1).astype(float)
    rpm = rpm[:, np.nanmax(rpm, axis=0) > 0]
    return data["timestamp"], rpm


def interp(data, field, time, default=np.nan):
    if data is None or field not in data or len(data["timestamp"]) == 0:
        return np.full_like(np.asarray(time, float), default, dtype=float)
    return np.interp(time, data["timestamp"], data[field], left=default)


def airborne(land, time):
    return interp(land, "landed", time, 1.0) <= 0.5


def hovering(land, position, time):
    vx = interp(position, "vx", time)
    vy = interp(position, "vy", time)
    vz = interp(position, "vz", time)
    return airborne(land, time) & (np.hypot(vx, vy) < 1.0) & (np.abs(vz) < 0.5)


def order_bands(fmotor, nyquist):
    lo1 = np.min(fmotor) * (1.0 - TOL)
    hi1 = np.max(fmotor) * (1.0 + TOL)
    result = []
    for order in range(1, KMAX + 1):
        lo, hi = order * lo1, order * hi1
        separate = (order == 1 or lo > (order - 1) * hi1) and (order == KMAX or hi < (order + 1) * lo1)
        result.append((lo, hi, separate and hi <= min(FMAX, nyquist)))
    return result


def band_amplitudes(spectrum, freq, length, window_power, fmotor, fs):
    values = np.full(KMAX, np.nan)
    for order, (lo, hi, valid) in enumerate(order_bands(fmotor, fs / 2), 1):
        mask = (freq >= lo) & (freq <= hi)
        if valid and np.any(mask):
            per_axis = np.sqrt(2.0 * spectrum[:, mask].sum(axis=1) / (length * length * window_power))
            values[order - 1] = np.degrees(np.linalg.norm(per_axis))
    return values


def peak_hit(peak, fmotor):
    targets = np.asarray(fmotor)[:, None] * np.arange(1, KMAX + 1)[None, :]
    return bool(np.any(np.abs(peak - targets) <= TOL * targets))


def random_hit_probability(fmotor):
    intervals = []
    for motor in fmotor:
        for order in range(1, KMAX + 1):
            target = order * motor
            lo, hi = max(FMIN, target * (1 - TOL)), min(FMAX, target * (1 + TOL))
            if hi > lo:
                intervals.append((lo, hi))
    intervals.sort()
    merged = []
    for lo, hi in intervals:
        if not merged or lo > merged[-1][1]:
            merged.append([lo, hi])
        else:
            merged[-1][1] = max(merged[-1][1], hi)
    return sum(hi - lo for lo, hi in merged) / (FMAX - FMIN)


def extract_windows(data, map_sum=None, map_count=None):
    gyro, esc = data.get("sensor_gyro_fifo"), data.get("esc_status")
    if gyro is None or esc is None:
        return []
    etime, rpm = rpm_data(esc)
    land, position = data.get("vehicle_land_detected"), data.get("vehicle_local_position")
    rows = []
    for time, xyz, fs in fifo_blocks(gyro):
        length = int(WIN_S * fs)
        if length < 8:
            continue
        hann = np.hanning(length)
        window_power = np.mean(hann ** 2)
        freq = np.fft.rfftfreq(length, 1.0 / fs)
        for start in range(0, xyz.shape[1] - length + 1, length):
            sample_time = time[start:start + length]
            centre = float(np.mean(sample_time))
            if not airborne(land, np.array([centre]))[0]:
                continue
            emask = (etime >= sample_time[0]) & (etime <= sample_time[-1])
            if emask.sum() < 3:
                continue
            mean_rpm = np.nanmean(rpm[emask], axis=0)
            if len(mean_rpm) == 0 or np.nanmin(mean_rpm) < 500:
                continue
            fmotor = mean_rpm / 60.0
            segment = xyz[:, start:start + length]
            segment = segment - segment.mean(axis=1, keepdims=True)
            spectrum = np.abs(np.fft.rfft(segment * hann, axis=1)) ** 2
            band = (freq >= FMIN) & (freq <= min(FMAX, fs / 2))
            if not np.any(band):
                continue
            combined = spectrum.sum(axis=0)
            peak = float(freq[band][np.argmax(combined[band])])
            is_hover = bool(hovering(land, position, np.array([centre]))[0])
            amplitudes = band_amplitudes(spectrum, freq, length, window_power, fmotor, fs)
            row = {"time_us": centre, "rotor_hz": float(np.mean(fmotor)), "fmotor": fmotor,
                   "peak_hz": peak, "hit": peak_hit(peak, fmotor),
                   "random_probability": random_hit_probability(fmotor), "hover": is_hover,
                   "amplitudes": amplitudes, "fs_hz": fs}
            rows.append(row)
            if map_sum is not None:
                xbin = int(np.searchsorted(MAP_X_EDGES, row["rotor_hz"], side="right") - 1)
                if 0 <= xbin < len(MAP_X_EDGES) - 1:
                    psd = 2.0 * combined / (length * fs * window_power)
                    values = np.interp(MAP_FREQ, freq, psd, left=np.nan, right=np.nan)
                    valid = np.isfinite(values)
                    map_sum[xbin, valid] += values[valid]
                    map_count[xbin, valid] += 1
    return rows


def metric_rows(data):
    status = data.get("vehicle_imu_status")
    if status is None:
        return {}, {name: 0 for name in METRICS}
    time = status["timestamp"]
    mask = hovering(data.get("vehicle_land_detected"), data.get("vehicle_local_position"), time)
    values, counts = {}, {}
    for name in METRICS:
        present = name in status
        selected = np.asarray(status[name][mask], float) if present else np.array([])
        selected = selected[np.isfinite(selected)]
        counts[name] = len(selected)
        values[name] = float(np.median(selected)) if len(selected) else np.nan
    return values, counts


def summarize_amplitudes(windows):
    result = np.full(KMAX, np.nan)
    hover = [row["amplitudes"] for row in windows if row["hover"]]
    if hover:
        values = np.stack(hover)
        for order in range(KMAX):
            finite = values[:, order][np.isfinite(values[:, order])]
            if len(finite):
                result[order] = np.median(finite)
    return result


def summarize_slopes(windows):
    result = []
    all_rotor = np.asarray([row["rotor_hz"] for row in windows])
    if len(all_rotor) < 10:
        return result
    span = np.percentile(all_rotor, 95) / np.percentile(all_rotor, 5)
    if span < 1.2:
        return result
    for order in (1, 2):
        pairs = [(row["rotor_hz"], row["amplitudes"][order - 1]) for row in windows
                 if np.isfinite(row["amplitudes"][order - 1]) and row["amplitudes"][order - 1] > 0]
        if len(pairs) < 10:
            continue
        pairs = np.asarray(pairs)
        slope = np.polyfit(np.log(pairs[:, 0]), np.log(pairs[:, 1]), 1)[0]
        result.append((order, float(slope), len(pairs), float(span)))
    return result


def append_metric_output(rows, missing, source, group, name, values, counts):
    absent = []
    for metric in METRICS:
        if counts[metric]:
            rows.append({"source": source, "group": group, "log": name, "metric": metric,
                         "value": values[metric], "hover_samples": counts[metric]})
        else:
            absent.append(metric)
    missing.append({"source": source, "group": group, "log": name, "missing_metrics": ";".join(absent)})


def distribution(rows, metric, source=None, group=None):
    values = [float(row["value"]) for row in rows if row["metric"] == metric
              and (source is None or row["source"] == source) and (group is None or row["group"] == group)]
    return np.asarray(values, float)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", type=Path, default=DEFAULT_IDS)
    parser.add_argument("--logs-dir", type=Path, required=True)
    parser.add_argument("--athrill-logs-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    ids = [row["log_id"] for row in csv.DictReader(args.ids.open())]

    map_sum = np.zeros((len(MAP_X_EDGES) - 1, len(MAP_FREQ)))
    map_count = np.zeros_like(map_sum, dtype=int)
    real_windows, windows_by_log = [], {}
    real_amplitudes, exponent_rows = {}, []
    metric_output, missing_output, settings = [], [], []

    topics = ["sensor_gyro_fifo", "esc_status", "vehicle_land_detected", "vehicle_local_position", "vehicle_imu_status"]
    for index, log_id in enumerate(ids, 1):
        path = find_log(args.logs_dir, log_id)
        ulog = ULog(str(path), topics)
        data = datasets(ulog)
        windows = extract_windows(data, map_sum, map_count)
        for row in windows:
            row.update(source="real", log_id=log_id, run="")
        windows_by_log[log_id] = windows
        real_windows.extend(windows)
        real_amplitudes[log_id] = summarize_amplitudes(windows)
        for order, slope, count, span in summarize_slopes(windows):
            exponent_rows.append({"log_id": log_id, "order": order, "slope": slope,
                                  "windows": count, "rotor_p95_over_p5": span})
        values, counts = metric_rows(data)
        append_metric_output(metric_output, missing_output, "real", "real", log_id, values, counts)
        if log_id == FIT_LOG:
            for name in PARAMETERS:
                if name in ulog.initial_parameters:
                    settings.append({"source": "real", "run": log_id, "parameter": name,
                                     "value": ulog.initial_parameters[name]})
        print(f"real {index:02d}/{len(ids)} {log_id[:8]}: {len(windows)} airborne windows", flush=True)
        del ulog, data
        gc.collect()

    # A fixed half-dataset cyclic shift pairs every real peak with motor speed from another time.
    shift = len(real_windows) // 2
    motors = [row["fmotor"] for row in real_windows]
    for index, row in enumerate(real_windows):
        row["wrong_time"] = peak_hit(row["peak_hz"], motors[(index + shift) % len(motors)])

    athrill_windows, athrill_amplitudes = [], {}
    for group in ("off", "vib-nonotch", "vib-notch"):
        for number in (1, 2, 3):
            run = f"{group}-{number}"
            path = args.athrill_logs_dir / f"{run}.ulg"
            ulog = ULog(str(path), topics)
            data = datasets(ulog)
            windows = extract_windows(data)
            for row in windows:
                row.update(source="athrill", log_id="", run=run, wrong_time=False)
            if group == "vib-nonotch":
                athrill_windows.extend(windows)
                athrill_amplitudes[run] = summarize_amplitudes(windows)
            values, counts = metric_rows(data)
            append_metric_output(metric_output, missing_output, "athrill", group, run, values, counts)
            for name in PARAMETERS:
                if name in ulog.initial_parameters:
                    settings.append({"source": "athrill", "run": run, "parameter": name,
                                     "value": ulog.initial_parameters[name]})
            print(f"athrill {run}: {len(windows)} airborne windows", flush=True)
            del ulog, data
            gc.collect()

    window_rows = []
    for row in real_windows + athrill_windows:
        window_rows.append({"source": row["source"], "log_id": row["log_id"], "run": row["run"],
                            "rotor_hz": row["rotor_hz"], "peak_hz": row["peak_hz"], "hit": int(row["hit"]),
                            "wrong_time_hit": int(row["wrong_time"]),
                            "random_hit_probability": row["random_probability"], "fifo_hz": row["fs_hz"]})
    write_csv(args.out / "order_windows.csv", window_rows)

    true_rate = np.mean([row["hit"] for row in real_windows])
    random_rate = np.mean([row["random_probability"] for row in real_windows])
    wrong_rate = np.mean([row["wrong_time"] for row in real_windows])
    logs_2x = 0
    for windows in windows_by_log.values():
        if np.mean([row["hit"] for row in windows]) >= 2 * np.mean([row["random_probability"] for row in windows]):
            logs_2x += 1
    order_summary = [{"real_windows": len(real_windows), "real_hit_rate": true_rate,
                      "random_hit_rate": random_rate, "wrong_time_hit_rate": wrong_rate,
                      "hit_over_random": true_rate / random_rate, "logs_at_least_2x_random": logs_2x,
                      "real_logs": len(ids), "athrill_windows": len(athrill_windows),
                      "athrill_hit_rate": np.mean([row["hit"] for row in athrill_windows])}]
    write_csv(args.out / "order_summary.csv", order_summary)

    map_rows = []
    for xi in range(map_sum.shape[0]):
        for yi, frequency in enumerate(MAP_FREQ):
            if map_count[xi, yi]:
                level = np.degrees(np.sqrt(map_sum[xi, yi] / map_count[xi, yi]))
                map_rows.append({"rotor_hz": (MAP_X_EDGES[xi] + MAP_X_EDGES[xi + 1]) / 2,
                                 "vibration_hz": frequency, "level_deg_s_sqrt_hz": level,
                                 "windows": map_count[xi, yi]})
    write_csv(args.out / "order_map.csv", map_rows)

    amplitude_rows = []
    athrill_matrix = np.stack(list(athrill_amplitudes.values()))
    real_matrix = np.stack(list(real_amplitudes.values()))
    for order in range(1, KMAX + 1):
        rv = real_matrix[:, order - 1]
        av = athrill_matrix[:, order - 1]
        rv, av = rv[np.isfinite(rv)], av[np.isfinite(av)]
        real_fit = real_amplitudes[FIT_LOG][order - 1]
        athrill_median = float(np.median(av))
        ratio = athrill_median / real_fit
        amplitude_rows.append({"order": order, "real_407292d1_deg_s": real_fit,
                               "real_flight_min_deg_s": np.min(rv), "real_flight_max_deg_s": np.max(rv),
                               "real_flights": len(rv), "athrill_median_deg_s": athrill_median,
                               "athrill_min_deg_s": np.min(av), "athrill_max_deg_s": np.max(av),
                               "athrill_runs": len(av), "athrill_over_real": ratio,
                               "criterion": "0.70-1.43" if order <= 5 else "not fitted",
                               "pass": "yes" if order <= 5 and 0.70 <= ratio <= 1.43 else ("no" if order <= 5 else "n/a")})
    write_csv(args.out / "amplitude.csv", amplitude_rows)
    write_csv(args.out / "exponent.csv", exponent_rows)
    exponent_summary = []
    for order, fitted in ((1, 2.61), (2, 3.13)):
        values = np.array([row["slope"] for row in exponent_rows if row["order"] == order])
        exponent_summary.append({"order": order, "flights": len(values), "median": np.median(values),
                                 "q25": np.percentile(values, 25), "q75": np.percentile(values, 75),
                                 "model_default": 2.0, "fitted_model": fitted})
    write_csv(args.out / "exponent_summary.csv", exponent_summary)

    write_csv(args.out / "px4_metrics.csv", metric_output)
    px4_summary = []
    real_fit_metrics = {row["metric"]: float(row["value"]) for row in metric_output
                        if row["source"] == "real" and row["log"] == FIT_LOG}
    for metric in METRICS:
        real = distribution(metric_output, metric, source="real")
        if not len(real) or metric not in real_fit_metrics:
            continue
        record = {"metric": metric, "real_407292d1": real_fit_metrics[metric], "real_min": np.min(real),
                  "real_max": np.max(real), "real_logs": len(real)}
        for group in ("off", "vib-nonotch", "vib-notch"):
            values = distribution(metric_output, metric, source="athrill", group=group)
            record[f"{group}_median"] = np.median(values) if len(values) else np.nan
            record[f"{group}_min"] = np.min(values) if len(values) else np.nan
            record[f"{group}_max"] = np.max(values) if len(values) else np.nan
            record[f"{group}_runs"] = len(values)
        ratio = record["vib-nonotch_median"] / record["real_407292d1"]
        record["vib-nonotch_over_real"] = ratio
        record["pass_factor_2"] = "yes" if 0.5 <= ratio <= 2.0 else "no"
        px4_summary.append(record)
    write_csv(args.out / "px4_summary.csv", px4_summary)
    write_csv(args.out / "missing_metrics.csv", missing_output)
    write_csv(args.out / "settings.csv", settings)
    write_csv(args.out / "measurement_meta.csv", [
        {"item": "window_seconds", "value": WIN_S}, {"item": "peak_band_hz", "value": f"{FMIN:g}-{FMAX:g}"},
        {"item": "order_tolerance", "value": TOL}, {"item": "hover_horizontal_speed_max_m_s", "value": 1.0},
        {"item": "hover_abs_vertical_speed_max_m_s", "value": 0.5},
        {"item": "wrong_time_pairing", "value": "cyclic shift by half of all real windows"},
    ])


if __name__ == "__main__":
    main()
