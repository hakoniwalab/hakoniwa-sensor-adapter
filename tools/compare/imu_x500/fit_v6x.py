#!/usr/bin/env python3
"""Fit the X500 V6X gyro and accelerometer rotor-order vibration model."""
import argparse
import gc
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from pyulog import ULog

from measure import (FMAX, FMIN, KMAX, WIN_S, datasets, fifo_blocks, find_log,
                     hovering, order_bands, rpm_data, write_csv)

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE.parents[2] / "docs/evaluation/imu-x500/data"
FIT_LOG = "75ac4a47-a73a-44ea-8016-24d79a164147"
LOG_IDS = (
    "14600267-e385-4ee0-b647-52a9be62dfea",
    "31fd374f-06ab-4aa5-9c48-8836c6e9cd0c",
    "4315b242-5f85-4f29-9cb4-93e7738eebed",
    FIT_LOG,
    "9105b951-5bf6-4918-99bd-618cd1a5f4ba",
    "a792ed5c-d83a-4e73-8b8f-7307578f7a83",
    "c8082ca8-c79b-4e9b-814a-4c1454546181",
    "d71cd659-1c9c-49e7-859a-79664657c857",
)
SENSORS = {"gyro": "sensor_gyro_fifo", "accel": "sensor_accel_fifo"}
AXES = "xyz"


def rms_from_mask(spectrum, mask, length, window_power):
    """Hann-power-corrected per-axis RMS for positive-frequency bins."""
    if not np.any(mask):
        return np.full(3, np.nan)
    return np.sqrt(2.0 * spectrum[:, mask].sum(axis=1) /
                   (length * length * window_power))


def extract_sensor_windows(fifo, esc, land, position):
    """Measure order and non-order broadband RMS in contiguous hover windows."""
    etime, rpm = rpm_data(esc)
    rows = []
    for time, xyz, fs in fifo_blocks(fifo):
        length = int(WIN_S * fs)
        if length < 8:
            continue
        hann = np.hanning(length)
        window_power = np.mean(hann ** 2)
        freq = np.fft.rfftfreq(length, 1.0 / fs)
        analysis_band = (freq >= FMIN) & (freq <= min(FMAX, fs / 2))
        for start in range(0, xyz.shape[1] - length + 1, length):
            sample_time = time[start:start + length]
            centre = float(np.mean(sample_time))
            if not hovering(land, position, np.array([centre]))[0]:
                continue
            emask = (etime >= sample_time[0]) & (etime <= sample_time[-1])
            if emask.sum() < 3:
                continue
            mean_rpm = np.nanmean(rpm[emask], axis=0)
            if len(mean_rpm) != 4 or not np.all(np.isfinite(mean_rpm)) or np.min(mean_rpm) < 500:
                continue
            fmotor = mean_rpm / 60.0
            segment = xyz[:, start:start + length]
            segment = segment - segment.mean(axis=1, keepdims=True)
            spectrum = np.abs(np.fft.rfft(segment * hann, axis=1)) ** 2
            order_rms = np.full((KMAX, 3), np.nan)
            order_masks = []
            for order, (lo, hi, valid) in enumerate(order_bands(fmotor, fs / 2)):
                mask = (freq >= lo) & (freq <= hi)
                if valid and np.any(mask):
                    order_rms[order] = rms_from_mask(spectrum, mask, length, window_power)
                    order_masks.append(mask)
            non_order = analysis_band.copy()
            for mask in order_masks:
                non_order &= ~mask
            line_50 = (freq >= 46.0) & (freq <= 54.0)
            line_620_660 = (freq >= 620.0) & (freq <= 660.0)
            rows.append({
                "time_us": centre,
                "fs_hz": fs,
                "rpm": mean_rpm,
                "mean_rpm": float(np.mean(mean_rpm)),
                "order_rms": order_rms,
                "broadband_rms": rms_from_mask(spectrum, non_order, length, window_power),
                "line_50_rms": rms_from_mask(spectrum, line_50, length, window_power),
                "line_620_660_rms": rms_from_mask(spectrum, line_620_660, length, window_power),
            })
    return rows


def common_exponent(windows, order):
    """Shared slope with separate gyro/accel intercepts in log-log space."""
    grouped = defaultdict(list)
    for row in windows:
        value = float(np.linalg.norm(row["order_rms"][order - 1]))
        if np.isfinite(value) and value > 0 and row["mean_rpm"] > 0:
            grouped[row["sensor"]].append((np.log(row["mean_rpm"] / 60.0), np.log(value)))
    numerator = denominator = 0.0
    for pairs in grouped.values():
        values = np.asarray(pairs)
        dx = values[:, 0] - np.mean(values[:, 0])
        dy = values[:, 1] - np.mean(values[:, 1])
        numerator += float(np.dot(dx, dy))
        denominator += float(np.dot(dx, dx))
    slope = numerator / denominator if denominator > 0 else np.nan
    return float(max(0.0, slope)), {sensor: len(pairs) for sensor, pairs in grouped.items()}


def per_rotor_scale(rpm, reference_rpm, exponent):
    q = (np.asarray(rpm, float) / reference_rpm) ** exponent
    return float(np.sqrt(np.sum(q ** 2) / 2.0))


def median_finite(values):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    return float(np.median(values)) if len(values) else np.nan


def percentile_finite(values, percentile):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, percentile)) if len(values) else np.nan


def window_csv_rows(windows):
    for row in windows:
        out = {
            "log_id": row["log_id"], "sensor": row["sensor"], "time_us": row["time_us"],
            "fifo_hz": row["fs_hz"], "mean_rpm": row["mean_rpm"],
        }
        for index, rpm in enumerate(row["rpm"]):
            out[f"rpm_{index}"] = rpm
        for order in range(1, KMAX + 1):
            for axis, value in zip(AXES, row["order_rms"][order - 1]):
                out[f"order_{order}_{axis}_rms"] = value
        for axis, value in zip(AXES, row["broadband_rms"]):
            out[f"broadband_{axis}_rms"] = value
        for axis, value in zip(AXES, row["line_50_rms"]):
            out[f"line_46_54_{axis}_rms"] = value
        for axis, value in zip(AXES, row["line_620_660_rms"]):
            out[f"line_620_660_{axis}_rms"] = value
        yield out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    topics = list(SENSORS.values()) + ["esc_status", "vehicle_land_detected", "vehicle_local_position"]
    all_windows = []
    for index, log_id in enumerate(LOG_IDS, 1):
        ulog = ULog(str(find_log(args.logs_dir, log_id)), topics)
        data = datasets(ulog)
        counts = []
        for sensor, topic in SENSORS.items():
            if topic not in data:
                raise RuntimeError(f"{log_id}: missing {topic}")
            windows = extract_sensor_windows(data[topic], data["esc_status"],
                                             data.get("vehicle_land_detected"),
                                             data.get("vehicle_local_position"))
            for row in windows:
                row.update(log_id=log_id, sensor=sensor)
            all_windows.extend(windows)
            counts.append(f"{sensor}={len(windows)}")
        print(f"{index}/{len(LOG_IDS)} {log_id[:8]}: {', '.join(counts)} hover windows", flush=True)
        del ulog, data
        gc.collect()

    exponents = {}
    exponent_rows = []
    for order in range(1, KMAX + 1):
        exponent, counts = common_exponent(all_windows, order)
        exponents[order] = exponent
        frequencies = [row["mean_rpm"] / 60.0 for row in all_windows
                       if np.isfinite(np.linalg.norm(row["order_rms"][order - 1]))]
        exponent_rows.append({
            "order": order, "exponent": exponent,
            "gyro_windows": counts.get("gyro", 0), "accel_windows": counts.get("accel", 0),
            "rotor_hz_min": min(frequencies), "rotor_hz_max": max(frequencies),
            "rpm_min": 60.0 * min(frequencies), "rpm_max": 60.0 * max(frequencies),
        })

    fit_accel = [row for row in all_windows if row["log_id"] == FIT_LOG and row["sensor"] == "accel"]
    if not fit_accel:
        raise RuntimeError("fit flight has no accelerometer hover windows")
    reference_rpm = float(np.median([row["mean_rpm"] for row in fit_accel]))

    amplitudes = {}
    amplitude_rows = []
    for order in range(1, KMAX + 1):
        for sensor in SENSORS:
            selected = [row for row in all_windows if row["log_id"] == FIT_LOG and row["sensor"] == sensor]
            normalized = [[] for _ in AXES]
            for row in selected:
                scale = per_rotor_scale(row["rpm"], reference_rpm, exponents[order])
                for axis in range(3):
                    normalized[axis].append(row["order_rms"][order - 1, axis] / scale)
            values = np.array([median_finite(axis_values) for axis_values in normalized])
            amplitudes[(sensor, order)] = values
            amplitude_rows.append({
                "order": order, "sensor": sensor, "exponent": exponents[order],
                "reference_rpm": reference_rpm, "fit_windows": len(selected),
                **{f"{axis}_amplitude": value for axis, value in zip(AXES, values)},
            })

    validation_rows = []
    held_out = [row for row in all_windows if row["log_id"] != FIT_LOG]
    for log_id in LOG_IDS:
        if log_id == FIT_LOG:
            continue
        for sensor in SENSORS:
            selected = [row for row in held_out if row["log_id"] == log_id and row["sensor"] == sensor]
            for order in range(1, KMAX + 1):
                ratios = []
                for row in selected:
                    predicted_axes = amplitudes[(sensor, order)] * per_rotor_scale(
                        row["rpm"], reference_rpm, exponents[order])
                    measured = float(np.linalg.norm(row["order_rms"][order - 1]))
                    predicted = float(np.linalg.norm(predicted_axes))
                    if np.isfinite(measured) and measured > 0:
                        ratios.append(predicted / measured)
                validation_rows.append({
                    "log_id": log_id, "sensor": sensor, "order": order, "windows": len(ratios),
                    "predicted_over_measured_median": median_finite(ratios),
                    "predicted_over_measured_q25": percentile_finite(ratios, 25),
                    "predicted_over_measured_q75": percentile_finite(ratios, 75),
                })

    validation_summary = []
    for sensor in SENSORS:
        for order in range(1, KMAX + 1):
            ratios = []
            flights = set()
            for row in held_out:
                if row["sensor"] != sensor:
                    continue
                predicted_axes = amplitudes[(sensor, order)] * per_rotor_scale(
                    row["rpm"], reference_rpm, exponents[order])
                measured = float(np.linalg.norm(row["order_rms"][order - 1]))
                if np.isfinite(measured) and measured > 0:
                    ratios.append(float(np.linalg.norm(predicted_axes)) / measured)
                    flights.add(row["log_id"])
            validation_summary.append({
                "sensor": sensor, "order": order, "flights": len(flights), "windows": len(ratios),
                "predicted_over_measured_median": median_finite(ratios),
                "predicted_over_measured_q25": percentile_finite(ratios, 25),
                "predicted_over_measured_q75": percentile_finite(ratios, 75),
            })

    flight_rows = []
    for log_id in LOG_IDS:
        for sensor in SENSORS:
            selected = [row for row in all_windows if row["log_id"] == log_id and row["sensor"] == sensor]
            flight_rows.append({
                "log_id": log_id, "sensor": sensor, "hover_windows": len(selected),
                "fifo_hz_median": median_finite([row["fs_hz"] for row in selected]),
                "mean_rpm_min": min(row["mean_rpm"] for row in selected),
                "mean_rpm_median": median_finite([row["mean_rpm"] for row in selected]),
                "mean_rpm_max": max(row["mean_rpm"] for row in selected),
                **{f"broadband_{axis}_rms_median": median_finite(
                    [row["broadband_rms"][index] for row in selected])
                   for index, axis in enumerate(AXES)},
                **{f"line_46_54_{axis}_rms_median": median_finite(
                    [row["line_50_rms"][index] for row in selected])
                   for index, axis in enumerate(AXES)},
                **{f"line_620_660_{axis}_rms_median": median_finite(
                    [row["line_620_660_rms"][index] for row in selected])
                   for index, axis in enumerate(AXES)},
            })

    model = {
        "referenceRpm": reference_rpm,
        "harmonics": [{
            "order": order,
            "exponent": exponents[order],
            "accelAmplitudeMps2": amplitudes[("accel", order)].tolist(),
            "gyroAmplitudeRadps": amplitudes[("gyro", order)].tolist(),
        } for order in range(1, KMAX + 1)],
    }
    with (args.out / "fit_v6x.json").open("w") as handle:
        json.dump(model, handle, indent=2)
        handle.write("\n")
    print(f"wrote {args.out / 'fit_v6x.json'}")
    write_csv(args.out / "fit_v6x_windows.csv", window_csv_rows(all_windows))
    write_csv(args.out / "fit_v6x_flights.csv", flight_rows)
    write_csv(args.out / "fit_v6x_exponents.csv", exponent_rows)
    write_csv(args.out / "fit_v6x_amplitudes.csv", amplitude_rows)
    write_csv(args.out / "fit_v6x_validation.csv", validation_rows)
    write_csv(args.out / "fit_v6x_validation_summary.csv", validation_summary)


if __name__ == "__main__":
    main()
