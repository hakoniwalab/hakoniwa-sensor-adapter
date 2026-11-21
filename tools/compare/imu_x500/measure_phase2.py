#!/usr/bin/env python3
"""Measure the V6X accelerometer, x-y phase, and per-instance PX4 metric cards."""
import argparse
import csv
from pathlib import Path

import numpy as np
from pyulog import ULog

from fit_v6x import AXES, FIT_LOG as V6X_LOG, LOG_IDS, extract_sensor_windows, median_finite
from measure import datasets, fifo_blocks, find_log, hovering, order_bands, rpm_data, write_csv


HERE = Path(__file__).resolve().parent
DEFAULT_DATA = HERE.parents[2] / "docs/evaluation/imu-x500/data"
ARK_LOG = "407292d1-cb50-48b7-a13e-142a50245044"
METRICS = ("gyro_vibration_metric", "accel_vibration_metric", "delta_angle_coning_metric")
DEVICE_NAMES = {
    0x26: "ICM-42688-P",
    0x2B: "IIM-42652",
    0x34: "ICM-45686",
    0x41: "BMI055 accel",
    0x42: "BMI055 gyro",
    0x58: "ADIS16470",
}


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def finite(values):
    values = np.asarray(values, float)
    return values[np.isfinite(values)]


def topic_instances(ulog, name):
    return sorted(((item.multi_id, item.data) for item in ulog.data_list if item.name == name), key=lambda item: item[0])


def device_name(device_id):
    return DEVICE_NAMES.get((int(device_id) >> 16) & 0xFF, "unknown")


def nominal_rate(rate):
    return int(round(float(rate) / 100.0) * 100)


def acceleration_rows(data_dir, athrill_dir):
    """Per-flight/run median of the three-axis magnitude in every accelerometer order band."""
    windows = read_csv(data_dir / "fit_v6x_windows.csv")
    flight_rows = []
    real = {}
    for log_id in LOG_IDS:
        selected = [row for row in windows if row["log_id"] == log_id and row["sensor"] == "accel"]
        real[log_id] = []
        for order in range(1, 7):
            values = finite([np.linalg.norm([float(row[f"order_{order}_{axis}_rms"]) for axis in AXES])
                             for row in selected])
            value = median_finite(values)
            real[log_id].append(value)
            flight_rows.append({"source": "real", "group": "fit" if log_id == V6X_LOG else "held-out",
                                "log": log_id, "order": order, "band_rms_m_s2": value,
                                "hover_windows": len(values)})

    simulated = {}
    topics = ["sensor_accel_fifo", "esc_status", "vehicle_land_detected", "vehicle_local_position"]
    for group in ("v6x-vib", "rot-vib"):
        simulated[group] = []
        for number in (1, 2, 3):
            run = f"{group}-{number}"
            ulog = ULog(str(athrill_dir / f"{run}.ulg"), topics)
            data = datasets(ulog)
            measured = extract_sensor_windows(data["sensor_accel_fifo"], data["esc_status"],
                                               data.get("vehicle_land_detected"),
                                               data.get("vehicle_local_position"))
            values_by_order = []
            for order in range(1, 7):
                values = finite([np.linalg.norm(row["order_rms"][order - 1]) for row in measured])
                value = median_finite(values)
                values_by_order.append(value)
                flight_rows.append({"source": "athrill", "group": group, "log": run, "order": order,
                                    "band_rms_m_s2": value, "hover_windows": len(values)})
            simulated[group].append(values_by_order)

    held_out = np.asarray([real[log_id] for log_id in LOG_IDS if log_id != V6X_LOG], float)
    summary = []
    for order in range(1, 7):
        record = {"order": order, "real_75ac4a47_m_s2": real[V6X_LOG][order - 1],
                  "held_out_min_m_s2": np.nanmin(held_out[:, order - 1]),
                  "held_out_max_m_s2": np.nanmax(held_out[:, order - 1]), "held_out_flights": 7}
        for group in ("v6x-vib", "rot-vib"):
            values = finite(np.asarray(simulated[group])[:, order - 1])
            prefix = "in_phase" if group == "v6x-vib" else "rotating"
            record[f"{prefix}_median_m_s2"] = np.median(values)
            record[f"{prefix}_min_m_s2"] = np.min(values)
            record[f"{prefix}_max_m_s2"] = np.max(values)
            ratio = record[f"{prefix}_median_m_s2"] / record["real_75ac4a47_m_s2"]
            record[f"{prefix}_over_real"] = ratio
            record[f"{prefix}_pass"] = "yes" if 0.70 <= ratio <= 1.43 else "no"
        summary.append(record)
    return flight_rows, summary


def per_instance_metrics(path, source, group, log_name):
    topics = ["vehicle_imu_status", "vehicle_land_detected", "vehicle_local_position"]
    ulog = ULog(str(path), topics)
    primary = datasets(ulog)
    rows = []
    for instance, status in topic_instances(ulog, "vehicle_imu_status"):
        mask = hovering(primary.get("vehicle_land_detected"), primary.get("vehicle_local_position"),
                        status["timestamp"])
        gyro_id = int(np.median(status["gyro_device_id"]))
        accel_id = int(np.median(status["accel_device_id"]))
        gyro_rate = float(np.median(status["gyro_rate_hz"][mask]))
        accel_rate = float(np.median(status["accel_rate_hz"][mask]))
        for metric in METRICS:
            values = finite(status[metric][mask])
            rows.append({"source": source, "group": group, "log": log_name, "instance": instance,
                         "metric": metric, "value": median_finite(values), "hover_samples": len(values),
                         "gyro_device_id": f"0x{gyro_id:06x}", "gyro_chip": device_name(gyro_id),
                         "gyro_rate_hz": gyro_rate, "gyro_rate_nominal_hz": nominal_rate(gyro_rate),
                         "accel_device_id": f"0x{accel_id:06x}", "accel_chip": device_name(accel_id),
                         "accel_rate_hz": accel_rate, "accel_rate_nominal_hz": nominal_rate(accel_rate)})
    return rows


def metric_rows(logs_dir, athrill_dir):
    rows = []
    rows.extend(per_instance_metrics(find_log(logs_dir, ARK_LOG), "real", "ark", ARK_LOG))
    rows.extend(per_instance_metrics(find_log(logs_dir, V6X_LOG), "real", "v6x", V6X_LOG))
    for group in ("vib-nonotch", "v6x-vib", "rot-vib"):
        for number in (1, 2, 3):
            run = f"{group}-{number}"
            rows.extend(per_instance_metrics(athrill_dir / f"{run}.ulg", "athrill", group, run))
    return rows


def summarize_metrics(rows):
    """Compare only processing-rate-matched instances; V6X has two real 800 Hz instances."""
    comparisons = (
        ("ARK / gyro-only fit", "ark", "vib-nonotch", {0, 1, 2}),
        ("V6X / in-phase", "v6x", "v6x-vib", {1, 2}),
        ("V6X / rotating", "v6x", "rot-vib", {1, 2}),
    )
    summary = []
    for label, real_group, sim_group, real_instances in comparisons:
        for metric in METRICS:
            real = finite([float(row["value"]) for row in rows
                           if row["source"] == "real" and row["group"] == real_group
                           and int(row["instance"]) in real_instances and row["metric"] == metric
                           and int(row["gyro_rate_nominal_hz"]) == 800])
            sim = finite([float(row["value"]) for row in rows
                          if row["source"] == "athrill" and row["group"] == sim_group
                          and int(row["instance"]) == 1 and row["metric"] == metric
                          and int(row["gyro_rate_nominal_hz"]) == 800])
            real_centre = float(real[0]) if real_group == "ark" else float(np.median(real))
            sim_centre = float(np.median(sim))
            ratio = sim_centre / real_centre
            summary.append({"comparison": label, "real_group": real_group, "sim_group": sim_group,
                            "metric": metric, "real_median": real_centre, "real_min": np.min(real),
                            "real_max": np.max(real), "real_instances": len(real),
                            "sim_median": sim_centre, "sim_min": np.min(sim), "sim_max": np.max(sim),
                            "sim_runs": len(sim), "sim_over_real": ratio,
                            "criterion": "0.5-2.0", "pass": "yes" if 0.5 <= ratio <= 2.0 else "no"})
    return summary


def phase_and_trajectory(path, condition, sensor):
    topic = f"sensor_{sensor}_fifo"
    ulog = ULog(str(path), [topic, "esc_status", "vehicle_land_detected", "vehicle_local_position"])
    data = datasets(ulog)
    etime, rpm = rpm_data(data["esc_status"])
    phase = {1: [[], []], 2: [[], []]}  # absolute phase and cross-power weight
    best = None
    for time, xyz, fs in fifo_blocks(data[topic]):
        length = int(0.5 * fs)
        window = np.hanning(length)
        freq = np.fft.rfftfreq(length, 1.0 / fs)
        for start in range(0, xyz.shape[1] - length + 1, length):
            sample_time = time[start:start + length]
            centre = float(np.mean(sample_time))
            if not hovering(data.get("vehicle_land_detected"), data.get("vehicle_local_position"), [centre])[0]:
                continue
            emask = (etime >= sample_time[0]) & (etime <= sample_time[-1])
            if emask.sum() < 3:
                continue
            mean_rpm = np.nanmean(rpm[emask], axis=0)
            if len(mean_rpm) != 4 or np.min(mean_rpm) < 500:
                continue
            fmotor = mean_rpm / 60.0
            segment = xyz[:, start:start + length].astype(float)
            segment -= segment.mean(axis=1, keepdims=True)
            transform = np.fft.rfft(segment * window, axis=1)
            for order, (lo, hi, valid) in enumerate(order_bands(fmotor, fs / 2), 1):
                if order > 2:
                    break
                mask = (freq >= lo) & (freq <= hi)
                if valid and np.any(mask):
                    cross = transform[0, mask] * np.conj(transform[1, mask])
                    phase[order][0].extend(np.abs(np.angle(cross)))
                    phase[order][1].extend(np.abs(transform[0, mask]) * np.abs(transform[1, mask]))

            # Preserve the x:y amplitude ratio while removing the uninteresting absolute scale.
            distance = abs(float(np.mean(mean_rpm)) - 5540.0)
            if best is None or distance < best[0]:
                centre_hz = float(np.mean(fmotor))
                raw_transform = np.fft.rfft(segment, axis=1)
                band = (freq >= 0.85 * centre_hz) & (freq <= 1.15 * centre_hz)
                filtered = np.fft.irfft(raw_transform * band[None, :], n=length, axis=1)
                turns = 4
                count = max(8, min(length, int(round(turns * fs / centre_hz))))
                first = (length - count) // 2
                xy = filtered[:2, first:first + count]
                scale = np.max(np.hypot(xy[0], xy[1]))
                best = (distance, centre_hz, fs, xy / scale)

    phase_rows = []
    for order in (1, 2):
        degrees = np.degrees(np.average(phase[order][0], weights=phase[order][1]))
        phase_rows.append({"condition": condition, "sensor": sensor, "order": order,
                           "weighted_abs_phase_deg": degrees, "spectral_bins": len(phase[order][0]),
                           "weight": np.sum(phase[order][1])})
    trajectory = []
    _, centre_hz, fs, xy = best
    for index, (x, y) in enumerate(zip(xy[0], xy[1])):
        trajectory.append({"condition": condition, "sensor": sensor, "sample": index,
                           "time_s": index / fs, "rotor_hz": centre_hz,
                           "x_normalized": x, "y_normalized": y})
    return phase_rows, trajectory


def phase_rows(logs_dir, athrill_dir):
    measurements, trajectories = [], []
    conditions = (
        ("real", [find_log(logs_dir, V6X_LOG)]),
        ("in-phase", [athrill_dir / f"v6x-vib-{number}.ulg" for number in (1, 2, 3)]),
        ("rotating", [athrill_dir / f"rot-vib-{number}.ulg" for number in (1, 2, 3)]),
    )
    for condition, paths in conditions:
        for sensor in ("gyro", "accel"):
            parts = []
            candidates = []
            for path in paths:
                measured, trajectory = phase_and_trajectory(path, condition, sensor)
                parts.extend(measured)
                candidates.append(trajectory)
            for order in (1, 2):
                selected = [row for row in parts if row["order"] == order]
                weight = np.array([float(row["weight"]) for row in selected])
                value = np.average([float(row["weighted_abs_phase_deg"]) for row in selected], weights=weight)
                measurements.append({"condition": condition, "sensor": sensor, "order": order,
                                     "weighted_abs_phase_deg": value,
                                     "spectral_bins": sum(int(row["spectral_bins"]) for row in selected),
                                     "logs": len(paths)})
            # The first run is a deterministic representative; phase values above use all three runs.
            trajectories.extend(candidates[0])
    return measurements, trajectories


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs-dir", type=Path, required=True)
    parser.add_argument("--athrill-logs-dir", type=Path, required=True)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    args = parser.parse_args()
    args.data.mkdir(parents=True, exist_ok=True)

    accel_flights, accel_summary = acceleration_rows(args.data, args.athrill_logs_dir)
    instances = metric_rows(args.logs_dir, args.athrill_logs_dir)
    metric_summary = summarize_metrics(instances)
    phases, trajectories = phase_rows(args.logs_dir, args.athrill_logs_dir)
    write_csv(args.data / "accel_flights.csv", accel_flights)
    write_csv(args.data / "accel_summary.csv", accel_summary)
    write_csv(args.data / "px4_instances.csv", instances)
    write_csv(args.data / "px4_instance_summary.csv", metric_summary)
    write_csv(args.data / "phase_summary.csv", phases)
    write_csv(args.data / "phase_trajectory.csv", trajectories)


if __name__ == "__main__":
    main()
