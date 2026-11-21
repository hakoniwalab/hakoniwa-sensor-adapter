# X500 IMU measurement tools

The inputs are public PX4 ULogs, nine original athrill3 closed-loop ULogs, and six V6X-fit athrill3 ULogs. The public IDs, source, and licence are in
`docs/evaluation/imu-x500/data/flight_ids.csv`; ULogs themselves are never stored in this repository.

```sh
# Optional: copy local logs into an external cache (without --logs-dir they are downloaded).
python tools/compare/imu_x500/fetch_logs.py \
  --logs-dir /Users/tmori/project/business-pack/work/imu-px4-logs \
  --cache-dir ~/.cache/hakoniwa-imu-x500

# Measure the original B1–B3 data. --logs-dir may be a cache or the parent of logs/ and logs_vib/.
python tools/compare/imu_x500/measure.py \
  --logs-dir /Users/tmori/project/business-pack/work/imu-px4-logs \
  --athrill-logs-dir /Users/tmori/project/business-pack/work/athrill-vib/run6c/ulog \
  --out docs/evaluation/imu-x500/data

# Fit gyro and accelerometer vibration from the eight PX4_FMU_V6X flights.
python tools/compare/imu_x500/fit_v6x.py \
  --logs-dir /Users/tmori/project/business-pack/work/imu-px4-logs \
  --out docs/evaluation/imu-x500/data

# Measure B7, B8, and corrected C1 from the fit CSV and the new closed-loop logs.
python tools/compare/imu_x500/measure_phase2.py \
  --logs-dir /Users/tmori/project/business-pack/work/imu-px4-logs \
  --athrill-logs-dir /Users/tmori/project/business-pack/work/athrill-vib/run6c/ulog \
  --data docs/evaluation/imu-x500/data

# Draw both languages from the derived CSV files.
python tools/compare/imu_x500/cards_imu_x500.py --data docs/evaluation/imu-x500/data \
  --out docs/evaluation/imu-x500 --lang ja
python tools/compare/imu_x500/cards_imu_x500.py --data docs/evaluation/imu-x500/data \
  --out docs/evaluation/imu-x500 --lang en
```

This writes the drone-pro model as `fit_v6x.json`, the per-window order and non-order broadband measurements, and
CSV summaries of the fit and seven-flight validation. The shared exponent for each order is a pooled log-log slope
with separate gyro and accelerometer intercepts; this keeps their different units from affecting the slope.
`measure_phase2.py` retains every per-flight/per-run accelerometer result, every PX4 IMU instance and device ID, the
power-weighted phase summaries, and the plotted phase trajectories. C1 comparisons use only nominally rate-matched
800 Hz instances.

The raw FIFO sample period comes from `timestamp_sample`, and a 0.5 s FFT window never crosses a FIFO gap. Airborne
state comes from `vehicle_land_detected`. Hover additionally means horizontal speed below 1 m/s and absolute vertical
speed below 0.5 m/s. Spectral amplitudes are Hann-corrected RMS values.
