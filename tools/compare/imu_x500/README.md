# X500 IMU measurement tools

The inputs are public PX4 ULogs and nine athrill3 closed-loop ULogs. The public IDs, source, and licence are in
`docs/evaluation/imu-x500/data/flight_ids.csv`; ULogs themselves are never stored in this repository.

```sh
# Optional: copy local logs into an external cache (without --logs-dir they are downloaded).
python tools/compare/imu_x500/fetch_logs.py \
  --logs-dir /Users/tmori/project/business-pack/work/imu-px4-logs \
  --cache-dir ~/.cache/hakoniwa-imu-x500

# Measure all four cards. --logs-dir may be a cache or the parent of logs/ and logs_vib/.
python tools/compare/imu_x500/measure.py \
  --logs-dir /Users/tmori/project/business-pack/work/imu-px4-logs \
  --athrill-logs-dir /Users/tmori/project/business-pack/work/athrill-vib/run6c/ulog \
  --out docs/evaluation/imu-x500/data

# Draw both languages from those CSV files.
python tools/compare/imu_x500/cards_imu_x500.py --data docs/evaluation/imu-x500/data \
  --out docs/evaluation/imu-x500 --lang ja
python tools/compare/imu_x500/cards_imu_x500.py --data docs/evaluation/imu-x500/data \
  --out docs/evaluation/imu-x500 --lang en
```

The raw FIFO sample period comes from `timestamp_sample`, and a 0.5 s FFT window never crosses a FIFO gap. Airborne
state comes from `vehicle_land_detected`. Hover additionally means horizontal speed below 1 m/s and absolute vertical
speed below 0.5 m/s. Spectral amplitudes are Hann-corrected RMS values.
