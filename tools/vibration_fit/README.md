# Rotor vibration identification (reference implementation)

Identifies the Hakoniwa Drone PRO rotor-order IMU vibration (`components.propellerVibration`) from real PX4 ULogs.
The method, its choices and its limits are in
[docs/design/propeller-vibration-identification.md](../../docs/design/propeller-vibration-identification.md).

Requires Python 3 with `numpy`, `scipy` and `pyulog`.

```sh
python tools/vibration_fit/identify.py \
  --fit-log <fit flight>.ulg --half first \
  --exponent-log <another flight of the same airframe>.ulg \
  --out vibration.json
```

- `--fit-log`: amplitudes and phases come from this log. `--half first` keeps the second half of its hover for
  comparison.
- `--exponent-log` (repeatable): more flights of the same airframe for the speed exponents.
- `--rotation-signs`: rotor directions per `esc_status` index (default `1,1,-1,-1`, as for the X500; use the same signs as the drone config).
- `--reference-rpm`: default is the median hover speed of the fit log.

Outputs `vibration.json` (paste it as `components.propellerVibration` of the drone config) and
`vibration.report.json` (window counts, how each exponent was obtained, phase coherence, unresolved rotors, chosen
values). The run on the X500 logs of the evaluation takes about 15 s.
