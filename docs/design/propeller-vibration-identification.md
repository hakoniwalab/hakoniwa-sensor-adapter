# Rotor vibration: identifying the model parameters from real flight logs (design)

Status: reference implementation in [tools/vibration_fit](../../tools/vibration_fit/README.md). The X500 values of the
real-flight evaluation ([evaluation/imu-real-flight](../evaluation/imu-real-flight/README.md)) were identified with
an earlier method; section 7 lists the differences and their effect.

## 1. Purpose and scope

Hakoniwa Drone PRO adds rotor-synchronous vibration to the accelerometer and gyro truth before the IMU model
(`components.propellerVibration`). This document defines how to obtain every parameter of that model from a real PX4
flight log, so that anyone with a suitable log can build the vibration of their own vehicle.

In scope: the rotor-order harmonics (amplitude per axis, speed exponent, per-rotor phase, phase jitter).
Out of scope: vibration that does not follow the rotor speed (broadband, structural resonances excited by other
sources), and forward flight (the method uses hover).

## 2. The model being identified

Per physics step k, rotor i (direction s_i = ±1), order h, axis a (body FRD x, y, z):

```text
θ_i[k] = wrap(θ_i[k−1] + s_i Ω_i[k] Δt)                  rotor phase (initial value random)
q_i,h  = (Ω_i[k] / Ω_ref)^p_h       (0 when Ω_i = 0)      speed scale
w_i,h,a = sin(h θ_i[k] + φ_h + Δ_i,h,a + ψ_i,h,a[k])     wave

δa_a[k] = Σ_i Σ_h A_acc[h][a]  · q_i,h · w_i,h,a
δω_a[k] = Σ_i Σ_h A_gyro[h][a] · q_i,h · w_i,h,a
ψ ← ρψ + σ_h √(1−ρ²) n,   n ~ N(0,1),   ρ = exp(−2π B Δt)
```

δa and δω are added to the truth at the IMU; orientation, sensitivity, the IMU low-pass, delay, sampling and noise
follow. The generator has no attenuation of its own.

| Symbol | Configuration key | Identified from the log? |
|---|---|---|
| Ω_ref | `referenceRpm` | chosen (default: median hover speed, 3.3) |
| p_h | `exponent` | yes, orders 1–6 (4.2); fixed 2 above |
| A_acc[h][a], A_gyro[h][a] | `accelAmplitudeMps2`, `gyroAmplitudeRadps` | yes, orders 1–14 (4.3) |
| Δ_i,h,a | `rotorAxisPhaseDeg` | yes, orders 1–6, from the gyro (4.4) |
| σ_h | `phaseJitterDeg` | yes, orders 1–6 (4.4) |
| B | `phaseJitterBandwidthHz` | no: 2 Hz |
| φ_h | `phaseDeg` | no: 0 (absorbed in Δ and the random initial phase) |

The amplitude is common to all rotors; only the phase differs per rotor.

## 3. Inputs

### 3.1 What the log must contain

| Topic | Used for |
|---|---|
| `sensor_gyro_fifo`, `sensor_accel_fifo` | raw IMU at the FIFO rate (`SDLOG_PROFILE` "raw FIFO high-rate IMU"). Without the accel FIFO, accelerometer amplitudes are not identified |
| `esc_status` (`esc_rpm`) | rotor speeds (ESC telemetry or bidirectional DShot) |
| `vehicle_land_detected`, `vehicle_local_position` | hover selection |
| `vehicle_angular_velocity` | excludes yawing (optional) |

### 3.2 Hover

A sample is hover when it is at least 10 s after take-off and 5 s before landing, the horizontal speed is below
1 m/s, the vertical speed below 0.5 m/s and the yaw rate below 10 °/s (the same definition as the evaluation). With
`--half first|second` only that half of the fit log's hover (split at the median hover time) is used, so the other
half stays available for comparison.

### 3.3 Reference speed

Ω_ref only sets the point where the amplitudes are stated; any value gives the same vibration once the amplitudes are
rescaled. The default is the median mean rotor speed of the fit log's gyro hover windows, so that the amplitudes are
read where the vehicle actually flies.

## 4. Method

### 4.1 Order RMS per window

The FIFO is cut into continuous blocks (a timestamp gap ends a block) and then into 0.5 s windows inside hover with
every rotor above 500 rpm. Each axis has its window mean removed and gets a Hann-windowed FFT; RMS values are
corrected for the window power.

For order h the band is the union of k·f_r·(1 ± 3 %) over the rotors r (f_r = rotor speed / 60). Above about order 5
the bands of neighbouring orders overlap, because the four rotors of one vehicle run at different speeds in hover
(X500: about 5,300–6,000 rpm). **Each frequency bin is counted once**: it belongs to the order whose nearest rotor line
is closest. Orders whose band reaches the FIFO Nyquist frequency are skipped.

### 4.2 Speed exponent

For each order, p_h is the least-squares slope of log(three-axis order RMS) on log(mean rotor speed). Every (log,
sensor) pair has its own intercept, so only speed changes **within one flight** inform the slope; level differences
between flights or airframes do not. A pair enters only when its windows span at least 10 % in speed. Orders above
`--exponent-max-order` (default 6), and orders with no usable pair, use 2.

Hover alone covers a narrow speed range; more logs of the same airframe (`--exponent-log`) add pairs. The exponent
matters little for hover comparisons and more for climbs, descents and manoeuvres.

### 4.3 Amplitude

The generator produces, for rotor r, a sine of amplitude A·q_r with an independent phase; its RMS is A·q_r/√2. The sum
over rotors therefore has RMS A·√(Σ_r q_r² / 2). Per window and axis:

```text
A[h][a] = RMS_h,a / sqrt( Σ_r ((rpm_r / referenceRpm)^p_h)² / 2 )
```

with the exponent of 4.2. The model value is the median over the fit log's windows.

### 4.4 Per-rotor phase and jitter (gyro)

For rotor i and order h, the rotor phase θ_i is integrated from its ESC speed. Each gyro axis is multiplied by
exp(−j·h·θ_i) and low-passed at 1 Hz (zero-phase), giving the complex amplitudes z_x, z_y, z_z. Samples are used only
in hover and only while every other rotor's order-h line is at least 3 Hz away. Then:

```text
Δ_x = 0,   Δ_y = −s_i · arg Σ z_x conj(z_y),   Δ_z = −s_i · arg Σ z_x conj(z_z)
R   = |Σ z_x conj(z_y)| / Σ |z_x conj(z_y)|        (phase coherence of the rotor)
σ_h = sqrt(−ln median_i R)                          (radians; reported in degrees)
```

The sign s_i appears because the demodulation uses the speed magnitude. A rotor whose order-h line never separates
from another rotor's by 3 Hz has no phase; `--unresolved-phase zero` writes 0, 0, 0 for it (and the report names it),
`omit-order` drops `rotorAxisPhaseDeg` for that order. The accelerometer uses the gyro's phases.

The same demodulation also gives each rotor's own gyro amplitude per axis, 2·sqrt(mean |z|²). It goes to the report
only (4.5).

### 4.5 Why amplitudes come from band power and phases from demodulation

Demodulation separates the rotors, so it could in principle give every parameter, amplitude included. It was tried
on the X500 logs and kept for phases only:

| | Band power (4.1, 4.3) | Demodulation (4.4) |
|---|---|---|
| Data it needs | 0.5 s of continuous FIFO | several seconds of continuous FIFO (1 Hz low-pass, 1 s dropped at each end) |
| X500 fit log | gyro and accel usable | gyro usable (continuous runs of about 6 s); **accel unusable**: the logged accel FIFO is broken into runs mostly shorter than 1 s |
| What it measures | all vibration energy in the order band, including energy spread around the rotor lines | only the part that follows the rotor angle within ±1 Hz |
| Orders 2–6 gyro, demodulated ÷ band power | | 0.6–0.96 |
| Orders 7–14 gyro, demodulated ÷ band power | | 0.3–0.7, falling with the order |
| Rotors | sum of all rotors (the model amplitude is common to them anyway) | per rotor; rotors that do not separate (X500 order 1, rotors 3 and 4) are missing |

The real higher orders do not follow the rotor angle tightly: their lines are spread in frequency (the Campbell
diagram of the public X500 flights shows orders 3 and above smeared). Demodulation drops that spread energy, while the
PX4 vibration metrics see all of it. Band power is therefore the right measure for the model amplitude, and
demodulation for what only it can give: the per-rotor phase.

The per-rotor amplitudes in the report show how unequal the rotors are. On the X500 fit log (first half), order 5,
gyro x: 7.5 °/s for rotor 1, 2.2 °/s for rotor 3, 0.5 °/s for rotors 2 and 4. The model cannot represent this yet
(section 8).

## 5. Output

- the `propellerVibration` object, ready to place under `components` of a Drone PRO drone config
  (`axisPhase: "rotating"` applies to orders without `rotorAxisPhaseDeg`);
- a report: inputs, window counts per log and sensor, how each exponent was obtained, phase coherence, per-rotor gyro
  amplitudes by demodulation (orders with a phase), unresolved rotors, and the values that were chosen rather than
  identified.

## 6. Check on the X500

Inputs: fit log `75ac4a47` (first half of the hover; gyro 134, accel 51 windows), exponents from it and the seven
other PX4_FMU_V6X X500 flights of [imu-x500](../evaluation/imu-x500/README.md); `--reference-rpm 5540` to compare
with the evaluation's values. Six (log, sensor) pairs had a speed span of 10 % or more.

| Order | Exponent | Phase coherence R | Jitter σ [°] |
|---|---|---|---|
| 1 | 1.24 | 0.99 | 5.3 |
| 2 | 1.46 | 0.78 | 28.5 |
| 3 | 1.79 | 0.86 | 21.9 |
| 4 | 1.73 | 0.80 | 27.2 |
| 5 | 1.59 | 0.73 | 31.9 |
| 6 | 1.53 | 0.91 | 17.6 |

Order 1 of rotors 3 and 4 (index 2, 3) stayed unresolved and was set to 0.

## 7. Relation to the evaluation's values (earlier method)

The X500 vibration used in [evaluation/imu-real-flight](../evaluation/imu-real-flight/README.md) was identified with an
earlier method: exponents with [fit_v6x.py](../../tools/compare/imu_x500/fit_v6x.py), amplitudes and phases with
unpublished scripts. Those scripts, re-run, give the evaluation's values exactly. The reference implementation
differs in two ways.

| | Earlier method | Reference implementation | Effect on the X500 values |
|---|---|---|---|
| Overlapping order bands (orders ≳ 5) | each order sums its whole band, so a bin shared by two orders is counted in both | each bin counted once (4.1) | orders 7–14: 0.47–0.90 × the earlier amplitudes |
| Exponent | one slope over all flights' windows with one intercept per sensor, so flight-to-flight level differences enter the slope | one intercept per (log, sensor): within-flight slope only (4.2) | orders 1–6: 1.2–1.8 instead of 1.5–9.0 |

The earlier amplitude normalization also used exponent 2 for every order. Together with the band change, the
orders 1–6 amplitudes differ by 2 % or less in hover. Phases differ by at most 2.3° and the jitter values are
identical.

Flown with each set on the emulated Pixhawk 6X (PILS, three flights each), against the second half of the real
hover (`vehicle_imu_status` medians, simulation ÷ real):

| Chip | Metric | Earlier values (evaluation) | Reference implementation |
|---|---|---|---|
| ADIS16470 | gyro vibration | 0.99–1.01 | 0.97–0.98 |
| | accel vibration | 0.97–0.98 | 0.94–0.96 |
| | coning | 0.92–0.97 | 0.82–0.84 |
| IIM-42652 | gyro vibration | 0.97–0.99 | 0.96–0.97 |
| | accel vibration | 0.95–0.96 | 0.93–0.95 |
| | coning | 0.91–0.94 | 0.83–0.85 |

With bins counted once, the simulation comes out lower, most clearly in coning. The double counting of the earlier
method was most likely filling part of a gap rather than describing rotor vibration: the model has no vibration
outside the rotor orders, while the real logs have broadband vibration there (section 8). The evaluation's numbers
stand as measured with its values; they are not a property of the reference implementation.

## 8. Limitations

- Vibration outside the rotor orders is not modelled. In the 20 public X500 flights the non-order gyro RMS
  (40–1000 Hz) has a median of 6.2 °/s against 25.9 °/s in total. How much of the coning shortfall it explains is not
  measured yet.
- Amplitudes are per axis but common to all rotors, although the real rotors differ by up to about 16 times in one
  order (4.5). A per-rotor amplitude needs a model change in Drone PRO; the report already holds the per-rotor values.
- Phases come from the gyro only, and only for orders 1–6; rotors whose lines stay within 3 Hz of each other have none.
- The exponent rests on the speed changes inside hover; it is weakly determined, and thrust and speed cannot be
  separated with these logs.
- Hover only. Forward flight, payload changes and airframe changes need their own logs.
