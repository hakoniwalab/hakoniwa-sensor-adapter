# Axis 1: accuracy (2026-11-01)

We added error settings one at a time and compared the statistics of (output − truth) with expected values computed from the settings.
To give the settings the same meaning, we wrote them as drone_config items for Hakoniwa, SDFormat items for Gazebo, and core_sim JSON items for Project AirSim (unit conversions in the notes below). Project AirSim specifies IMU white noise as a density, so we converted it as σ × √(sample interval). The truth is a stationary state (IMU level and at rest, GPS at the origin).

Method: as in "Method" in [Drone sensor results](README.md).

## Results

Project AirSim is the core_sim sensors compiled as is. "—" means that the setting cannot be expressed in Project AirSim (explained below).

| Sensor | Setting | Statistic | Expected | Hakoniwa | Gazebo | Project AirSim |
|---|---|---|---|---|---|---|
| Gyro | Noise 0.01 rad/s | Standard deviation (N = 20,000) | 0.01 | 0.01001 | 0.01004 | 0.00997 |
| Accelerometer | Noise 0.05 m/s² | Standard deviation | 0.05 | 0.0507 | 0.0499 | 0.0501 |
| Gyro | Bias 0.5 °/s | Mean | 0.008727 rad/s | 0.008727 | **0.000087** (mean of 200 runs) | 0.008727 |
| Gyro | Bias 0.5 °/s | Fraction of runs with a positive value | 1 | 1 | **0.505** | 1 |
| Gyro | Turn-on bias drawn at start-up 1 °/s | Standard deviation over 400 runs | 0.01745 rad/s | 0.01823 | 0.01623 | — |
| Gyro | Slowly varying bias σ 0.05 °/s, τ 1 s | Standard deviation (1000 τ) | 0.000873 rad/s | 0.000867 | 0.000850 | — |
| Gyro | Same | Autocorrelation at τ | 0.368 | 0.369 | **0.302** | — |
| Gyro | Bias random walk 0.001, τ 500 s | Standard deviation over 200 runs after 100 s | 0.000447 rad/s | — | — | **0 (all 200 runs identical)** |
| Gyro | Same | Standard deviation of the change per 100 s within one long run (200 intervals) | 0.000447 rad/s | — | — | 0.000436 |
| Gyro | Resolution 0.001 rad/s (with noise) | Fraction of values on the step grid | 1 | 1 | 1 | — |
| Magnetometer | Noise 100 nT | Standard deviation | 100 | 100.2 | 101.2 | 99.9 |
| Magnetometer | Bias 3000 nT | Mean | 3000 | 3000 | **−3000** | — |
| Magnetometer | Turn-on bias b = 3000 nT (`noise-bias`) | Mean and standard deviation over 400 runs | Mean −3000, standard deviation 3000 | — | — | **−7209 in all 400 runs (standard deviation 0)** |
| Magnetometer | Slowly varying bias σ 100 nT, τ 1 s | Standard deviation | 100 | 99.6 | **0 (does not move)** | — |
| Magnetometer | Resolution 15 nT | Fraction of values on the step grid | 1 | 1 | **0 (no effect)** | — |
| Barometer | Noise 0.5 m (5.94 Pa) | Standard deviation | 5.94 Pa | 5.95 | 5.86 | 5.97 |
| Barometer | Bias +2 m | Mean − true pressure | −23.7 Pa | −23.7 | −23.7 | −23.8 (via QNH) |
| Barometer | Slowly varying bias σ 0.3 m, τ 1 s | Standard deviation | 3.56 Pa | 3.47 | **0 (does not move)** | **0.35 (1/10)** |
| Barometer | Resolution 1 Pa | Fraction of values on the step grid | 1 | 1 | 1 | — |
| GPS | Horizontal noise 0.5 m | North standard deviation | 0.5 | 0.501 | 0.500 | — |
| GPS | Same | East standard deviation | 0.5 | 0.497 | **0.337** | — |
| GPS | Vertical noise 1.0 m | Standard deviation | 1.0 | 1.001 | 1.001 | — |
| GPS | Velocity noise 0.1 m/s | North standard deviation | 0.1 | 0.100 | 0.101 | — |
| GPS | Slowly varying bias σ 0.5 m, τ 1 s | North standard deviation | 0.5 | 0.515 | **0 (does not move)** | — |

Bold values do not match the expected value.

## Explanation of the differences

### Gazebo

- **Sign of the bias**: When Gazebo loads `bias_mean`, it flips the sign with 50 % probability (gz-sensors `GaussianNoiseModel`). A fixed bias cannot be expressed. The single magnetometer run (−3000 nT) also hit this.
- **Slowly varying bias (dynamic bias) of the magnetometer, barometer and GPS**: Gazebo advances the bias using the time step, but the magnetometer, barometer and GPS add noise with a time step of 0. So the bias stays at its initial value (0) and does not move. Only the IMU passes the time step, so only it moves.
- **Magnetometer resolution**: Gazebo treats a step smaller than 1e-6 as 0 and ignores it. The magnetometer is specified in tesla, so 15 nT = 1.5e-8 T is ignored.
- **GPS east noise**: Gazebo adds horizontal noise in degrees, with the same size to latitude and longitude. One degree of longitude is cos(latitude) times shorter than one degree of latitude, so in the east direction it shrinks to 0.5 × cos(47.6°) ≈ 0.34 m.
- **Autocorrelation of the IMU bias**: In Gazebo the standard deviation matches, but the autocorrelation at τ came out at 0.30, smaller than the expected 0.37 (the same when measured over 1000 τ). The reason is not known yet.

### Project AirSim (core_sim)

- **Fixed seed**: core_sim starts the random number generator of every instance with a fixed seed (1, 2, 3 for the IMU and magnetometer; 42 for the barometer). It does not use the host seed. So every instance gives the same value sequence, and the spread across runs is 0. The random walk matched the formula when measured within one long run.

- **Noise**: White noise is set by "random walk density ÷ √(sample interval)". When the density was converted to give the same σ, it matched the setting.
- **IMU bias**: The turn-on bias (`turn-on-bias`) is a fixed vector; there is no form that draws it from random numbers. The slowly varying bias is a random walk (`bias-stability`), not Gauss-Markov. The spread of the random walk (proportional to √time) matched the formula.
- **Magnetometer bias**: `noise-bias` draws once at start-up from a normal distribution with mean −b and standard deviation b. Because the seed is fixed, the drawn value is −7209 nT (x axis) every time. A fixed bias of an arbitrary value (hard iron) cannot be expressed.
- **Barometer slowly varying bias**: The Gauss-Markov process of the pressure factor uses a formula that does not scale the per-step noise by √(1 − α²). So the steady-state spread is σ·√((1 − α)/(1 + α)), which at 50 Hz and τ 1 s is about 1/10 of σ (0.36 Pa). Changing the sample period also changes the spread.
- **Barometer bias**: Can be expressed with the QNH setting (as configured).
- **GPS**: Position and velocity have no noise; the truth is output as is.
- **Resolution**: None of the sensors has it.

### Hakoniwa

- All 21 items agreed with the expected values within the statistical spread.
- **Turn-on bias drawn at start-up**: The spread of the standard deviation over 400 runs is about ±0.6e-3. Both Hakoniwa and Gazebo fall within this range.

## Unit conversions (to give the same meaning)

| Item | Hakoniwa | Gazebo | Project AirSim |
|---|---|---|---|
| Slowly varying bias | Steady-state standard deviation σ | `dynamic_bias_stddev` = σ·√(2/τ) | (IMU uses a random walk.) Barometer: `pressure_factor_sigma` = σ ÷ pressure |
| Barometer noise and bias | Height (m) | Pressure (Pa). Multiply by the local pressure difference per metre (11.87 Pa/m) | Pressure (Pa). Bias via QNH |
| GPS horizontal position | m | Degrees (divide by the length of one degree of latitude) | (no noise) |
| Magnetometer | nT | T | Gauss (1 G = 100,000 nT) |
