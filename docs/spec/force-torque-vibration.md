# Force-torque vibration model specification

This is the specification of the vibration model that the host adds to the true wrench in front of a force-torque
sensor adapter. The design, the evidence and the assumptions (`VIB-A*`) are in
[force-torque-vibration.md](../design/force-torque-vibration.md). Requirements are numbered `FTV-<nnn>`.

The model is a host-side model: it does not know the sensor adapter, and the adapter does not know it. The host
calls the model every physics step with the joint velocities and adds its output to the wrench it passes to the
adapter (`ForceTorqueTruth`). The interface below is public; implementations (for example Sensor PRO) are reached by
name.

Notation: axes i = Fx, Fy, Fz, Tx, Ty, Tz in sensor axes (FLU), joints j in the order of the configuration, h the
physics step [s], N(0, σ) the normal distribution.

## 1. Interface

| ID | Requirement |
|---|---|
| FTV-010 | A model is created from a configuration file (section 5), the physics step, the vehicle name and index, the sensor name, and an optional seed, the fields of `SensorAdapterSetup`. Creation fails with a message on any configuration error (FTV-510) |
| FTV-020 | `step(sim_time_ns, joint_velocity, out, error)` takes the joint velocities [rad/s, joint side] of this physics step and returns the wrench to add [N, N·m, sensor axes]. The number of velocities must equal the number of configured joints (`joint_names()` gives them in order), and each must be finite; otherwise it returns false with a message and changes no state |
| FTV-030 | Step times are 0, h, 2h, …; any other time is an error without side effects, as for the force-torque adapter (FTS-ADP-020) |
| FTV-040 | `reset` returns the motor phases, the filters and the random streams to their state at creation; the same inputs then give the same outputs as right after creation |
| FTV-050 | An implementation provides the model through an entry point of its own, `HAKO_SENSOR_HOST_MODEL_ENTRY(<name>)`, returning a table with `create_force_torque_vibration`. `SensorAdapterImplementation` is not changed; an implementation without the model has no such entry |
| FTV-060 | The output is the sum of the order components (section 2), the structural modes (section 3) and the broadband component (section 4). With no component configured, the output is zero |

## 2. Order components (follow motor speed)

| ID | Requirement |
|---|---|
| FTV-110 | Motor phase per joint: φ_j ← φ_j + N_j · q̇_j · h every step, from φ_j = 0 at creation and reset. N_j is the joint's gear ratio |
| FTV-120 | Motor frequency: f_m,j = N_j · \|q̇_j\| / 2π [Hz] |
| FTV-130 | Each configured order k of joint j adds on each axis i: A_{j,k,i} · sin(k · φ_j + θ_{j,k,i}). k is a positive number (usually an integer) |
| FTV-140 | Amplitude: A = a_{j,k,i} · (f_m,j / f_ref)^p · min(1, f_m,j / f_dead,j). p defaults to 0 (amplitude constant with speed, design references [6][9]); f_dead,j defaults to 1 Hz. The factor makes the component vanish smoothly as the joint stops |
| FTV-150 | θ_{j,k,i} is drawn once at creation from U(−π, π) in the stream `vibration_phase`, in the order of joints, then orders, then axes |
| FTV-160 | Optional resonance gain per order (`"resonance": true`): A is multiplied by max over the configured modes of 1 / √((1 − r²)² + (2ζ_n r)²), r = k·f_m,j / f_n. Without modes the gain is 1 |

## 3. Structural modes (fixed frequency)

| ID | Requirement |
|---|---|
| FTV-210 | Each mode n has a frequency f_n and a damping ζ_n (0 < ζ_n < 1, f_n < 1 / 2h). Its signal is white noise N(0, 1), one draw per step and axis from the stream `vibration_mode`, through a second-order band-pass at f_n with Q = 1 / (2ζ_n), bilinear transform with pre-warping at the physics step, zero initial state |
| FTV-220 | The band-pass output is scaled so that its steady-state standard deviation is 1: by 1 / √(Σ h_k²), with h_k its impulse response, computed at creation |
| FTV-230 | The mode adds σ_{n,i} times that signal on axis i, with σ_{n,i} = s0_{n,i} + s1_{n,i} · activity. s0 is the level at rest (`at_rest`), s1 the growth with motion (`per_activity`) |
| FTV-240 | activity = Σ_j f_m,j / f_ref, with f_ref from the configuration (default 50 Hz) |

## 4. Broadband component

| ID | Requirement |
|---|---|
| FTV-310 | On each axis i, adds N(0, σ_b,i) per step from the stream `vibration_broadband`, with σ_b,i = √(D_i · 1 / 2h) and the one-sided density D_i = c_i · activity^p_b [unit²/Hz]. p_b defaults to 1 |
| FTV-320 | With activity 0 the broadband component is 0 |

## 5. Configuration

| ID | Requirement |
|---|---|
| FTV-410 | The configuration is JSON with `"type": "force_torque_vibration"` and the optional keys `f_ref_hz`, `joints`, `modes`, `broadband`, as in the design (section 6) |
| FTV-420 | `joints` is a list of `{name, gear_ratio, deadband_hz, orders}`; `orders` a list of `{order, exponent, resonance, amplitude}`. `name` is for logs and must be unique |
| FTV-430 | `modes` is a list of `{frequency_hz, damping, at_rest, per_activity}` |
| FTV-440 | `broadband` is `{exponent, density_per_activity}` |
| FTV-450 | Per-axis values (`amplitude`, `at_rest`, `per_activity`, `density_per_activity`) are objects of axis groups and axes as for the force-torque sensor (FTS-ADP-520), in the axis units; an axis not given is 0. Percentages are not allowed |
| FTV-510 | Creation fails on: an unknown key, a wrong type, a missing `type`, `name`, `gear_ratio`, `order`, `frequency_hz` or `damping`, a gear ratio, order or `f_ref_hz` of 0 or less, a negative amplitude, level, density, exponent or dead band, a damping outside (0, 1), a mode frequency at or above 1 / 2h, a non-finite value, or duplicate joint names |

## 6. Random numbers and reproducibility

| ID | Requirement |
|---|---|
| FTV-610 | The streams `vibration_phase`, `vibration_mode` and `vibration_broadband` are separate and derived from the seed, the vehicle name and index, the sensor name and the characteristic (the common derivation of the sensor adapters). Turning one component off does not change the others' sequences |
| FTV-620 | With a seed, the same seed, configuration, inputs and call order give the same outputs; the model's streams are independent of the force-torque adapter's streams |

## 7. Assumptions

Each requirement rests on the assumptions of the design (section 8 there): FTV-110 to 140 on VIB-A2 and A3, FTV-150 on
A4, FTV-210 to 240 on A5 and A6, FTV-310 on A6, FTV-060 on A8, and the whole model on A7 (fixed direction in sensor
axes) and A9 (no load, temperature, wear or contact effects).
