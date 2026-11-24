# Force-torque vibration: what data identifies which parameter (design)

Status: step 1 of 3. This document defines the data that the identification needs and which parameter each part of
it identifies. The method per parameter (step 2) and a reference tool with a check on public data (step 3) follow, as
for the rotor vibration of the IMU ([propeller-vibration-identification.md](propeller-vibration-identification.md)).
The model is [force-torque-vibration.md](force-torque-vibration.md); its requirements are
[spec/force-torque-vibration.md](../spec/force-torque-vibration.md) (`FTV-*`).

## 1. Purpose

The parameters of the arm's vibration model were first fitted by hand to one public recording (Zenodo 11096791,
design section 7.1). That reproduces the recording but does not tell anyone else how to get the parameters of their
own arm, nor which values were measured and which were assumed. This document fixes the first half of that:

- what a recording must contain, and how to record it (section 2, 3);
- for every parameter, which part of the recording identifies it, and what happens without that part (section 4);
- for the public recording used so far, which parameters it identifies and which it does not (section 5).

## 2. Signals

| Signal | Requirement | Used for |
|---|---|---|
| Wrist force-torque, 6 axes | raw sensor output (not a controller estimate), at f_s ≥ 500 Hz and f_s / 2 above the highest order line (below); the sensor's own filter setting known | everything |
| Joint positions or velocities | every joint, at 100 Hz or more, on the same clock as the force-torque (or with a sync event in both) | motor speeds f_m,j = N_j · \|q̇_j\| / 2π, activity, order tracking |
| Gear ratios N_j | from the arm's data sheet; otherwise inferred from the order lines (section 4, A) | order frequencies |
| Joint torques (or motor currents) | the joints that turn, same clock | load direction g± (R4) |
| Sensor orientation | from the joint positions and the arm's kinematics | world-fixed modes (R6) |
| Tool mass and centre of mass | from a calibration (several static poses) | removing gravity from the force-torque, so that only the vibration is left |
| Contact | none during the segments below, or a flag that marks contact | contact is out of the model (VIB-A9) |

The highest order line is k_max · N_j · q̇_max / 2π. With N = 160, k up to 4 and 45 °/s it is 80 Hz; at 90 °/s,
160 Hz. The force-torque rate must stay above twice that, with margin for the sensor's own filter.

## 3. Recording protocol

Six segments. Each can be a separate recording; all use the same tool and payload.

| Segment | What to record | Duration or count |
|---|---|---|
| R1 rest | arm still, servos on, in the pose of R2 | 30 s or more |
| R2 speeds | the same single-joint motion at two or more constant speeds, each with a cruise, and the speed-up and slow-down | cruise 10 s or more per speed; speeds at least 2× apart |
| R3 start and stop | the motion of R2 started and stopped | 5 starts and 5 stops or more, with 2 s of rest between |
| R4 lift and lower | a joint moved against gravity and with gravity (the gravity torque changes sign during the motion, or two motions) | both directions at one speed of R2 |
| R5 repeats | R2 at one speed, repeated | 3 times or more, the same day and settings |
| R6 orientation | a motion that turns the sensor by 90° or more about a horizontal axis; R1 in a second pose | one of each |

R1 and R2 are the minimum. Without R3 to R6 the model still runs, with the parameters of those segments left at their
defaults or assumed (section 4).

## 4. Which data identifies which parameter

Status words: **identified** (computed from the data by a fixed rule), **inferred** (a reading of the data, to be
checked), **default** (not identifiable from the data; the default of the specification is used), **not observable**
(no recording shows it; it does not change the spectra).

### A, gear orders

| Parameter | Key | Identified from | Without it |
|---|---|---|---|
| order k | `orders[].order` | R2: a line whose frequency moves with motor speed; k = line frequency / f_m | lines are taken as modes (B) |
| gear ratio N_j | `gear_ratio` | data sheet; or R2: the slope of line frequency against joint speed = k · N_j / 2π, needs k assumed | k and N cannot be told apart: only k · N_j is known |
| amplitude a_{j,k,i} | `orders[].amplitude` | R2: band power of the line per axis, at the reference speed | — (required) |
| speed exponent p | `orders[].exponent` | R2: the line amplitude at two or more speeds (or along a speed-up) | default 0 (constant), literature |
| dead band f_dead | `deadband_hz` | R3: the speed below which the line disappears | default 1 Hz |
| phase θ | — | not observable | random |
| resonance gain | `orders[].resonance` | R2: the line grows where k · f_m crosses a mode | off |
| load gain g+, g−, lag T | `load_gain` | R4: line amplitude while lifting vs lowering; the lag from the transition | 1 (no load effect) |
| run variation σ_r | `joints[].run_log_sigma` | R5: spread of ln(amplitude) over the repeats | 0 |

### B, arm modes

| Parameter | Key | Identified from | Without it |
|---|---|---|---|
| frequency f_n, damping ζ | `frequency_hz`, `damping` | R1: a peak in the spectrum at rest (half-power width); R2: a peak that does not move with speed | — (required for each mode) |
| level at rest s0 | `at_rest` | R1: band level around f_n per axis | — (required) |
| level while moving s2 | `per_motion` | R2 at two speeds: the part of the band level that is the same at both; R3: the jump at a start | cannot be told from s1 (one speed: s1 · activity and s2 are the same number) |
| growth with activity s1 | `per_activity` | R2 at two speeds: the part that grows with speed | as above |
| motion ramp a_r | `motion_ramp` | R3: the activity at which the level reaches s2 after a start | default 0.02 |
| share kept while moving | `at_rest_while_moving` | R1 vs R2: a peak present at rest and absent at cruise | default 1 (kept) |
| frame sensor or world | `frame` | R6: the level on a sensor axis follows the sensor orientation (for example \|cos θ\|) | sensor (VIB-A7) |
| run variation σ_r | `modes[].run_log_sigma` | R5 | 0 |

### C, broadband

| Parameter | Key | Identified from | Without it |
|---|---|---|---|
| density c_i | `broadband.density_per_activity` | R2: the floor between the lines per axis, minus the sensor noise of R1, at the reference activity | — (required) |
| exponent p_b | `broadband.exponent` | R2 at two speeds: how the floor grows with activity | default 1 |
| run variation σ_r | `broadband.run_log_sigma` | R5 | 0 |

### Shared

| Parameter | Key | Identified from | Without it |
|---|---|---|---|
| reference frequency f_ref | `f_ref_hz` | chosen: the motor frequency at the main speed of R2, so that amplitudes are stated where the arm works | default 50 Hz |
| run groups | `run_group` | R5: components whose multipliers move together | none |

## 5. What the public recording identifies (Zenodo 11096791)

KUKA LBR Med 14 with an ATI Gamma at about 700 Hz, CC BY 4.0: a rest recording and two rotations of one joint at
one cruise speed (45 °/s), each with a speed-up and a slow-down, the tool going over the top; the second rotation has
taps on the tool, so only the windows away from them are used. Joint orientation
at about 100 Hz; no joint torques; the turning joint and its gear ratio are not documented.

| Segment | In the recording | |
|---|---|---|
| R1 rest | yes | |
| R2 speeds | one cruise speed; the speed-up and slow-down sweep 5 to 45 °/s | partial |
| R3 start and stop | one start and one stop per rotation | partial |
| R4 lift and lower | the gravity torque changes sign half-way (from a model, not measured torque) | partial |
| R5 repeats | two rotations | partial |
| R6 orientation | the sensor turns 180°; one pose at rest | partial |

| Parameter | Status on this recording |
|---|---|
| k (order 2) | identified (the line follows the speed during speed-up and slow-down) |
| N | inferred (160, from k = 2 assumed) |
| a | identified at 45 °/s |
| p | inferred (0.8 along the speed-up; against the literature's 0) |
| f_dead | default |
| g+, g−, T | inferred (two recordings; load from a model; the lag fitted) |
| σ_r | inferred (two recordings only) |
| f_n, ζ, s0 | identified (R1) |
| s1 vs s2 | inferred (s2 from the start, stop and the level being similar over the speed-up; s1 not separable at one cruise speed) |
| a_r | inferred (one start) |
| share kept while moving | inferred (one mode, removed while moving) |
| frame world | inferred (one rotation; the direction read from \|cos θ\|) |
| c | identified at 45 °/s |
| p_b | default (one speed) |

The parameters marked inferred were set by hand in the first fit (the private tuning log T0 to T14). To identify them
by a fixed rule, a recording with the missing segments is needed: from a public data set if one exists, otherwise our
own recording following section 3.

## 6. Next steps

1. Step 2: the method for each parameter (windows, bands, estimators, uncertainty), written so that a tool can follow
   it.
2. Step 3: a reference tool in `tools/`, checked against the values of the first fit on Zenodo 11096791 where they are
   identified (section 5), and a held-out part of the recording.
