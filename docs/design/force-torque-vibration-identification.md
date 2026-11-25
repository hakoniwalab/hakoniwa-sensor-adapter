# Force-torque vibration: what data identifies which parameter (design)

Status: steps 1 and 2 of 3; the rules in use are version 4 (section 8), found by exploration and still to be confirmed on unseen data. This document defines the data that the identification needs and which parameter each part of
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
| R7 tap test (optional) | light hammer taps on the tool or the last link, the arm still in the pose of R1 | 5 taps or more, 2 s or more apart |

R1 and R2 are the minimum. Without R3 to R6 the model still runs, with the parameters of those segments left at their
defaults or assumed (section 4).

R7 has one purpose: to check the arm modes (frequency, damping, axes) identified from the motion (section 8.5). Taps excite them much
above the background vibration. Taps 2 s apart let each ringing decay; taps 1 s apart, as in the public recording,
cut it. Tap with the arm still, so that the modes do not move with the pose. R7 is not used to model the noise of a
motion with taps (section 8.6). An impact hammer that records its force is not needed.

## 4. Which data identifies which parameter

Status words: **identified** (computed from the data by a fixed rule), **inferred** (a reading of the data, to be
checked), **default** (not identifiable from the data; the default of the specification is used), **not observable**
(no recording shows it; it does not change the spectra).

### A, gear orders

| Parameter | Key | Identified from | Without it |
|---|---|---|---|
| order k | `orders[].order` | R2: a line whose frequency moves with motor speed; k = line frequency / f_m | lines are taken as modes (B) |
| gear ratio N_j | `gear_ratio` | data sheet; or R2: the slope of line frequency against joint speed = k · N_j / 2π, needs k assumed | k and N cannot be told apart: only k · N_j is known |
| amplitude a_{j,k,i} | `orders[].amplitude` | R2: band power of the line per axis, at the reference speed. The band power gives the RMS; a is the peak of the sine: a = √2 · RMS | — (required) |
| speed exponent p | `orders[].exponent` | R2: the line amplitude at two or more speeds (or along a speed-up) | default 0 (constant), literature |
| dead band f_dead | `deadband_hz` | R3: the speed below which the line disappears | default 1 Hz |
| phase θ | — | not observable | random |
| resonance gain | `orders[].resonance` | R2: the line grows where k · f_m crosses a mode | off |
| load gain g+, g−, lag T | `load_gain` | R4: line amplitude while lifting vs lowering; the lag from the transition. Only the ratio g− / g+ is identified: a · g is what the data shows, so a and g trade against each other. Convention: `positive` = 1 and a is the amplitude while lifting | 1 (no load effect) |
| run variation σ_r | `joints[].run_log_sigma` | R5: spread of ln(amplitude) over the repeats | 0 |

### B, arm modes

| Parameter | Key | Identified from | Without it |
|---|---|---|---|
| frequency f_n, damping ζ | `frequency_hz`, `damping` | R1: a peak in the spectrum at rest (half-power width); R2: a peak that does not move with speed | — (required for each mode) |
| level at rest s0 | `at_rest` | R1: band level around f_n per axis, as a standard deviation (RMS; no √2, unlike a) | — (required) |
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
| reference frequency f_ref | `f_ref_hz` | chosen, not identified (below) | default 50 Hz |
| run groups | `run_group` | R5: components whose multipliers move together | none |

How to choose f_ref. It is a unit, not a property of the arm: any value gives the same vibration once the other
parameters are rescaled with it. Changing f_ref to f_ref' changes

- the amplitudes: a' = a · (f_ref' / f_ref)^p (no change when p = 0);
- the activity: activity' = activity · f_ref / f_ref', so s1, a_r and c change with it
  (s1' = s1 · f_ref' / f_ref, a_r' = a_r · f_ref / f_ref', c' = c · (f_ref' / f_ref)^p_b).

Choose it so that the numbers are read where they were measured: the motor frequency of the main joint at the main
cruise speed of R2 (for example N = 160 at 45 °/s gives 20 Hz). The amplitudes a are then the line amplitudes measured
at that speed, and activity 1 means that joint at that speed. With several joints, use the joint and speed that the
application moves most. Fix f_ref before fitting the other parameters and keep it with them.

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

## 6. Method (step 2), use case 1 first

Rules that a tool follows without hand tuning, for the parameters that matter to use case 1 (force control with the arm
nearly still or slow; spec section 0). The other parameters keep their defaults or are reported as not identified.
Fit data and validation data are kept apart: the tool reads only the fit data.

### 6.1 Preparation

1. Gravity and the tool: subtract the mean of each window (the vibration is what varies within a window). Over a long
   rotation, subtract the gravity of the tool model first when it is known; otherwise use windows of 0.3 s or less.
2. Joint speed |ω_j| from the joint positions (or, as for Zenodo 11096791, from the sensor orientation of a
   single-joint motion), resampled to the force-torque samples.
3. States: rest (the rest recording, or |ω| < 0.5 °/s for 1 s or more), moving (|ω| ≥ 5 °/s), slow (0.5 °/s < |ω| <
   20 °/s). Windows that contain contact or impacts are dropped.
4. Controller view: high-pass 3 Hz (4th order) and low-pass 20 Hz (2nd order), zero-phase.

### 6.2 Parameters

| Parameter | Rule |
|---|---|
| sensor white noise (adapter) | at rest, the PSD floor above the highest mode band (or 40–300 Hz), per axis, as density |
| modes f_n, ζ | **(v2)** candidates from two places: (1) peaks of the rest PSD between 3 and 60 Hz, 3 dB or more above the rest floor; (2) peaks of the cruise PSD between 3 and 300 Hz, 3 dB or more above the floor, that are not order lines (above) and appear within ±1 Hz in the pooled speed-up and slow-down PSD. Candidates within 1 Hz of each other are merged. f_n at the peak; ζ from the half-power width, ζ = Δf / (2 f_n), Δf at least two bins of the spectrum it was found in (the rest spectrum is coarse: its ζ is an upper bound) |
| level at rest s0 | at rest, σ in the band f_n ± max(3 Δf, 1 Hz) per axis, minus the sensor noise in that band (in power); 0 when not above the noise |
| level while moving s2 | moving, σ in the same band minus s0 and the noise (in power). With one cruise speed s1 = 0 and the whole moving level goes to s2 (use case 1: the level is about the same at slow speeds) |
| share kept while moving | **(v2)** 0 for a candidate found only at rest (source 1 and not source 2), or when its rest peak is more than 6 dB below the moving floor at f_n while moving; otherwise 1 |
| motion ramp a_r | default 0.02; replaced when R3 has five or more starts: the activity at which the band σ reaches 90 % of its moving level |
| order k, gear ratio N | **(v2)** order lines are the lines whose frequency follows the motor speed: in the speed-up and the slow-down (0.3 s windows, step 0.1 s), a peak between 20 and 300 Hz whose frequency, regressed on \|ω\| through the origin, stays within ±1.5 Hz of the fit in at least 80 % of the windows. Among them, the one with the most cruise power is the main order. With N from the data sheet, k = f_line / f_m. Without it, k = 2 (strain-wave gear) and N from the slope, reported as inferred. Lines that do not follow the speed are fixed lines and go to the modes |
| amplitude a (per axis) | cruise PSD band power of the line (± 3 Hz, minus the floor), a = √2 · RMS, at f_ref = the cruise motor frequency |
| speed exponent p | in the speed-up and the slow-down, the line's RMS in 0.3 s windows that track 2 · N · \|ω\| / 2π; least squares of ln RMS on ln f_m. Reported with its standard error; p = 0 when the error is larger than \|p\| |
| dead band f_dead | default 1 Hz (R3 with slow starts needed) |
| broadband c, p_b | cruise floor between the lines (40–300 Hz, ± 3 Hz around lines left out), minus the sensor noise, per axis, at activity 1 (f_ref as above); p_b = 1 (one speed) |
| load gain, frame, run variation | not identified for use case 1 from one rotation: defaults (no load gain, sensor axes, σ_r = 0), reported |


Version 2 (2026-11-25). Version 1 chose the order line as the strongest cruise line and found modes only in the moving
spectrum. On Zenodo 11096791 it picked a fixed line at 99.9 Hz as the order (N ≈ 400) and found no mode, so the level
at rest was the sensor noise only; use case 1 failed (U1 0/6, U2 4/6, U3 2/6, U4 not judged). Both rules contradicted
the model: orders are by definition the lines that follow the motor speed (design section 7, step 2), and a mode can
exist only at rest (`at_rest_while_moving`, FTV-232). Version 2 corrects those two rules; the acceptance is unchanged.
Both results are kept.

Version 2 also failed use case 1 (U1 2/6, U2 2/6, U3 2/6, U4 not judged: 2 eligible windows). It tracked the order
line correctly (40 Hz follows the speed in 11 of 11 windows; N = 161.6, close to the 160 inferred before) and kept the
fixed lines at 100 and 300 Hz out of the orders. But the rest spectrum of 0.5 s is a single Welch segment, and its
random ripple passed the 3 dB test: 13 modes at rest, 18 in all, with overlapping bands that counted the same energy
several times; the moving level came out 3 to 4 times the recording.

Version 3 (2026-11-25), **exploratory**. It was written after the validation result of version 2 was seen, so its
result does not count as a pre-registered pass; a pass has to be confirmed on new data. It is the last version on this
data set: whatever its result, there is no version 4 here. The acceptance is unchanged. Version 3 changes two rules,
both following from statistics, not from the validation numbers:

1. A peak counts only when it is significant against the floor for the number of averaged segments: a Welch PSD
   averaged over K segments (50 % overlap counted as K/2 independent) is χ²-distributed with 2K' degrees of freedom; a
   bin is a peak when its power exceeds the floor (the running median over ± 5 Hz) by the χ² quantile at 1 − 0.01 / M,
   M the number of bins searched (Bonferroni). The same test applies to the rest, cruise and pooled speed-change
   spectra.
2. Mode bands do not overlap: each frequency bin belongs to at most one mode (the nearest f_n), and s0 and s2 are
   computed in that bin set, so the same energy is counted once.

### 6.3 Output

A configuration file (`force_torque_vibration`, spec section 5) and a report that lists, for every parameter, its
value, the rule used, and whether it was identified, inferred or left at its default.

## 7. Diagnosis first: the data decides the model and the method

Versions 1 to 3 applied one set of rules to every parameter. Each failure was a rule that produced a value the data
could not determine: one speed, yet an order chosen by strength (v1); 0.5 s of rest, yet modes searched at rest (v2).
The identification is therefore organised as a pipeline whose early stages look at the data before any parameter is
estimated:

```text
0. Diagnosis       what the recording holds: rest duration, number of speeds, starts and stops, lift and lower,
                   poses, sample rate and the sensor's filter, contact
1. Identifiability which parameters this recording can determine (section 4, applied to the diagnosis)
2. Model           the structure that fits that map:
                     minimal  sensor noise + one or two modes + the main order line
                     standard + speed exponent, start/stop ramp, keep
                     full     + load gain, world frame, run variation
3. Method          the estimator that fits the data length:
                     long records       averaged spectra (Welch)
                     short records      parametric fits (a sum of damped oscillations, AR models)
                     tap (hammer) tests the free decay after each tap: f_n and ζ directly
4. Estimation      the parameters of the chosen model with the chosen method
5. Validation      held-out data, acceptance written beforehand
```

What is known before any recording is used, not estimated. The diagnosis takes as input what the maker's data,
the arm's physical model and the sensor's documentation already state. A parameter given by a specification is
entered as it is, and the value the recording would give is used only as a check (a mismatch points at the recording,
the procedure or the specification). A parameter that can be computed from specifications and physics (for example a
mode frequency from the link inertia of the physical model and the catalogue stiffness of the gears) is used as the
starting value or the search range. Only the rest is estimated from the recording. Each parameter falls into one of
three classes:

| Class | Treatment | Examples |
|---|---|---|
| Specification | entered as given; the recording only checks it | gear ratio of each joint, gear type (strain-wave: orders 2, 4, …), sensor sample rate and internal filter, sensor noise from the data sheet, tool mass |
| Specification + physics | a starting value or a search range | arm mode frequencies (link inertia + gear stiffness), the sign of the joint load (physical model) |
| Recording | estimated by the procedure | vibration levels (a, s0, s2, c), damping, the friction excitation, run-to-run variation |

The parameter list with the class of each parameter and, for the KUKA recording, where its value came from, is in
section 10.

A parameter outside the identifiability map is not estimated: it keeps its default, takes a documented prior (the
literature, another arm of the same type), or is reported as not identified.

The rules of stages 0 to 3 (for example "with less than 10 s of rest, no mode is taken from the rest") and the way two
model structures are compared (inside the fit data only, for example its first half against its second) are fixed
before the validation data are looked at. Then a recording may lead to a different model than another, and the
procedure stays objective.

How the rules are developed:

1. Explore: find, for each measurement condition, what works best, without restricting which data are looked at.
   Synthetic recordings from the model itself with known parameters, under varied conditions (rest length, number of
   speeds, repeats, noise), give a ground truth: which method recovers which parameter, and from how much data.
2. Rebuild: write what was found as general rules for stages 0 to 3.
3. Confirm: apply the rules, unchanged, to data that were not used while exploring (another public data set, or a
   new recording).

Zenodo 11096791 has been looked at in full by versions 1 to 3, so it serves the exploration from here on; the
confirmation needs other data.

## 8. Version 4: band by band, found by exploration (2026-11-25)

Versions 1 to 3 (section 6) are kept as the record of what did not work. Version 4 was found by exploring Zenodo
11096791 freely (section 7, step 1): starting from the hand fit T14, each part of the model was replaced by a rule,
lowest frequencies first, and kept when the result was as good as T14 on the same yardstick. It is therefore an
in-sample result; it still has to be confirmed on data not used while exploring (section 7, step 3).

The rules follow the structure of the model: each part comes from its own source and is identified from the
evidence that belongs to that source. Three devices recur:

- **The model's own shapes.** A mode is fitted with the spectrum of its own signal x_n (a second-order band-pass of
  white noise, unit variance), not with a generic peak shape.
- **The model run as a probe.** What a part would look like at the sensor (the rest of the model without that part,
  a unit world-fixed mode, a unit load gain) is obtained by running the model on the recorded motion. No orientation
  convention or load sign has to be decided by hand.
- **The spectrum as the window sees it.** A short record blurs a sharp peak; the model spectrum is passed through the
  same analysis window (the expected periodogram) before it is compared.

The number of modes or lines, and the choice between two structures, is made by BIC; never by hand.

| Order | Part | Evidence | Rule |
|---|---|---|---|
| 1 | B, arm modes 5–20 Hz | moving PSD (0.5–5.5 s) | target = recorded PSD; model = (the model without these modes, run as a probe) + Σ σ²_k · shape(f_k, ζ_k). f and ζ shared by the axes, σ per axis by NNLS, log-error fit; K = 1…4 by BIC |
| 2 | B, direction | 0.3 s windows of the 5–20 Hz band, Fx, Fy, Fz | log env²_i(w) = log(base_i(w) + s²_i Q_i(w) + g² P_{d,i}(w)); Q and P from probe runs of a unit sensor-fixed and a unit world-fixed mode along each world axis d; d (or none) by BIC |
| 3 | B, modes at rest | the rest record (one periodogram) | Whittle likelihood of the expected periodogram (model spectrum through the record's Hann window) over the sensor's white noise; K by BIC; a mode more than 1 Hz from every moving mode gets `at_rest_while_moving` 0 |
| 4 | fixed lines 20–300 Hz | moving PSD, order-line bins (± 3 Hz) left out | greedy: add a mode at the largest remaining excess, refit (window-kernel shapes), stop when BIC no longer improves |
| 5 | A, gear orders | the record itself, by order tracking | motor angle φ = N · q; N from the cruise line (k = 2 assumed without a data sheet) and the cruise speed; f_ref = cruise motor frequency. For k = 1…14, amplitude(t) = \|2 · LP_3Hz(x · e^{−ikφ})\|; line² = recorded² − probe-base². Keep k when its cruise line has 3 dB over the base on the force axes, unless its cruise frequency is within 3 Hz of a fixed line (not separable at one speed); per axis only where 3 dB. p and f_dead fitted on order 2 over the whole motion, shared |
| 6 | f_ref change | — | when f_ref changes, rescale every part that uses activity (section 4, How to choose f_ref) |
| 7 | C, broadband | cruise PSD, 40–300 Hz, order and fixed lines (± 3 Hz) left out | c = median excess over the probe base, at activity 1; p_b = 1 with one speed |
| 8 | A, load gain | order-2 tracked amplitude in the cruise | lifting and lowering windows from a probe run of the model (one order, gain 1 / 2); ratio g−/g+ from the cruise windows (convention `positive` = 1); lag by a grid on the transition; order amplitudes rescaled to the lifting value |
| 9 | run variation | two recordings of the same motion | per component group, σ_r = \|ln(A₂ / A₁)\| / √2; components that change together share a `run_group` |

Result on Zenodo 11096791, in-sample (`1-baseline`; the rest record; `2-vibrations` for step 9 only), harness of the
KUKA evaluation, 20 seeds, simulation / recording, median:

| Yardstick | T14 (by hand) | Version 4 (rules only) |
|---|---|---|
| 5–20 Hz while moving (6 axes) | 0.77–1.30 | 0.76–1.08 |
| 20–60 Hz | 0.75–1.19 | 0.67–1.05 |
| 60–150 Hz | 0.76–1.11 | 0.83–1.44 |
| 150–300 Hz | 0.84–1.08 | 0.97–1.27 |
| cruise, 20 Hz low-pass: recording inside the 5–95 % of the runs | 6 of 6 axes | 4 of 6 |
| Fz 5–20 Hz against \|cos θ\| (recording 0.80) | 0.86 | 0.82 |
| at rest, controller view: recording inside the 5–95 % | 4 of 6 (0.59–1.28) | 6 of 6 (0.73–1.26) |
| order 2, lowering / lifting (recording 1.27) | 1.46 [1.40, 1.64] | 1.43 [1.25, 1.53] |

What the rules found by themselves, against T14: a rest-only mode at 9.30 Hz (T14 9.33 Hz), the world axis y (the
same as T14), fixed lines at 99.9 and 301.4 Hz (T14 100.9 and 299.9 Hz), the gear ratio 160.3 (T14 160), the even
orders 2–14 (as T14) plus 3 and 11, f_dead 20 Hz (as T14), a load-gain ratio of 1.42 (the recordings showed 1.36 and
1.49; T14 1.95). They did not take the 33 Hz mode of T14.

Known weak points: with a large run variation (fixed lines σ_r 1.28), the median over runs moves up (log-normal),
which shows in 60–150 Hz on Fz; the amplitudes are taken from one recording while σ_r comes from two, so the
configured values are not yet the median over recordings. The 40 Hz rest mode sits at the edge of its search range.

The implementation of these rules is part of Sensor PRO (private), like the model itself; this document is the
method.

### 8.1 Version 4 depends on its starting point

Started from an empty configuration instead of T14 (three passes, each starting from the previous result), version 4
did not return to its T14-started result. Stable over the passes: the rest modes, the gear ratio, f_dead and the load
gain, all stages that use only their own evidence. Unstable: the 5–20 Hz moving modes, the fixed lines, the list of
orders and the broadband floor, all stages that fit "the recording minus the rest of the model". With the rest of the
model missing or wrong, a stage absorbs another part's energy (eight fixed lines in the first pass, no broadband),
and the next stage builds on that. T14 had held these stages near the right answer; version 4 was not yet independent
of the hand fit.

### 8.2 Version 5: own evidence first

The parts are taken in the order of how little they depend on the others, and the stages that can be decided
locally no longer subtract the model:

| Order | Part | Evidence, without the rest of the model |
|---|---|---|
| 1 | B, modes at rest | the rest record only (as version 4, step 3) |
| 2 | A, gear orders | order tracking; the line is compared with the floor right beside it (the same demodulation at k · φ shifted by ± 5 Hz), not with a model base. N, f_ref, p, f_dead as version 4 |
| 3 | fixed lines | narrow peaks of the cruise PSD that are not order lines, against the running median floor (± 5 Hz, the peak ± 1.5 Hz left out); greedy with BIC, window-kernel shapes |
| 4 | C, broadband | cruise PSD between the lines of 2 and 3, minus the sensor's own white noise (from the rest record), not minus a model base |
| 5 | B, moving modes and direction | last: the model with 1 to 4 run as the base; then version 4, steps 1 and 2 |
| 6 | load gain, run variation | as version 4, steps 8 and 9 |

Version 5 is accepted as independent of the starting point when, from an empty configuration and from T14, it gives
the same structure (number of modes and lines and their frequencies within 0.5 Hz, world axis, list of orders) and
amplitudes within 10 %, and a second pass from its own result changes nothing beyond those tolerances.

Result of version 5 on Zenodo 11096791: from an empty configuration, from T14 and from its own result, the
configuration came out byte for byte the same, so the procedure no longer depends on the hand fit. In-sample, the
controller band and the rest are as good as T14 or better (5–20 Hz 0.88–1.13; cruise and rest 6 of 6 axes inside;
load ratio 1.34 against 1.27 recorded), 150–300 Hz reaches 1.60. One decision is wrong: the 99.8 Hz line was taken as
order 5 (5 × 19.95 Hz). At one cruise speed a fixed line that sits on an order frequency cannot be told from that
order; version 2 had tracked this line through the speed-up and slow-down and found that it does not follow the
speed. The speed exponent came out at the end of its range (2.0), possibly for the same reason.

### 8.3 Version 6: an order must follow the speed

Version 5, with one rule added to step 2: a candidate order k is accepted only when its line follows the motor speed.
In the speed-up and the slow-down (0.3 s windows, step 0.1 s) where k · f_m lies between 20 and 300 Hz, the strongest
peak within ± 4 Hz of k · f_m must lie within ± 1.5 Hz of k · f_m in at least 80 % of the windows. A candidate that
fails, or that has fewer than five such windows, is not an order: its cruise line goes to the fixed lines (step 3).
This is the definition of an order (section 3.1 of the design), now used as the test.

Result of version 6: the 99.8 Hz line went to the fixed lines (order 5 followed the speed in 5 of 15 windows), and
the configuration stayed independent of the starting point. But the test was too strict: every order but 2 failed
(orders 4 and 6 in 36 % and 20 % of the windows) although their cruise lines stand 25 and 112 times above the floor
beside them. With 0.3 s windows the spectrum resolves about 3 Hz; a high order sweeps quickly during the speed-up and
smears within a window, and the strongest peak near the prediction is often another line. The 120 Hz line of order 6
became a fixed line, and 60–150 Hz rose to 1.87.

### 8.4 Version 7: which hypothesis carries more energy

Version 6 with its test replaced. A candidate order k is tested on the speed-up and slow-down samples (|ω| ≥ 5 °/s
outside the cruise, k · f_m between 20 and 300 Hz) by extracting its line in two ways and comparing the energies:

- following the speed: demodulation at k · φ (order tracking), as in step 2;
- fixed in frequency: demodulation at the cruise frequency f_c = k · f_ref, a constant.

Each energy is taken over the force axes and less the floor beside it (the same demodulation shifted by ± 5 Hz). An
order line carries its energy along k · φ and passes f_c only briefly; a fixed line does the opposite. The candidate
is an order when E_follow > E_fixed; otherwise its cruise line goes to the fixed lines. No window, no peak picking.

Result of version 7: orders 2, 4, 6 and 14 were taken, and the 99.8 Hz line went to the fixed lines (E_fixed is 13.6
times E_follow). The configuration stayed independent of the starting point and the earlier rules reproduce their
outputs. Orders 8, 10 and 12 were not taken: during the speed-up and slow-down their energy less the floor is negative,
so the speed changes give no evidence either way. A likely cause, not checked: at low speed the order spacing f_m is
below 5 Hz, so the floor 5 Hz beside a line falls on its neighbour orders. 60–150 Hz improved from 1.87 to 1.41 and
150–300 Hz went from 1.39 to 1.69. p stays at its upper bound 2.0.

### 8.5 Natural frequencies from taps (exploration, 2026-11-25)

The second public recording (2-vibrations) repeats the free rotation with hammer taps. The motion is the same as in
the first (aligned within 0.5 ms below 2 Hz), so the first gives, at the same time and pose, the vibration without a
tap. Subtracting it does not work above 5 Hz: the arm modes are random (part B) and the difference only adds them.
The first recording is therefore used as the background level, not subtracted.

Rules:

1. Taps: a jump of more than 1 N between samples, at least 0.2 s apart; keep a tap when its 5–40 Hz envelope just
   after it is 5 times or more the level before it (median over the axes).
2. Keep only taps whose ringing no order line crosses: during the cruise. In a speed change the order lines sweep
   through the band.
3. Band-pass around the mode (8–16 Hz here). Measure while the envelope is above 3 times the background, the same
   time window of the recording without taps.
4. Frequency from the slope of the phase, damping ratio from the slope of the log envelope.
5. In the other stages, exclude each tap until its envelope falls below 3 times the background, not a fixed time.

Result: of 15 jumps, 4 were strong taps and 2 of them in the cruise. Both rang in F_y and T_x for about 0.85 s, at
12.05 and 12.56 Hz with a damping ratio of 0.008 and 0.009; the motion gave 12.4 Hz with 0.005. The other axes rang for
0.1 to 0.2 s. The tap in the speed-up rang weakly (12.9 Hz, 0.45 s) and the one in the slow-down was disturbed by the
order lines (ζ 0.13). The present tap exclusion of 0.4 s is too short for F_y and T_x. The 10 Hz mode and the modes
from 20 to 40 Hz were not found. With less than 1 s of ringing the damping is known only to about a factor of 2.

### 8.6 What we want: the sensor noise, reproduced (2026-11-25)

The aim is not to identify the arm but to reproduce the noise the force controller sees. The parameters A, B and C are
the means; they matter as far as the noise they make matches the real one, also at other speeds and poses. Two
recordings may give slightly different parameters and the same noise, and that is enough.

The ringing after a contact is part of this noise: a force-controlled arm touches things. So the taps of the second
recording are not something to remove but something to reproduce. Removing them only by subtraction does not work
either: the ringing and the motion vibration sound in the same arm modes, and a flexible fit takes the motion
vibration with the ringing (what is left falls to 0.13–0.3 of the recording without taps), a stiff one leaves the
ringing (2.6–3.3 times). The right amount would be the vibration without the tap, which is what is to be measured.

**Hammer model (exploration).** A tap is an impulse into the resonator of an arm mode; nothing else changes. Each
mode of part B is already a resonator driven by noise; the tap adds an input:

    x_n'' + 2 ζ_n ω_n x_n' + ω_n² x_n = (noise, as now) + J · δ(t − t_tap)
    added output, per axis i:  h_{n,i} · x_n

with t_tap the tap time, J the impulse and h_{n,i} the new parameter, how strongly a tap shows on axis i.

Test, outside MuJoCo: the 12.39 Hz mode of version 7 (ζ 0.005, from the first recording only), an impulse at each of
the 4 strong taps, h calibrated on one tap and the others predicted. The vibration without the tap is the first
recording at the same time, so only the ringing is modelled.

| | Real (2 taps in the cruise) | Model |
|---|---|---|
| frequency | 12.05, 12.56 Hz | 12.4 Hz |
| axes | F_y, T_x | F_y, T_x (from the calibration) |
| time above 3 times the background | 0.84, 0.85 s | 0.85 s |
| damping ratio | 0.008, 0.009 | 0.005 |

How it rings is reproduced by the model fitted on the first recording. The model rings about 1.7 times longer; with
taps 1 s apart this does not show in the time above the background. The damping from the taps is the better value.

How strongly it rings is not. Three inputs were tried for J: the size of the force spike, its F_x component and its
F_y component. None gives the size and the sign of the ringing over the taps: the best case got the size (7.7 against
8.2 times the background) with the waveform inverted; another gave a third of the size. Likely cause, not checked:
the impact lasts about 1 ms and the sensor takes one or two samples of it, so the spike depends on where the samples
fall; where and in which direction the hammer hit is not recorded. The taps in the speed-up and the slow-down ring
differently (13 to 14 Hz, ζ 0.02 to 0.13), probably disturbed by the order lines.

With the impulse of each tap left free (signed), and even with the frequency of each tap free, F_y and T_x of the
second cruise tap still keep 3 to 6 times the background: a single mode decaying at one frequency does not describe
the ringing. Not checked: two close modes beating (12 to 13 Hz showed several peaks), or the mode moving while the
arm turns about 50° during the ringing. Two usable taps cannot settle this without fitting anything.

**Decision: the parameters come from the first recording (free rotation); taps only check the arm modes found there**
(frequency, damping, the axes they show on). They are not
used to reproduce the noise of a motion with taps in it: the sensor noise during a sequence carrying that ringing is
too complex for what use case 1 needs, and the strength of the ringing is not part of the noise model. Consequences:

- The second public recording is a modal test, not a validation recording of the motion noise. The acceptance file
  written before fitting (Sensor PRO, `criteria-force-control.md`) used it, with 0.4 s around each tap removed, for
  U2 to U4; the ringing lasts about 0.85 s, so that validation is dropped before being run. The motion noise has no
  held-out public recording of the same setup left; it is confirmed on other data (section 9).
- The hammer model (impulse input, h_{n,i}) stays an idea for scenarios with contacts; it is not developed now.

## 9. Next steps

1. Check, with the taps of the second recording, the arm modes identified from the first: frequency, damping and axes.
   So far: 12.4 Hz rings at 12.05 and 12.56 Hz on F_y and T_x, damping 0.008–0.009 against 0.005; the 10 Hz mode and
   20–40 Hz not seen. A mismatch is reported, not fed back into the parameters.
2. Confirm the rules, unchanged, on data not used while exploring: another public data set (properties such as order
   lines that follow the speed and fixed modes) or a new recording of a customer arm following section 3.

## 10. Parameter list: from specifications or from the recording

Class: **S** specification, **SP** specification + physics (start or range), **R** recording. For Zenodo 11096791
(KUKA LBR Med 14 R820, ATI Gamma), the source actually used today and what a public specification says (checked
2026-11-25; "not found" means not found in public documents so far).

| Parameter | Key | Class | KUKA: value used | Public specification |
|---|---|---|---|---|
| gear ratio N_j | `gear_ratio` | S | 160.3, from the recording (order 2 assumed) | **160:1** for A6 and A7 of the LBR iiwa 14 R820 (A1–A4 160:1, A5 100:1), research document using KUKA's nominal model; the recording agrees within 0.19 % |
| gear type, orders k | `orders[].order` | S | strain-wave assumed (k = 2, 4, …) | strain-wave (harmonic) gears in all seven axes, research documents on the LBR Med and the iiwa; part numbers not published |
| order amplitudes a, exponent p | `orders[]` | R | identified (p 1.5) | — |
| dead band f_dead | `deadband_hz` | R | 20 Hz, identified | — |
| reference f_ref | `f_ref_hz` | (chosen) | cruise motor frequency 19.95 Hz | — |
| load gain g+, g−, lag | `load_gain` | R (sign: SP) | ratio 1.42, lag 0; lifting/lowering from the physical model | — |
| mode f_n, ζ | `modes[]` | SP → R | identified (9.3, 10.5, 12.4, 13.6 Hz …) | joint torsional stiffness A6, A7: nominal 20 kN·m/rad (identified 21.9 in a research study); with the link inertia of the physical model this gives a starting value for the arm modes (not done yet) |
| mode levels s0, s2, keep, frame | `modes[]` | R | identified | — |
| fixed lines (99.9, 301 Hz) | `modes[]` | R | identified | — |
| broadband c, p_b | `broadband` | R | identified (p_b = 1 default) | — |
| run variation σ_r | `run_log_sigma` | R | from two recordings | — |
| sensor sample rate | adapter `update_rate` | S | 697.8 Hz from the time stamps | 698.3 Hz in the data set's paper (ATI Net Box, Ethernet RDT; the RDT rate is set by dividing 7000 Hz) |
| sensor range and resolution | adapter `range` | S | not used (fixed large range) | Gamma SI-32-2.5 / SI-65-5 / SI-130-10: Fx,Fy 32 / 65 / 130 N, Fz 100 / 200 / 400 N, Tx,Ty,Tz 2.5 / 5 / 10 N·m; resolution e.g. SI-65-5 Fx,Fy 1/80 N, Fz 1/40 N (which calibration the recording used: not found) |
| sensor noise | adapter `noise` | S → R | from the rest record | resolution only (effective resolution after dropping four counts of noise); the Net Box low-pass setting used in the recording is not stated (choices: off, 5 … 3000 Hz) |
| sensor resonance | — | S | above the band used | Gamma: about 1400 Hz (Fx, Fy, Tz), 2000 Hz (Fz, Tx, Ty) |
| tool mass and centre of mass | (truth) | S or R | 0.9323 kg, about 44 mm, from the 24-pose calibration recording | 0.932 kg, (0, 0, 44) mm in the data set's paper: agrees |
| arm payload, reach, axis speed | — | S | cruise 45 °/s | LBR Med 14 R820: 14 kg, 820 mm; iiwa 14 R820 A6, A7: 135 °/s at rated payload |

Not found in public documents: the gear ratios and gear part numbers of the LBR Med 14 itself, which joint the data
set turned, the Gamma calibration (SI-32-2.5 / SI-65-5 / SI-130-10) and the Net Box filter used.

Sources: [Skrede, RCAR 2024 (the data set's paper)](https://doi.org/10.1109/RCAR61438.2024.10671273),
[Ban, TU Wien 2024, tables A.4, A.5, 4.3](https://repositum.tuwien.at/bitstream/20.500.12708/201285/1/Ban%20Pascal%20Frederic%20-%202024%20-%20Verbesserung%20der%20Absolut-%20und%20Regelgenauigkeit%20des...pdf),
[KUKA LBR iiwa 14 R820 data sheet](https://www.kuka.com/-/media/kuka-downloads/imported/8350ff3ca11642998dbdc81dcc2ed44c/0000246833_de.pdf),
[ATI Transducer section, Gamma, table 5.75](https://www.ati-ia.com/app_content/documents/9610-05-1033.pdf),
[ATI Net F/T manual](https://www.ati-ia.com/app_content/documents/9620-05-NET%20FT.pdf),
[KUKA LBR Med brochure](https://www.kuka.com/-/media/kuka-corporate/documents/products/lbr-med/broschre_kuka_lbr-med_en_161111.pdf),
[ATI Gamma](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Gamma),
[DLR LWR III mechatronics](https://www.dlr.de/en/rm/research/robotic-systems/arms/lwr-iii/mechatronic-design-of-the-lwr-iii).

