# Force-torque sensor: vibration of a moving arm (design draft)

Status: the requirements are in [spec/force-torque-vibration.md](../spec/force-torque-vibration.md) (`FTV-*`). Open questions 2 and 3 of section 9 stay open; the first version uses per-axis amplitudes and leaves the arm IMU out.

## 1. Why

A wrist force-torque sensor on a real arm measures more than its own noise. Measurements of a KUKA LBR Med 14 with an
ATI Gamma (Zenodo 11096791, CC BY 4.0, about 700 Hz) show three layers:

| Layer | At rest | Arm rotating slowly about one axis |
|---|---|---|
| Sensor white noise (40–300 Hz floor) | reproduced by the noise model of [force-torque-adapter.md](force-torque-adapter.md) within 0.97–1.00 per axis | — |
| Arm vibration, 8–17 Hz | present (servo holding) | present |
| Broadband rise and sharp peaks (about 40, 90 and 200 Hz) | absent | floor 64–350 times higher in PSD |

The second and third layers are not errors of the sensor: the sensor measures a structure that really vibrates. They
belong between the physics and the sensor, like the rotor vibration of the drone sensors
([drone-common.md](../spec/drone-common.md) 4.4), which adds disturbances without changing the physics. No
simulator we checked (Gazebo, Isaac Sim, MuJoCo, Drake) models motion-induced vibration of a force-torque sensor;
their noise is Gaussian plus bias or drift at most. ArduPilot SITL drives IMU vibration from motor speed
(`SIM_VIB_*`), the closest analogue.

## 2. What the literature establishes

Checked in full on 2026-11-19 unless marked †.

- Strain-wave (harmonic drive) gearing produces error and torque ripple at **2× the wave-generator (motor) rotation
  and its even harmonics**:
  - [1] describes a disturbance "of two cycles per motor revolution and its higher harmonics", with band-pass
    filters at 2×, 4× and 6× the motor frequency.
  - [9] models the synchronous transmission error as the 2nd and 4th harmonics of the motor angle.
  - [6] states that the reducer of a robot joint "vibrates at double the frequency of the rotation speed of the
    motor".
  - [3] reports the wave-generator rotation frequency 1× in the axial direction and 4× in the circumferential
    direction.
  - "Little gear-mesh content" [4] † could not be read.
- **The frequency follows motor speed:**
  - [6] sets a notch centre at twice the motor speed.
  - [7] makes the notch centre "change linearly with the angular velocity" (a rotating machine, not a robot).
  - Order tracking by equal-angle resampling is used for robot reducers under varying speed [8] † (abstract only).
- **The amplitude is roughly constant with speed:**
  - [6] states the vibration amplitude "is approximately constant regardless of the rotation speed of the motor".
  - [9] models the error with fixed angle amplitudes.
- **Resonance amplifies it.** It grows when a harmonic k·f_m meets a structural frequency [1][3][9]. On a 6-axis
  industrial robot [9], the tip vibration is excited near 12 Hz (11.5–11.8 Hz, the anti-resonance seen from the
  motor, which is the arm with the motor held). 22 Hz is the motor-side resonance.
- Rigid-body compensation (gravity and inertia of the payload) is established [10][11][12] †. It removes the slow,
  deterministic part, not the vibration. Learned compensation from joint states also targets the slow part
  [13][14] †.
- Collision detection uses thresholds that grow with joint velocity [15][16] †.

## 3. Model

Three components, added to the true wrench in sensor axes every physics step, from the joint velocities of the arm.

### 3.1 Order components (follow motor speed)

For joint j with gear ratio N_j and joint velocity q̇_j, the motor phase advances with the joint:

```text
φ_j(t) = ∫ N_j · q̇_j dt              (motor angle, rad)
f_m,j  = N_j · |q̇_j| / 2π            (motor rotation frequency, Hz)
```

Each listed order k of joint j adds, on each sensor axis i,

```text
w_i += A_{j,k,i}(f_m,j) · sin(k · φ_j + θ_{j,k,i})
A    = a_{j,k,i} · (f_m,j / f_ref)^p_{j,k} · min(1, f_m,j / f_dead,j)   (default p = 0: constant [6][9]; taper to 0 at rest)
```

Following the phase rather than the frequency keeps the tone continuous when the speed changes (order tracking).
θ is drawn once per unit from the vibration random stream. Typical orders: 2 and 4 for strain-wave gearing,
1 for shaft imbalance, pole pairs × 1 for electrical ripple.

Optional: an order's amplitude is multiplied by the gain of the structural modes at its frequency k·f_m
(section 3.2), so that it grows when it crosses a resonance [9].

### 3.2 Structural modes (fixed frequencies)

Narrow-band random vibration at fixed frequencies: white noise through a second-order band-pass at f_n with damping
ζ, scaled to a standard deviation per axis. Each mode has an at-rest level (servo holding; the 8–17 Hz component
of the data) and a level that grows with motion activity:

```text
σ_n,i = s0_n,i + s1_n,i · activity,   activity = Σ_j f_m,j / f_ref
```

### 3.3 Broadband rise with motion

White noise whose density grows with activity, on top of the sensor's own noise:

```text
density_i = c_i · activity^p_b
```

This covers the 64–350 times rise of the 40–300 Hz floor in the data.

## 4. Where it sits: in front of the adapter, implemented in Sensor PRO

```text
joint velocities ─▶ force-torque vibration model ─▶ vibration wrench
true wrench (physics) ───────────────────────────┴─(+)─▶ ForceTorqueTruth ─▶ any force-torque adapter
```

- **Position:** on the host side, in front of the adapter. The vibration is added to the truth, so
  `ForceTorqueTruth` and every adapter stay as they are, and the same vibration can be put in front of any
  implementation (Gazebo too).
- **Public (this repository):** this design, the configuration and the requirements, and an interface header for the
  model, so that hosts and tools can call it and other implementations can provide one.
- **Private (Sensor PRO):** the implementation, delivered as binaries like the sensor models.
- **Reached by name:** a host gets the model from an implementation by name, through an entry point of its own
  (for example `HAKO_SENSOR_HOST_MODEL_ENTRY(hakoniwa)`). `SensorAdapterImplementation` is not extended, so binaries
  built against the current header keep working; an implementation without the model simply has no such entry.

A sketch of the interface (names not final):

```cpp
class IForceTorqueVibration {
public:
    virtual ~IForceTorqueVibration() = default;
    // One physics step: joint velocities [rad/s] in the order of the configuration; returns the wrench to add,
    // in sensor axes.
    virtual bool step(std::uint64_t sim_time_ns, const std::vector<double>& joint_velocity, Wrench& out,
                      std::string& error) = 0;
    virtual bool reset(std::string& error) = 0;  // phases, filters, random streams back to creation
};
// Created from a configuration file (section 6), the physics step and a seed.
```

Its random streams follow the common derivation (seed, vehicle, sensor, characteristic), so the same seed gives the
same vibration, independent of the adapter's own streams.

## 5. Inputs

- Joint velocities q̇_j every physics step (rad/s, joint side), for example from the joint truth that the joint sensor
  adapter already receives, or from MoveIt trajectories replayed in the simulation.
- Gear ratio per joint (N_j). Load or motor current could scale amplitudes; the literature gives no simple law, so
  they are left out of the first version.

## 6. Configuration sketch

```json
{
  "type": "force_torque_vibration",
  "f_ref_hz": 50,
  "joints": [
    { "name": "joint7", "gear_ratio": 160, "deadband_hz": 1,
      "orders": [ { "order": 2, "exponent": 0, "resonance": true,
                    "amplitude": { "fx": 0.05, "fy": 0.03, "fz": 0.08, "tx": 0.002, "ty": 0.004, "tz": 0.0005 } } ] }
  ],
  "modes": [
    { "frequency_hz": 9.5, "damping": 0.05,
      "at_rest": { "fz": 0.08, "fx": 0.03 }, "per_activity": { "fz": 0.05 } }
  ],
  "broadband": { "exponent": 1.0, "density_per_activity": { "fx": 0.01, "fz": 0.006 } }
}
```

Per-axis values use the same keys as the rest of the force-torque configuration (groups or axes, FTS-ADP-520).

## 7. Fitting from data

1. Record at rest, then the same motion at three or four speeds (at least two), or one motion whose acceleration and deceleration sweep the speed.
2. Spectrogram against motor frequency N·|q̇|: lines whose frequency moves with speed are orders, fixed lines are
   modes.
3. Orders: fit a and p from the line amplitude at each speed. Modes: f_n and ζ from the peak, s0 at rest, s1 from the
   speeds. Broadband: c and p_b from the floor at each speed.
4. Validate on a motion or speed not used for fitting.

Zenodo 11096791 has one cruise speed, but its acceleration and deceleration sweep the speed, which is enough to
tell orders from modes (section 7.1); the at-rest levels and the broadband level are fitted at that one cruise
speed.

Open data sets checked on 2026-11-19 (landing pages and file listings only). None has a wrist F/T at 500 Hz or more,
joint states, and the same motion at two or more speeds:

| Data set | License | F/T | Joint states | Speeds | Use here |
|---|---|---|---|---|---|
| [Zenodo 11096791](https://zenodo.org/records/11096791) (KUKA LBR Med 14) | CC BY 4.0 | ATI Gamma, ~700 Hz | orientation ~100 Hz | one slow rotation | modes, at-rest levels, broadband at one speed |
| [REASSEMBLE](https://researchdata.tuwien.ac.at/records/0ewrv-8cb44) (Franka FR3) | CC BY 4.0 | AIDIN AFT200, rate not stated | yes | teleoperated, varying | possible, if the F/T rate is high; 54.8 GiB in one archive, mostly contact |
| [voraus-AD](https://github.com/vorausrobotik/voraus-ad-dataset) (Yu-Cobot) | CC BY-NC-SA 4.0 (non-commercial) | none | 500 Hz, motor and joint speed, torques | speed-up and slow-down within moves | order tracking on the joint side only; licence excludes commercial use |
| [Zenodo 7235963](https://zenodo.org/records/7235963), [7240050](https://zenodo.org/records/7240050) (Fraunhofer IPK cobot) | CC BY 4.0 | yes, rate not stated | yes | one free linear move | only if the bag rate is 500 Hz or more |
| [NIST UR5 degradation](https://data.nist.gov/od/id/754A77D9DA1E771AE0532457068179851962) | NIST open data | none | 125 Hz | speed, payload, temperature varied | how joint-side peaks follow speed, below 62.5 Hz |

Others were rejected for a low rate (100–125 Hz controller-estimated TCP force), missing joint states, restricted
access or a paid licence. A clean separation of orders and modes most likely needs our own recording: rest, then the
same motion at two or more speeds, with a wrist F/T at 500 Hz or more and the joint velocities.

### 7.1 What one recording already showed (2026-11-19)

The free rotation of Zenodo 11096791 (about 180° in 5 s) accelerates for about 1.3 s, cruises at about 45°/s and
decelerates for about 1 s. Short-time spectra of all six axes, tracked against the rotation speed (from the
orientation records), show:

- The strongest line between 8 and 60 Hz moves in proportion to speed: about 19 Hz at 20°/s, 40 Hz at 45°/s.
  - The correlation is 0.973.
  - The slope through the origin gives order × gear ratio = 324, that is order 2 with a gear ratio of about 162.
  - 152 of 154 short windows lie within ±3 Hz of order 2 with a ratio of 160.
- At cruise speed, a comb at 40.0, 80.0, 120.4, 160.8, 200.8, 241.5 and 281.5 Hz: the order-2 line and its multiples.
- Lines near 33 Hz and 100 Hz stay at the same frequency in acceleration, cruise and deceleration: mode candidates.
- In the runs with hammer taps and a push, the taps and the push dominate the low band at times. Where the line shows,
  it lies on the same slope.

So a single motion with acceleration and deceleration can separate orders from modes; separate recordings at two
speeds are not required.

## 8. Scope and assumptions

The model is a simplification on purpose. Its assumptions are numbered so that results can say which ones they rely
on, and each carries its evidence: **measured** (checked on data), **literature** (established, not yet checked on
our data), **inferred** (our reading, to be checked), or **simplification** (known not to hold exactly; kept for a
first version).

| ID | Assumption | Evidence | How to check | If wrong |
|---|---|---|---|---|
| VIB-A1 | The sensor's own white noise is independent of the motion | measured at rest only (Zenodo 11096791) | rest vs motion at a frequency free of vibration | the sensor noise has to grow with motion as well |
| VIB-A2 | Order frequencies are integer multiples k of the motor rotation frequency N·\|q̇\|/2π | literature [1][6][9]; **measured** on Zenodo 11096791 (section 7.1), with the gear ratio inferred | the line frequency against speed, within one motion (acceleration, cruise, deceleration) or at several speeds | the orders are fixed modes, or the gear ratio is wrong |
| VIB-A3 | Order amplitude follows a · (f_m / f_ref)^p, tapered linearly to zero below a dead band; p = 0 by default | literature for p = 0 [6][9]; dead band and p ≠ 0 inferred | amplitude of the tracked line against speed | a table of amplitude vs speed replaces the power law |
| VIB-A4 | Order phases are random per unit and constant | simplification | phase is not observable in a PSD | none for spectra; matters only for time-domain matching |
| VIB-A5 | Structural modes have fixed frequency and damping, independent of arm pose | simplification | two poses at rest | mode frequency becomes a function of pose |
| VIB-A6 | The broadband rise grows with activity = Σ f_m / f_ref | inferred | floor at two or more speeds | another activity measure (max, per joint) |
| VIB-A7 | The direction of each component in sensor axes is fixed (per-axis amplitudes) | simplification | the same motion in two poses | amplitudes must be mapped through the arm geometry (section 9, question 2) |
| VIB-A8 | Components add linearly to the true wrench; the sensor stages after them (low-pass, saturation) act on the sum | literature (superposition) and the design of the adapter | — | — |
| VIB-A9 | Load, temperature, wear and contact do not change the vibration | simplification | out of scope | amplitudes and frequencies drift; not modelled |

**In scope:** order components following motor speed, fixed structural modes (at rest and with motion), a broadband
rise with motion, reproducible from a seed.

**Out of scope (first version):** load- and temperature-dependent vibration, vibration caused by contact or impacts,
pose-dependent modes, non-stationary changes (wear), and the vibration of the arm's IMU (section 9, question 3).

**Known gaps against the real data now:**
- The gear ratio of the rotating joint is not documented; 160 (order 2) is inferred from the fitted slope.
- The joint that turned is not identified beyond "about the sensor y axis".
- Lines that stay at the same frequency in every speed phase (near 33 and 100 Hz) are taken as mode candidates
  (VIB-A5) by inference.

## 9. Open questions

1. **Where the model lives — decided (2026-11-19): in front of the adapter, implemented in Sensor PRO.** It
   computes the vibration wrench from the joint velocities and adds it to the truth, like the drone rotor vibration.
   The specification and the interface header are public; the implementation is private and ships as binaries
   (section 4). `ForceTorqueTruth` and `SensorAdapterImplementation` do not change, so existing implementations and
   binaries keep working, and the same vibration can be added in front of any implementation (Gazebo too), which
   keeps comparisons fair. (Rejected: inside the adapter, which would add the joint velocities to
   `ForceTorqueTruth`, an interface and binary change.)
2. Whether amplitudes are configured per sensor axis directly (simple, as above) or as a force direction per joint
   mapped through the arm geometry (more physical, needs the kinematics).
3. Whether the IMU of an arm (end-effector accelerometer) gets the same model.

## References

Read in full on 2026-11-19 unless marked †; † items were not opened in full (abstract or search result only).

1. M. Itoh, "Self-Compensating Control for Suppression of Rotational Speed Variation Generated in a Geared Servo
   System", SANYO DENKI Technical Report No. 5, May 1998.
   <https://sanyodenki.com/archive/document/corporatedata/technicalreport/1998/TR5e_s_en.pdf>
2. Lu et al., Electrical Engineering (Springer), 2× torque ripple of a harmonic drive.
   <https://link.springer.com/article/10.1007/s00202-012-0264-4> †
3. J. Tan, D. Shi, D. Huang, J. Gu, "Test and analysis of three axis vibration of harmonic drives", Vibroengineering
   PROCEDIA 16, 19–24, 2017. DOI 10.21595/vp.2017.19378. <https://extrica.com/article/19378>
4. G. He, B. Zhou, Y. Zheng, H. Lin, L. Xu, Z. Zheng, "Unraveling the Vibration Mechanism in Robotic Harmonic Drive:
   Coupled Dynamics Under Cyclic Inertial Loading and Installation Errors", Machines 13(12):1083, 2025.
   DOI 10.3390/machines13121083. † (site blocked automated access)
5. Shi et al., kinematic error of harmonic drives as position harmonics, largely speed independent. † (no stable
   link found)
6. F. Zhao, T. Mamba (Yaskawa Electric), "Notch filter, external force estimator, motor control apparatus, and robotic
   system", US 2016/0016310 A1, 2016. <https://patents.google.com/patent/US20160016310A1/en>
7. D. H. Carlson (3M), "Adaptable spatial notch filter", WO 2004/029634 A2, 2004.
   <https://patents.google.com/patent/WO2004029634A2/en>
8. M. Lin, X. Zhang, L. Yao, Y. Gong, J. Zhang, Y. Yang, "Application of Order Tracking Analysis Method in Fault
   Diagnosis of Swing Fatigue Tests of Precision Reducers for Robots", Jixie Chuandong 46 (2022), 156–162.
   DOI 10.16578/j.issn.1004.2539.2022.10.024. † (abstract only)
9. M. Iwasaki, H. Nakamura, "Vibration Suppression for Angular Transmission Errors in Harmonic Drive Gearings and
   Application to Industrial Robots", Proc. 19th IFAC World Congress, Cape Town, 2014.
   <https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/0659.pdf>
10. Kubus, Kröger and Wahl, IROS 2008: payload inertial parameters for F/T compensation (used by ROS
    force_torque_tools). <https://index.ros.org/r/force_torque_tools/> †
11. Lund University: accelerometer and force sensor fusion. <https://lup.lub.lu.se/search/files/5836402/625573.pdf> †
12. Automated continuous F/T bias estimation, arXiv 2403.01068 (2024). <https://arxiv.org/abs/2403.01068> †
13. Mohy el Dine et al. (2018): recurrent-network observer for wrist F/T, KUKA LWR-4 with ATI Gamma.
    <https://jsanch2s.github.io/files/2018_rnnob.pdf> †
14. Wrist F/T compensation on an industrial dual-arm robot, regression vs random forest. <https://core.ac.uk/download/288359083.pdf> †
15. Sotoudehnejad and Kermani, ICRA 2014 and thesis: velocity-dependent collision thresholds. <https://ir.lib.uwo.ca/etd/1829> †
16. Legged manipulator collision detection, arXiv 2207.14745 (2022). <https://arxiv.org/abs/2207.14745> †
17. ArduPilot SITL vibration parameters `SIM_VIB_FREQ_X/Y/Z`, `SIM_VIB_MOT_MAX`. <https://discuss.ardupilot.org/t/how-to-simulate-tuning/102826> †
18. Skrede et al., RCAR 2024, contact-wrench estimation; data: Zenodo 11096791 (CC BY 4.0). <https://zenodo.org/records/11096791>

The gear ratios of the KUKA LBR Med 14 / LBR iiwa 14 joints were not found in an official source.
