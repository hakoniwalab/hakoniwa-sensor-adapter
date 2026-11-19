# Force-torque sensor: vibration of a moving arm (design draft)

Status: draft for discussion. No requirement IDs yet; they follow once the open questions in section 8 are settled.

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

- Strain-wave (harmonic drive) gearing produces error and torque ripple at 2× the wave-generator (motor) speed and its
  harmonics (4×, 6×); motor control places suppression filters there [1][2]. Measurements also show 1× and 4×
  components and little gear-mesh content [3][4]. Kinematic error is periodic in shaft angle and largely
  independent of speed [5].
- The frequency of these components follows motor speed; speed-scheduled notch filters set their centre in
  proportion to rotation speed [6][7]. Order tracking (resampling at constant shaft-angle steps) is the standard way to
  analyse them under varying speed [8].
- The amplitude seen at the tool is not monotone in speed: speed-synchronous transmission error excites the arm's
  structural resonance when a harmonic crosses it (about 22 Hz on a 6-axis industrial robot) [9].
- Rigid-body compensation (gravity and inertia of the payload) is established [10][11][12]; it removes the slow,
  deterministic part, not the vibration. Learned compensation from joint states also targets the slow part [13][14].
- Collision detection uses thresholds that grow with joint velocity [15][16], an indirect sign that disturbances grow
  with speed.

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
A    = a_{j,k,i} · (f_m,j / f_ref)^p_{j,k}   for f_m,j ≥ f_dead,j, otherwise 0
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
  "vibration": {
    "f_ref_hz": 50,
    "joints": [
      { "name": "joint4", "gear_ratio": 100, "deadband_hz": 1,
        "orders": [ { "order": 2, "exponent": 1.0, "amplitude": { "fx": 0.05, "fy": 0.03, "fz": 0.08, "tx": 0.002, "ty": 0.004, "tz": 0.0005 } } ] }
    ],
    "modes": [
      { "frequency_hz": 9.5, "damping": 0.05,
        "at_rest": { "fz": 0.08, "fx": 0.03 }, "per_activity": { "fz": 0.05 } }
    ],
    "broadband": { "exponent": 1.0, "density_per_activity": { "fx": 0.01, "fz": 0.006 } }
  }
}
```

Per-axis values use the same keys as the rest of the force-torque configuration (groups or axes, FTS-ADP-520).

## 7. Fitting from data

1. Record at rest, then the same motion at three or four speeds (at least two).
2. Spectrogram against motor frequency N·|q̇|: lines whose frequency moves with speed are orders, fixed lines are
   modes.
3. Orders: fit a and p from the line amplitude at each speed. Modes: f_n and ζ from the peak, s0 at rest, s1 from the
   speeds. Broadband: c and p_b from the floor at each speed.
4. Validate on a motion or speed not used for fitting.

With Zenodo 11096791 alone (one speed) the modes, the at-rest levels and the broadband level at that speed can be
fitted; the orders cannot be told apart from modes.

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

## 8. Open questions

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

Links as found on 2026-11-19. Items marked † were not opened in full; the summary comes from the abstract or the
search result.

1. Sanyo Denki, Technical Report TR5e (1998): ripple suppression at 2×, 4×, 6× motor frequency in strain-wave
   drives. <https://sanyodenki.com/archive/document/corporatedata/technicalreport/1998/TR5e_s_en.pdf> †
2. Lu et al., Electrical Engineering (Springer), 2× torque ripple of a harmonic drive.
   <https://link.springer.com/article/10.1007/s00202-012-0264-4> †
3. Vibration test of a harmonic drive: 1×, 4× of the wave generator, resonance near 30 Hz. <https://extrica.com/article/19378> †
4. Harmonic drive dynamic model, little gear-mesh content. MDPI Machines 13(12). <https://www.mdpi.com/2075-1702/13/12/1083> †
5. Shi et al., kinematic error of harmonic drives as position harmonics, largely speed independent. † (no stable link found)
6. US 2016/0016310, notch filter centre proportional to rotation speed. <https://uspto.report/patent/app/20160016310> †
7. WO 2004/029634 A2, speed-scheduled notch filter. <https://patents.google.com/patent/WO2004029634A2/en> †
8. Order tracking of robot cycloid reducers, Jixie Chuandong (2022). <https://doaj.org/article/aea9ea0fbecd4a42a33576a443403b61> †
9. Iwasaki and Nakamura, IFAC 2014: speed-synchronous transmission error exciting the resonance of a 6-axis robot.
   <https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/0659.pdf> †
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
18. Skrede et al., RCAR 2024, contact-wrench estimation; data: Zenodo 11096791. <https://zenodo.org/records/11096791>

Before the requirements are written, each reference that a requirement relies on is to be read in full and its
statement checked; the † marks show which are still to be checked.
