# Rangefinder specification

The common rules are in the [common specification](README.md). The design is in [range-image-adapter.md](../design/range-image-adapter.md).

## 1. What it measures

The rangefinder measures the distance to the nearest object along the sensor x axis. It distinguishes laser, infrared and ultrasound.

| ID | Requirement |
|---|---|
| RIS-RNG-010 | The output is `RangeSample`: radiation type (`radiation_type`: `laser`, `infrared`, `ultrasound`), field of view (the larger full angle of the cone; 0 for a single ray), minimum and maximum, range, and variance |
| RIS-RNG-020 | The cone (`cone`) is written as the horizontal full angle h, the vertical full angle v, and the number of rays n. If omitted, there is one ray (the sensor x axis). h and v are 0 or more; n is 1 or more |
| RIS-RNG-030 | The n rays are placed on a grid with side s = ⌈√n⌉. Ray i (from 0) has fu = (i mod s) / (s − 1) − 0.5 and fv = ⌊i / s⌋ / (s − 1) − 0.5 (both 0 when s = 1), yaw fu·h and pitch fv·v |
| RIS-RNG-040 | The true range is the smallest value of hit distance × (x component of the ray direction) over all rays that hit (the distance projected on the sensor x axis). If no ray hits, it is +inf |
| RIS-RNG-050 | The ray cast length is maximum ÷ (smallest x component of the ray directions). This lets off-axis rays measure up to the maximum after projection on the axis |
| RIS-RNG-060 | The variance is σ squared at the true range. It is 0 when nothing is hit |
| RIS-RNG-070 | The log column is `true_range` [m] (the true range at the last sample) |

## 2. Configuration

Schema: `$defs/rangefinder` in [schemas/sensor-definition.schema.json](../../schemas/sensor-definition.schema.json).

```json
{
  "type": "rangefinder",
  "update_rate": 50,
  "radiation_type": "laser",
  "range": { "min": 0.05, "max": 40.0, "resolution": 0.01 },
  "cone": { "horizontal": 0.0, "vertical": 0.0, "rays": 1 },
  "accuracy": [ { "range": { "min": 0.05, "max": 1.0 }, "stddev": 0.025 },
                { "range": { "min": 1.0, "max": 40.0 }, "percent": 1.0 } ],
  "no_return": "inf"
}
```
