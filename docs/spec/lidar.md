# LiDAR specification

This covers 2D and 3D LiDAR (one vertical row means 2D). The common rules are in the [common specification](README.md). The design is in [range-image-adapter.md](../design/range-image-adapter.md).

## 1. Scan grid

| ID | Requirement |
|---|---|
| RIS-LID-010 | The horizontal (`scan.horizontal`) and vertical (`scan.vertical`) axes are each written as a sample count and minimum and maximum angles. Samples are evenly spaced from minimum to maximum, both ends included (step = (maximum − minimum) / (samples − 1)). With one sample, only the minimum angle is used. If the vertical axis is omitted, there is one row (angle 0). The sample count is 1 or more; the maximum is not less than the minimum |
| RIS-LID-020 | The horizontal angle az is about the sensor z axis (0 is forward, positive is left). The vertical angle el is measured from the xy plane (positive is up). The ray direction is (cos el · cos az, cos el · sin az, sin el) |
| RIS-LID-030 | The output grids (`ranges`, `intensities`) are row-major. Rows are vertical rows (the lowest is 0); columns follow the horizontal angle order |

## 2. Timing (rotating and instant scans)

| ID | Requirement |
|---|---|
| RIS-LID-040 | `scan_mode` is `rotating` (default) or `instant`. One sample period is one scan |
| RIS-LID-050 | `rotating`: with a period of P steps and H horizontal samples, column h (all vertical rows) is cast on step ⌊h·P / H⌋ counted from the start of the period |
| RIS-LID-060 | `rotating`: on the last step of the period, one scan is output (`updated = true`). `sample_time_ns` is the time at the start of the period, `scan_time` is the period length, and `time_increment` is period / H |
| RIS-LID-070 | `instant`: all rays are cast at once on the first step of the period, and the scan is output on that step. `scan_time` and `time_increment` are 0 |
| RIS-LID-080 | `pose_at_end` is the truth (sensor pose) on the step where the scan is output |

## 3. Points

| ID | Requirement |
|---|---|
| RIS-LID-090 | Points (`points`) include only rays with a finite value. The position is value × ray direction (in the sensor axes of the step on which that ray was cast). `ring` is the row index. `time_offset` is the time [s] from the start of the scan to the step on which that ray was cast. `object_id` is the ID of the object hit. Motion distortion is not corrected: points of one scan are written in sensor axes at different times. `pose_at_end` is only the pose at the end of the scan; aligning the points to one pose (correcting with a pose history) is the receiver's job |
| RIS-LID-100 | Intensity is not provided (`intensities` are all 0; `intensity` in the capability table is false) |
| RIS-LID-110 | Values from a scan in progress are not visible until the scan is output. Between scans, the last output scan is kept with `updated = false` |
| RIS-LID-120 | The ray cast length is the maximum |
| RIS-LID-140 | Outputting a scan does not copy the full scan arrays on the output step (for example, the in-progress and published buffers are swapped) |
| RIS-LID-130 | The log column is `points` (the point count of the last output scan; during a scan it keeps the previous scan's count) |

## 4. Configuration

Schema: `$defs/lidar` in [schemas/sensor-definition.schema.json](../../schemas/sensor-definition.schema.json).

```json
{
  "type": "lidar",
  "update_rate": 10,
  "scan_mode": "rotating",
  "scan": {
    "horizontal": { "samples": 1800, "min_angle": -3.14159265358979, "max_angle": 3.13810199508580 },
    "vertical":   { "samples": 16,   "min_angle": -0.261799387799,  "max_angle": 0.261799387799 }
  },
  "range": { "min": 0.5, "max": 100.0, "resolution": 0.002 },
  "accuracy": [ { "range": { "min": 0.5, "max": 100.0 }, "stddev": 0.03 } ],
  "no_return": "inf"
}
```
