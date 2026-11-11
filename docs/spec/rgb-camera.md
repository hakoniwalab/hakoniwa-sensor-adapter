# RGB camera specification

The common rules (pinhole camera) are in the [common specification](README.md). The design is in [range-image-adapter.md](../design/range-image-adapter.md).

| ID | Requirement |
|---|---|
| RIS-RGB-010 | The adapter asks the host renderer for colour only (width and height, horizontal FOV, near and far). If `clip` is omitted, near is 0.05 m and far is 1000 m. 0 < near < far |
| RIS-RGB-020 | Noise (`noise`): for each colour channel of each pixel, 255 × (mean + stddev × N(0, 1)) is added, the result is rounded to an integer and clamped to 0–255. Mean and stddev are in units of the full scale (0–1). If omitted, there is no noise. stddev is 0 or more |
| RIS-RGB-030 | The output is `RgbImage`: width and height, bytes per row (width × 3), an rgb8 array (rows from the top), and `CameraInfo` |
| RIS-RGB-040 | If the renderer returns no colour, it is an error |
| RIS-RGB-050 | Whether different instances can run on different threads depends on the renderer, so the capability table reports false |

## Configuration

Schema: `$defs/rgb_camera` in [schemas/sensor-definition.schema.json](../../schemas/sensor-definition.schema.json).

```json
{
  "type": "rgb_camera",
  "update_rate": 25,
  "image": { "width": 320, "height": 240 },
  "horizontal_fov": 1.2,
  "clip": { "near": 0.05, "far": 200 },
  "noise": { "mean": 0.0, "stddev": 0.01 }
}
```
