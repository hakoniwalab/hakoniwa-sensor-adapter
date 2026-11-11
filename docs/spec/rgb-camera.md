# RGB camera specification

The common rules (pinhole camera) are in the [common specification](README.md). The design is in [range-image-adapter.md](../design/range-image-adapter.md).

| ID | Requirement |
|---|---|
| RIS-RGB-010 | The adapter asks the host renderer for colour only: one `RenderRequest` per camera with the camera pose of the step, width and height, intrinsics K from the horizontal FOV (square pixels, fx = fy, so every renderer can draw it; principal point at the image centre; pixel centres at integer coordinates), near and far, and a destination it owns. If `clip` is omitted, near is 0.05 m and far is 1000 m. 0 < near < far |
| RIS-RGB-020 | Noise (`noise`): for each colour channel of each pixel, 255 × (mean + stddev × N(0, 1)) is added, the result is rounded to an integer and clamped to 0–255. Mean and stddev are in units of the full scale (0–1). If omitted, there is no noise. stddev is 0 or more |
| RIS-RGB-030 | The output is `RgbImages`: one `RgbImage` per camera of the rig, each with the camera name, width and height, bytes per row (width × 3), an rgb8 array (rows from the top), and `CameraInfo` |
| RIS-RGB-040 | If the renderer returns no colour, it is an error |
| RIS-RGB-050 | Whether different instances can run on different threads depends on the renderer, so the capability table reports false |
| RIS-RGB-060 | A sensor is a rig of 1..N cameras (`cameras`). Without `cameras` it is one camera at the rig origin with an empty name. Each camera's pose in the rig (`position_m` [m] and `orientation_deg` [deg], rig axes FLU) is composed with the rig pose of the step. `orientation_deg` is the rotation from the camera axes to the rig axes, R = Rz(yaw) · Ry(pitch) · Rx(roll), the same convention as the host's mounts. All cameras share the image size, field of view, clip and noise of the file. The capability table reports the largest number of cameras (`max_cameras`) |
| RIS-RGB-070 | All cameras of a rig are asked for in one render call, so they show the same physics step, and they have the same sample time |
| RIS-RGB-080 | Noise uses a separate random stream per camera (derived from the seed, the vehicle, the sensor and the camera name), so the cameras of a rig do not share noise. A sensor without `cameras` uses the same stream as a single RGB camera. How the name enters the derivation is up to the implementation: the same seed and inputs give the same images in one implementation, but bit-identical noise across implementations is not required |
| RIS-RGB-090 | Stereo (two cameras): the first camera is the left one. The cameras must have the same orientation and differ in position only along the rig's y axis; otherwise creation fails. The right camera's `CameraInfo` P carries the baseline b (the distance between the two cameras): P[3] = -fx × b (ROS stereo). The left camera's P[3] is 0 |

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

A stereo rig with a 0.12 m baseline (left first, the right camera 0.12 m to the right, that is −y):

```json
{
  "type": "rgb_camera",
  "update_rate": 15,
  "image": { "width": 640, "height": 480 },
  "horizontal_fov": 1.6,
  "cameras": [
    { "name": "left", "position_m": [0, 0.06, 0] },
    { "name": "right", "position_m": [0, -0.06, 0] }
  ]
}
```
