# Depth camera specification

The common rules (pinhole camera, range error model) are in the [common specification](README.md). The design is in [range-image-adapter.md](../design/range-image-adapter.md).

| ID | Requirement |
|---|---|
| RIS-DEP-010 | The depth type (`depth_type`) is `planar` (default; distance from the camera plane) or `perspective` (distance along the ray from the camera point) |
| RIS-DEP-020 | The method (`method`) is `ray` (default) or `render`. `ray` needs the host's ray caster; `render` needs the host's renderer |
| RIS-DEP-030 | `ray`: one ray is cast per pixel. Planar distance = hit distance × (x component of the ray direction); along-ray distance = hit distance. The cast length is maximum ÷ (smallest x component) for `planar`, and the maximum for `perspective` |
| RIS-DEP-040 | `render`: the renderer's depth (planar distance) is used. Along-ray distance = planar distance ÷ (x component of the ray direction). The render near and far planes are written in `clip`. If omitted, near = max(0.01, minimum / 2) and far = 2 × maximum ÷ (smallest x component) |
| RIS-DEP-050 | The range error model is applied to the output depth (planar or along-ray distance) |
| RIS-DEP-060 | One image measures all pixels at once, on one capture step |
| RIS-DEP-070 | The output is `DepthImage`: depth type, width and height, the depth array (m, rows from the top), and `CameraInfo` |
| RIS-DEP-080 | Only with `ray` may different instances run on different threads (`thread_safe_across_instances` in the capability table). With `render`, this follows the host renderer's rules |

## Configuration

Schema: `$defs/depth_camera` in [schemas/sensor-definition.schema.json](../../schemas/sensor-definition.schema.json).

```json
{
  "type": "depth_camera",
  "update_rate": 25,
  "image": { "width": 160, "height": 120 },
  "horizontal_fov": 1.51843644923507,
  "depth_type": "planar",
  "method": "ray",
  "range": { "min": 0.3, "max": 10.0, "resolution": 0.001 },
  "accuracy": [ { "range": { "min": 0.3, "max": 10.0 }, "percent": 2.0 } ],
  "no_return": "inf"
}
```
