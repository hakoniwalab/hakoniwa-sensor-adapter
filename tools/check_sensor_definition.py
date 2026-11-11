#!/usr/bin/env python3
"""Check sensor definition files against schemas/sensor-definition.schema.json.

The schema checks each field; this tool adds the rules that span fields, as an
adapter checks them at creation (range.min < range.max, each accuracy interval
min < max, clip near < far, scan max_angle >= min_angle).

Usage:
  python tools/check_sensor_definition.py FILE...   (exit 1 when any file fails)
  python tools/check_sensor_definition.py --self-test
      Every file in schemas/examples/valid and every JSON example in docs/spec
      must pass; every file in schemas/examples/invalid must fail.
As a module: problems(definition) returns the list of problems of one parsed file.
"""

import json
import re
import sys
from pathlib import Path

import jsonschema

ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = ROOT / "schemas" / "sensor-definition.schema.json"


def _schema():
    with open(SCHEMA_PATH, encoding="utf-8") as stream:
        return json.load(stream)


def problems(definition, schema=None):
    """Problems of one parsed sensor definition; empty when it is valid."""
    validator = jsonschema.Draft202012Validator(schema or _schema())
    found = [f"{'/'.join(map(str, e.absolute_path)) or '(top)'}: {e.message}"
             for e in sorted(validator.iter_errors(definition), key=lambda e: list(e.absolute_path))]
    if found:
        return found
    rng = definition.get("range")
    if definition.get("type") not in ("force_torque", "joint") and rng is not None and not rng.get("min", 0.0) < rng["max"]:
        found.append("range: min must be below max")
    for index, entry in enumerate(definition.get("accuracy", [])):
        if not entry["range"]["min"] < entry["range"]["max"]:
            found.append(f"accuracy/{index}/range: min must be below max")
    clip = definition.get("clip")
    if clip is not None and "near" in clip and "far" in clip and not clip["near"] < clip["far"]:
        found.append("clip: near must be below far")
    if definition.get("type") == "rgb_camera" and clip is not None:
        near, far = clip.get("near", 0.05), clip.get("far", 1000.0)
        if not near < far:
            found.append("clip: near must be below far (defaults near 0.05, far 1000)")
    for axis in ("horizontal", "vertical"):
        values = definition.get("scan", {}).get(axis)
        if values and "max_angle" in values and values["max_angle"] < values.get("min_angle", 0.0):
            found.append(f"scan/{axis}: max_angle must not be below min_angle")
    return found


def self_test(schema):
    failed = 0
    passing = sorted((ROOT / "schemas" / "examples" / "valid").glob("*.json"))
    cases = [(str(p.relative_to(ROOT)), json.loads(p.read_text(encoding="utf-8"))) for p in passing]
    for page in sorted((ROOT / "docs" / "spec").glob("*.md")):
        for index, block in enumerate(re.findall(r"```json\n(.*?)```", page.read_text(encoding="utf-8"), re.S)):
            cases.append((f"{page.relative_to(ROOT)} example {index + 1}", json.loads(block)))
    for name, definition in cases:
        found = problems(definition, schema)
        if found:
            failed += 1
            print(f"FAIL {name} should be valid: {found[0]}")
    invalid = sorted((ROOT / "schemas" / "examples" / "invalid").glob("*.json"))
    for path in invalid:
        if not problems(json.loads(path.read_text(encoding="utf-8")), schema):
            failed += 1
            print(f"FAIL {path.relative_to(ROOT)} should be rejected")
    print(f"{len(cases)} valid, {len(invalid)} invalid checked, {failed} failed")
    return 1 if failed else 0


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    schema = _schema()
    if argv == ["--self-test"]:
        return self_test(schema)
    failed = 0
    for name in argv:
        found = problems(json.loads(Path(name).read_text(encoding="utf-8")), schema)
        if found:
            failed += 1
            for problem in found:
                print(f"{name}: {problem}")
        else:
            print(f"{name}: ok")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
