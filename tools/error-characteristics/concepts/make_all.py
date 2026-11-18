"""Draws the concept figures of the error-characteristics pages (Japanese for now).

    python3 make_all.py --out <dir>

The figures explain one stage of the common processing order each; their data are illustrative, made with numpy,
not measured. The measured figures are made by ../imu_figures.py and ../mag_baro_gps_figures.py.
Needs numpy, matplotlib and a Japanese font (Hiragino Sans on macOS).
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIGURES = [("pipeline", "concept-1-pipeline-ja"), ("mount", "concept-2-mount-ja"), ("noise", "concept-3-noise-ja"),
           ("offset", "concept-4-offset-ja"), ("drift", "concept-5-drift-ja"), ("timing", "concept-6-timing-ja"),
           ("range", "concept-7-range-ja")]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    Path(args.out).mkdir(parents=True, exist_ok=True)
    for script, name in FIGURES:
        subprocess.run([sys.executable, str(HERE / f"{script}.py"), str(Path(args.out) / f"{name}.png")], check=True)


if __name__ == "__main__":
    main()
