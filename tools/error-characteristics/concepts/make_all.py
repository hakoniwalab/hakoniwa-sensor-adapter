"""Draws the concept figures of the error-characteristics pages, in Japanese and/or English.

    python3 make_all.py --out <dir> [--lang ja|en|both] [--set drone|force-torque|all]

Writes concept-<n>-<name>-<lang>.png for the drone sensors and ft-<n>-<name>-<lang>.png for the force-torque
sensor. The figures explain one stage of the processing order each; their data are illustrative, made with numpy,
not measured. The measurement cards are made by ../cards_*.py.
Needs numpy, matplotlib and a Japanese font (Hiragino Sans on macOS).
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIGURES = ["pipeline", "mount", "noise", "offset", "drift", "timing", "range"]
FT_FIGURES = ["what", "pipeline", "tare", "calib", "noise", "limits"]  # scripts ft_<name>.py


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True)
    parser.add_argument("--lang", choices=["ja", "en", "both"], default="both")
    parser.add_argument("--set", choices=["drone", "force-torque", "all"], default="all")
    args = parser.parse_args()
    langs = ["ja", "en"] if args.lang == "both" else [args.lang]
    Path(args.out).mkdir(parents=True, exist_ok=True)
    jobs = []  # (script, output name without the language)
    if args.set in ("drone", "all"):
        jobs += [(f"{name}.py", f"concept-{n}-{name}") for n, name in enumerate(FIGURES, start=1)]
    if args.set in ("force-torque", "all"):
        jobs += [(f"ft_{name}.py", f"ft-{n}-{name}") for n, name in enumerate(FT_FIGURES, start=1)]
    for lang in langs:
        for script, stem in jobs:
            out = Path(args.out) / f"{stem}-{lang}.png"
            subprocess.run([sys.executable, str(HERE / script), str(out), lang], check=True)


if __name__ == "__main__":
    main()
