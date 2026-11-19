"""Draws the concept figures of the error-characteristics pages, in Japanese and/or English.

    python3 make_all.py --out <dir> [--lang ja|en|both]

Writes concept-<n>-<name>-<lang>.png. The figures explain one stage of the common processing order each; their
data are illustrative, made with numpy, not measured. The measurement cards are made by ../cards_*.py.
Needs numpy, matplotlib and a Japanese font (Hiragino Sans on macOS).
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIGURES = ["pipeline", "mount", "noise", "offset", "drift", "timing", "range"]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True)
    parser.add_argument("--lang", choices=["ja", "en", "both"], default="both")
    args = parser.parse_args()
    langs = ["ja", "en"] if args.lang == "both" else [args.lang]
    Path(args.out).mkdir(parents=True, exist_ok=True)
    for lang in langs:
        for n, script in enumerate(FIGURES, start=1):
            out = Path(args.out) / f"concept-{n}-{script}-{lang}.png"
            subprocess.run([sys.executable, str(HERE / f"{script}.py"), str(out), lang], check=True)


if __name__ == "__main__":
    main()
