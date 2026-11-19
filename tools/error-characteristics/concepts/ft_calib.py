"""Force-torque concept figure 4: nonlinearity, scale error and crosstalk (draft, Japanese/English).
Illustrative values; the formulas are docs/spec/force-torque.md FTS-ADP-210 to 230."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, MEAS, BLUE = "#14213d", "#8a96a8", "0.55", "#d1242f", "#2f6fd6"
fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))

# 1. nonlinearity: u + n * 4u(1 - |u|), largest at half range
ax = axes[0]
u = np.linspace(-1, 1, 400)
n = 0.05  # 5 % to make the bow visible
y = u + n * 4 * u * (1 - np.abs(u))
ax.plot(u * 200, u * 200, color=TRUTH, lw=2.5, label=L("理想：出力＝入力", "ideal: output = input"))
ax.plot(u * 200, y * 200, color=MEAS, lw=2, label=L("非線形 5 %（見やすく大きめ）", "nonlinearity 5 % (large, to show it)"))
ax.annotate("", xy=(100, 110), xytext=(100, 100), arrowprops=dict(arrowstyle="<->", color=MEAS))
ax.text(104, 92, L("レンジの半分で\n一番ずれる（+10 N）", "largest error at\nhalf range (+10 N)"), fontsize=9, color=MEAS, va="top")
ax.plot([200, -200], [200, -200], "o", color=INK, ms=5)
ax.text(150, 205, L("端と 0 では\nずれない", "no error at\nthe ends and 0"), fontsize=9, color=INK, ha="right", va="bottom")
ax.set_xlim(-210, 230)
ax.set_ylim(-230, 250)
ax.set_xlabel(L("入力（真値）Fz [N]（レンジ ±200 N）", "Input (truth) Fz [N] (range ±200 N)"))
ax.set_ylabel(L("出力 Fz [N]", "Output Fz [N]"))
ax.set_title(L("① 非線形：まっすぐでない", "① Nonlinearity: not a straight line"), fontsize=12, color=INK, loc="left")
ax.legend(loc=L("upper left", "lower right"), fontsize=8.5)
ax.grid(alpha=0.3)

# 2. scale error: the slope is not 1
ax = axes[1]
x = np.linspace(-200, 200, 50)
ax.axhline(0, color=TRUTH, lw=2.5, label=L("理想（ずれ 0）", "ideal (no error)"))
for s, color, label in ((0.03, MEAS, L("この個体：+3 %", "this unit: +3 %")), (-0.02, BLUE, L("別の個体：−2 %", "another unit: −2 %"))):
    ax.plot(x, s * x, color=color, lw=2, label=label)
ax.text(200, 6.6, "+6 N", fontsize=9, color=MEAS, ha="right")
ax.text(-190, 6.5, L("大きな力ほど\nずれも大きい", "larger force,\nlarger error"), fontsize=9, color=INK)
ax.text(-190, L(4.6, 5.3), L("個体ごとに、作ったときに 1 つ決まる\n（±設定値の範囲でばらつく）", "one value per unit, set at build\n(spread within ± the setting)"), fontsize=8.5, color=MUTED, va="top")
ax.set_ylim(-8, 8)
ax.set_xlabel(L("入力（真値）Fz [N]", "Input (truth) Fz [N]"))
ax.set_ylabel(L("出力 − 入力 [N]（ずれだけを拡大）", "Output − input [N] (error only, zoomed)"))
ax.set_title(L("② スケール誤差：倍率がずれる", "② Scale error: the gain is off"), fontsize=12, color=INK, loc="left")
ax.legend(loc="lower right", fontsize=8.5)
ax.grid(alpha=0.3)

# 3. crosstalk: a pure Fz shows on the other axes
ax = axes[2]
labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
truth = np.array([0, 0, -100, 0, 0, 0], dtype=float)
ranges = np.array([200, 200, 200, 10, 10, 6.5])
E = np.array([0.04, -0.03, 0, 0.05, -0.02, 0.01])  # the Fz column of E, crosstalk up to 5 %
measured = truth + E * (truth[2] / ranges[2]) * ranges  # u = x / R, C u, back to units
measured[2] = truth[2]
pos = np.arange(6)
ax.bar(pos - 0.18, truth / ranges * 100, width=0.36, color=TRUTH, label=L("真値（Fz だけ）", "truth (Fz only)"))
ax.bar(pos + 0.18, measured / ranges * 100, width=0.36, color=MEAS, label=L("出力（ほかの軸にも漏れる）", "output (leaks into other axes)"))
for i in (0, 1, 3, 4):
    v = measured[i]
    unit = "N" if i < 3 else "N·m"
    ax.text(pos[i] + 0.18, measured[i] / ranges[i] * 100 + (1.5 if v > 0 else -1.5), f"{v:+.1f} {unit}" if i < 3 else f"{v:+.2f} {unit}",
            fontsize=8.5, color=MEAS, ha="center", va="bottom" if v > 0 else "top")
ax.axhline(0, color=INK, lw=0.8)
ax.set_xticks(pos)
ax.set_xticklabels(labels)
ax.set_ylim(-60, 15)
ax.set_ylabel(L("レンジに対する割合 [%]", "Fraction of range [%]"))
ax.set_title(L("③ 軸の混ざり：押した軸以外にも出る", "③ Crosstalk: other axes respond too"), fontsize=12, color=INK, loc="left")
ax.legend(loc="lower right", fontsize=8.5)
ax.text(2.55, -24, L("Fz に −100 N（レンジの半分）\nかけると、混ざり 5 % 以内の\n設定で Fx に −4 N などが出る", "−100 N on Fz (half range)\nwith crosstalk up to 5 %\ngives e.g. −4 N on Fx"), fontsize=9, color=INK)
ax.grid(alpha=0.3, axis="y")

fig.text(0.04, 0.02,
         L("3 つとも個体のくせ：作ったときに乱数で 1 つ決まり、ずっと同じ（同じ乱数の種なら同じ値）。"
           "計算は「レンジに対する割合」で行うので、ずれの大きさはレンジに比例する。\n"
           "設定のキー：\"nonlinearity\"（軸ごと）、\"scale_error\"（倍率）、\"crosstalk\"（混ざり）。"
           "データシートの「非線形性」「クロストーク（干渉）」の % をそのまま書く。",
           "All three are unit traits: drawn at random once at build, then fixed (same seed, same values). "
           "They are computed as a fraction of the range, so the error scales with the range.\n"
           "Setting keys: \"nonlinearity\" (per axis), \"scale_error\" (gain), \"crosstalk\". "
           "Write the datasheet's nonlinearity and crosstalk % as is."),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.2, wspace=0.27)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
