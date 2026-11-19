"""Force-torque concept figure 2: the processing order (draft, Japanese/English). From docs/spec/force-torque.md 2.2-2.3."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED = "#14213d", "#8a96a8"
GROUPS = {
    "time": ("#2f6fd6", L("時間：遅れ・なまし・飛び飛び", "Time: delay, smoothing, sampling")),
    "unit": ("#b26a00", L("個体のくせ：作ったときに決まり、ずっと同じ", "Unit traits: set at build, never change")),
    "wander": ("#d1242f", L("時間とともに変わる・測るたびに変わる", "Change over time or with each sample")),
    "limit": ("#5b6475", L("装置の限界", "Device limits")),
    "user": ("#1a7f37", L("使う人の操作", "User action")),
}
ROW1 = [  # every physics step
    (L("過負荷の見張り", "Overload\nwatch"), "overload", L("壊れる強さを\n超えたら印", "flag above the\nbreaking load"), "limit"),
    (L("遅れ", "Delay"), "delay", "", "time"),
    (L("ローパス\nフィルタ", "Low-pass\nfilter"), "lowpass_cutoff_hz", L("速い揺れを\nなます", "smooths fast\nshaking"), "time"),
]
ROW2 = [  # every sample
    (L("サンプリング", "Sampling"), "update_rate", L("周期ごとに\n測って保持", "sample and\nhold each\nperiod"), "time"),
    (L("非線形", "Non-\nlinearity"), "nonlinearity", L("まっすぐ\nでない", "not a\nstraight line"), "unit"),
    (L("校正行列", "Calibration\nmatrix"), "scale_error\ncrosstalk", L("倍率のずれ・\n軸の混ざり", "scale error,\ncrosstalk"), "unit"),
    (L("バイアス", "Bias"), "bias", L("個体ごとの\n一定のずれ", "constant\noffset per\nunit"), "unit"),
    (L("ドリフト", "Drift"), "drift_per_hour", L("ゆっくり\nずれていく", "slowly\nwanders"), "wander"),
    (L("ノイズ", "Noise"), "noise", L("測るたびに\n散らばる", "scatters\neach sample"), "wander"),
    (L("飽和", "Saturation"), "range", L("測れる範囲\nで切れる", "clipped to\nthe range"), "limit"),
    (L("風袋引き", "Tare"), "tare", L("今の値を\nゼロにする", "zero the\ncurrent value"), "user"),
    (L("分解能", "Resolution"), "resolution", L("値の刻み", "value step"), "limit"),
]

fig, ax = plt.subplots(figsize=(16, 6.6))
ax.set_xlim(-1.6, 9.6)
ax.set_ylim(-2.75, 3.3)
ax.axis("off")


def box(x, y, name, key, note, group):
    color = GROUPS[group][0]
    ax.add_patch(FancyBboxPatch((x - 0.43, y - 0.38), 0.86, 0.76, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=color, ec="none", alpha=0.92))
    ax.text(x, y, name, ha="center", va="center", color="white", fontsize=10.5, fontweight="bold")
    ax.text(x, y + 0.45, key, ha="center", va="bottom", fontsize=7.3, color=MUTED, family="monospace")
    if note:
        ax.text(x, y - 0.47, note, ha="center", va="top", fontsize=8, color=INK)


def arrow(a, b, **kw):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, color=INK, **kw))


# row 1: every physics step
y1 = 2.0
ax.text(-1.55, y1 + 0.95, L("物理の刻みごと（毎ステップ）", "Every physics step"), fontsize=11, color=INK, fontweight="bold")
ax.text(-1.15, y1, L("真値", "Truth"), fontsize=14, fontweight="bold", color=INK, ha="center", va="center")
ax.text(-1.15, y1 - 0.42, L("工具側が\nロボット側に\nかける力", "force of the\ntool side on\nthe robot side"), fontsize=8, color=MUTED, ha="center", va="top")
for i, (name, key, note, group) in enumerate(ROW1):
    box(i * 1.1, y1, name, key, note, group)
    if i:
        arrow(((i - 1) * 1.1 + 0.45, y1), (i * 1.1 - 0.45, y1))
arrow((-0.75, y1), (-0.45, y1))
# the overload watch looks at the truth; the signal itself goes on
ax.text(2.75, y1 + 0.05, L("→ 周期の瞬間だけ下の段へ", "→ only at each sample time, to the row below"), fontsize=9.5, color=INK, va="center")

# row 2: every sample
y2 = -0.9
ax.text(-1.55, y2 + 1.0, L("測る瞬間ごと（周期ごと）", "Every sample period"), fontsize=11, color=INK, fontweight="bold")
for i, (name, key, note, group) in enumerate(ROW2):
    box(i * 1.0, y2, name, key, note, group)
    if i:
        arrow(((i - 1) * 1.0 + 0.45, y2), (i * 1.0 - 0.45, y2))
arrow((8.45, y2), (8.8, y2))
ax.text(9.25, y2, L("測った値", "Measured"), fontsize=13, fontweight="bold", color=INK, ha="center", va="center")
ax.annotate("", xy=(0.0, y2 + 0.85), xytext=(2.2, y1 - 0.75),
            arrowprops=dict(arrowstyle="-|>", color=INK, connectionstyle="arc3,rad=0.25"))
# the normalised part: nonlinearity and calibration work on the value divided by the range
ax.plot([0.6, 2.4], [y2 - 1.3, y2 - 1.3], color=GROUPS["unit"][0], lw=2)
ax.text(1.5, y2 - 1.38, L("ここは「レンジに対する割合」で計算", "computed as a fraction of the range"), fontsize=8.5, color=GROUPS["unit"][0], ha="center", va="top")

# legend
for k, (group, (color, label)) in enumerate(GROUPS.items()):
    x = 4.2 + (k % 2) * 2.7 if k < 4 else 4.2
    yy = 3.05 - (k // 2) * 0.33
    ax.add_patch(FancyBboxPatch((x, yy - 0.09), 0.22, 0.18, boxstyle="round,pad=0.01", fc=color, ec="none"))
    ax.text(x + 0.3, yy, label, fontsize=8.8, color=INK, va="center")

ax.text(-1.55, -2.7, L("ドローンのセンサーとの違い：取り付けの段がない（ホストがセンサーの軸で真値を渡す）。"
                       "フィルタ・過負荷・風袋引きがある。書かなかった段は何もしない。",
                       "Unlike the drone sensors: no mounting stage (the host gives the truth in sensor axes). "
                       "Filter, overload and tare stages are added. Stages you do not set do nothing."),
        fontsize=9, color=MUTED)
fig.tight_layout()
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
