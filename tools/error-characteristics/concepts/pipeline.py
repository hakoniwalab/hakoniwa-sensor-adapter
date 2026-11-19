"""Concept figure 1: the common processing order of the five drone sensors (draft).

    python3 pipeline.py <out.png> [ja|en]
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en


STAGES = [
    (L("取り付け", "Mounting"), L("位置・向き", "position, orientation"), "mount"),
    (L("感度", "Sensitivity"), L("スケール・軸の混ざり\n（地磁気は鉄の影響）", "scale, cross-axis\n(mag: iron effects)"),
     "sensitivity\nsoftIron / hardIron"),
    (L("遅れ", "Delay"), "", "delayMsec"),
    (L("サンプリング", "Sampling"), L("周期ごとに測って保持", "sample each period,\nhold"), "updatePeriodMsec"),
    (L("零点のずれ", "Zero offset"), L("個体ごとに一定", "fixed per unit"), "zero…Offset"),
    (L("ノイズ", "Noise"), L("測るたびに散らばる", "scatters at\neach sample"), "noise\nnoiseDensity"),
    (L("ドリフト", "Drift"), L("ゆっくりさまよう", "wanders slowly"), "drift"),
    (L("分解能", "Resolution"), L("値の刻み", "value steps"), "resolution…"),
    (L("レンジ", "Range"), L("上限で切れる", "clipped at limits"), "range"),
]
# Which sensor has which stage (from docs/spec/<sensor>.md, section "Processing order")
SENSORS = [
    (L("ジャイロ", "Gyro"), L("向き", "orientation"), [1, 1, 1, 1, 1, 1, 1, 1, 1]),
    (L("加速度", "Accel"), L("位置・向き", "position + orientation"), [1, 1, 1, 1, 1, 1, 1, 1, 1]),
    (L("地磁気", "Mag"), L("向き", "orientation"), [1, 1, 1, 1, 0, 1, 1, 1, 1]),
    (L("気圧", "Baro"), L("高さ", "height"), [1, 0, 1, 1, 1, 1, 1, 1, 1]),
    ("GPS", L("アンテナの位置", "antenna position"), [1, 0, 1, 1, 0, 1, 1, 0, 0]),
]
HOST = "#2f6fd6"
IMPL = "#d1242f"
INK = "#14213d"
MUTED = "#8a96a8"

fig, ax = plt.subplots(figsize=(15, 7.2))
ax.set_xlim(-2.3, len(STAGES) + 0.6)
ax.set_ylim(-len(SENSORS) - 0.9, 3.4)
ax.axis("off")

# truth and output
ax.text(-1.55, 1.55, L("真値", "Truth"), ha="center", va="center", fontsize=15, fontweight="bold", color=INK)
ax.text(-1.55, 1.05, L("物理エンジンが\n毎ステップ渡す", "from the physics\nengine every step"), ha="center", va="center", fontsize=8.5, color=MUTED)
ax.text(len(STAGES) + 0.15, 1.55, L("測った値", "Measured"), ha="center", va="center", fontsize=15, fontweight="bold", color=INK)

for i, (name, note, key) in enumerate(STAGES):
    color = HOST if i == 0 else IMPL
    box = FancyBboxPatch((i - 0.42, 1.1), 0.84, 0.9, boxstyle="round,pad=0.02,rounding_size=0.08",
                         fc=color, ec="none", alpha=0.92)
    ax.add_patch(box)
    ax.text(i, 1.55, name, ha="center", va="center", color="white", fontsize=12 if LANG == "ja" else 10.5,
            fontweight="bold")
    ax.text(i, 0.72, note, ha="center", va="top", fontsize=8, color=INK)
    ax.text(i, 2.12, key, ha="center", va="bottom", fontsize=7.5, color=MUTED, family="monospace")
    if i + 1 < len(STAGES):
        ax.add_patch(FancyArrowPatch((i + 0.43, 1.55), (i + 0.57, 1.55), arrowstyle="-|>", mutation_scale=12,
                                     color=INK))
ax.add_patch(FancyArrowPatch((-1.1, 1.55), (-0.45, 1.55), arrowstyle="-|>", mutation_scale=14, color=INK))
ax.add_patch(FancyArrowPatch((len(STAGES) - 0.55, 1.55), (len(STAGES) - 0.35, 1.55), arrowstyle="-|>",
                             mutation_scale=14, color=INK))

# who does what
ax.annotate("", xy=(-0.42, 2.95), xytext=(0.42, 2.95), arrowprops=dict(arrowstyle="-", color=HOST, lw=3))
ax.text(0, 3.1, L("ホスト（シミュレータ側の共通部品）", "Host (the simulator's common parts)"), ha="center", va="bottom", fontsize=9, color=HOST)
ax.annotate("", xy=(0.58, 2.95), xytext=(len(STAGES) - 0.58, 2.95), arrowprops=dict(arrowstyle="-", color=IMPL, lw=3))
ax.text((len(STAGES) - 1) / 2 + 0.5, 3.1, L("センサーの実装（センサーPRO）：共通の順番で、データシートの値を1段ずつ当てはめる",
          "Sensor implementation (Sensor PRO): applies the datasheet values one stage at a time, in the common order"),
        ha="center", va="bottom", fontsize=9, color=IMPL)

# sensors x stages
for row, (sensor, mount, has) in enumerate(SENSORS):
    y = -row - 0.5
    ax.text(-1.55, y, sensor, ha="center", va="center", fontsize=12, fontweight="bold", color=INK)
    for i, flag in enumerate(has):
        if flag:
            color = HOST if i == 0 else IMPL
            ax.scatter([i], [y], s=260, color=color, alpha=0.9, zorder=3)
            if i == 0:
                ax.text(i, y - 0.3, mount, ha="center", va="top", fontsize=7.5, color=HOST)
        else:
            ax.text(i, y, "—", ha="center", va="center", fontsize=12, color=MUTED)
    ax.plot([-2.1, len(STAGES) - 0.4], [y - 0.5, y - 0.5], color="#e4e9f1", lw=0.8, zorder=0)
ax.text(-2.1, -len(SENSORS) - 0.55, L("● その段を持つ　— 持たない。IMU（ジャイロ・加速度）は、感度と遅れの間に任意のフィルタも入れられる。",
          "● has the stage   — does not. The IMU (gyro, accel) can also take an optional filter between Sensitivity and Delay."), ha="left", va="center", fontsize=8.5, color=MUTED)

fig.tight_layout()
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
