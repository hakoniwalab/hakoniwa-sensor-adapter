"""Concept figure 1: the common processing order of the five drone sensors (draft, Japanese)."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]

STAGES = [
    ("取り付け", "位置・向き", "mount"),
    ("感度", "スケール・軸の混ざり\n（地磁気は鉄の影響）", "sensitivity\nsoftIron / hardIron"),
    ("遅れ", "", "delayMsec"),
    ("サンプリング", "周期ごとに測って保持", "updatePeriodMsec"),
    ("零点のずれ", "個体ごとに一定", "zero…Offset"),
    ("ノイズ", "測るたびに散らばる", "noise\nnoiseDensity"),
    ("ドリフト", "ゆっくりさまよう", "drift"),
    ("分解能", "値の刻み", "resolution…"),
    ("レンジ", "上限で切れる", "range"),
]
# Which sensor has which stage (from docs/spec/<sensor>.md, section "Processing order")
SENSORS = [
    ("ジャイロ", "向き", [1, 1, 1, 1, 1, 1, 1, 1, 1]),
    ("加速度", "位置・向き", [1, 1, 1, 1, 1, 1, 1, 1, 1]),
    ("地磁気", "向き", [1, 1, 1, 1, 0, 1, 1, 1, 1]),
    ("気圧", "高さ", [1, 0, 1, 1, 1, 1, 1, 1, 1]),
    ("GPS", "アンテナの位置", [1, 0, 1, 1, 0, 1, 1, 0, 0]),
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
ax.text(-1.55, 1.55, "真値", ha="center", va="center", fontsize=15, fontweight="bold", color=INK)
ax.text(-1.55, 1.05, "物理エンジンが\n毎ステップ渡す", ha="center", va="center", fontsize=8.5, color=MUTED)
ax.text(len(STAGES) + 0.15, 1.55, "測った値", ha="center", va="center", fontsize=15, fontweight="bold", color=INK)

for i, (name, note, key) in enumerate(STAGES):
    color = HOST if i == 0 else IMPL
    box = FancyBboxPatch((i - 0.42, 1.1), 0.84, 0.9, boxstyle="round,pad=0.02,rounding_size=0.08",
                         fc=color, ec="none", alpha=0.92)
    ax.add_patch(box)
    ax.text(i, 1.55, name, ha="center", va="center", color="white", fontsize=12, fontweight="bold")
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
ax.text(0, 3.1, "ホスト（シミュレータ側の共通部品）", ha="center", va="bottom", fontsize=9, color=HOST)
ax.annotate("", xy=(0.58, 2.95), xytext=(len(STAGES) - 0.58, 2.95), arrowprops=dict(arrowstyle="-", color=IMPL, lw=3))
ax.text((len(STAGES) - 1) / 2 + 0.5, 3.1, "センサーの実装（センサーPRO）：共通の順番で、データシートの値を1段ずつ当てはめる",
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
ax.text(-2.1, -len(SENSORS) - 0.55, "● その段を持つ　— 持たない。IMU（ジャイロ・加速度）は、感度と遅れの間に任意のフィルタも入れられる。", ha="left", va="center", fontsize=8.5, color=MUTED)

fig.tight_layout()
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
