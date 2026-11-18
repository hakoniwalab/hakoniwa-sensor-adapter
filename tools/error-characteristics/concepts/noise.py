"""Concept figure 3: noise and noise density (draft, Japanese). Illustrative data made with numpy."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
INK, MUTED, TRUTH, MEAS, BAND = "#14213d", "#8a96a8", "0.45", "#d1242f", "#fbe3e5"
rng = np.random.default_rng(3)

fig = plt.figure(figsize=(16, 6.4))
grid = fig.add_gridspec(2, 3, height_ratios=[1, 1], width_ratios=[1.15, 1.15, 0.9], hspace=0.55, wspace=0.28)

# 1. what noise is
ax = fig.add_subplot(grid[:, 0])
t = np.arange(0, 0.2, 0.004)
sigma = 1.0
y = rng.normal(0, sigma, t.size)
ax.axhspan(-sigma, sigma, color=BAND, zorder=0)
ax.axhspan(-2 * sigma, -sigma, color="#fdf1f2", zorder=0)
ax.axhspan(sigma, 2 * sigma, color="#fdf1f2", zorder=0)
ax.axhline(0, color=TRUTH, lw=2, label="真値（ここでは 0）")
ax.plot(t * 1000, y, "o", color=MEAS, ms=5, label="測った値（4 ms ごと）")
ax.text(201, sigma / 2, "±σ\n約 68 %", fontsize=9, color=MEAS, va="center")
ax.text(201, 1.5 * sigma, "±2σ\n約 95 %", fontsize=8.5, color=MUTED, va="center")
ax.set_xlim(-3, 225)
ax.set_ylim(-3.2, 3.6)
ax.set_xlabel("時間 [ms]")
ax.set_ylabel("値（σ を 1 とした目盛り）")
ax.set_title("① ノイズ：測るたびに、真値の周りに散らばる", fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
fig.text(0.05, 0.035, "前の値とは関係なく、毎回新しく散らばる（白色ノイズ）\n設定：\"noise\": σ　（軸ごとに [x, y, z] でも書ける）",
         fontsize=8.8, color=INK)

# 2. noise density: the same sensor at two periods
density = 1.0
for row, period_ms in enumerate((2, 8)):
    ax = fig.add_subplot(grid[row, 1])
    dt = period_ms / 1000
    s = density / np.sqrt(dt) / (density / np.sqrt(0.002))  # sigma, scaled so that the 2 ms case is 1
    t = np.arange(0, 0.2, dt)
    ax.axhspan(-s, s, color=BAND, zorder=0)
    ax.axhline(0, color=TRUTH, lw=2)
    ax.plot(t * 1000, rng.normal(0, s, t.size), "o" if period_ms > 2 else ".", color=MEAS, ms=4 if period_ms > 2 else 5)
    ax.set_ylim(-3.2, 3.2)
    ax.set_xlim(-3, 203)
    ax.set_yticks([-2, 0, 2])
    ax.text(205, 0, f"周期 {period_ms} ms\nσ = {s:.2f}", fontsize=9.5, color=MEAS, va="center")
    if row == 0:
        ax.set_title("② ノイズ密度：同じセンサーでも、周期でばらつきが変わる", fontsize=12.5, color=INK, loc="left")
    else:
        ax.set_xlabel("時間 [ms]")
fig.text(0.375, 0.035,
         "データシートは「ノイズ密度」（例：0.0028 °/s/√Hz）で書くことが多い。その値をそのまま書く：\"noiseDensity\"\n"
         "σ ＝ ノイズ密度 ÷ √周期。速く測る（周期が短い）ほど1回のばらつきは大きいが、回数が多い",
         fontsize=8.8, color=INK, ha="left")

# 3. the same stage in every sensor, its own unit
ax = fig.add_subplot(grid[:, 2])
ax.axis("off")
ax.set_title("③ 5 センサーとも同じ段、単位だけが違う", fontsize=12.5, color=INK, loc="left")
rows = [("ジャイロ", "rad/s", "noise / noiseDensity"),
        ("加速度", "m/s²", "noise / noiseDensity"),
        ("地磁気", "nT", "noise / noiseDensity"),
        ("気圧", "m（高度）", "noise / noiseDensity"),
        ("GPS", "m（位置）\nm/s（速度）", "positionNoise\nvelocityNoise")]
table = ax.table(cellText=[[a, b, c] for a, b, c in rows], colLabels=["センサー", "σ の単位", "設定のキー"],
                 loc="upper center", cellLoc="left", colWidths=[0.24, 0.3, 0.46], bbox=[0, 0.25, 1.0, 0.68])
table.auto_set_font_size(False)
table.set_fontsize(9)
for (r, c), cell in table.get_celld().items():
    cell.set_edgecolor("#d6deea")
    if r == 0:
        cell.set_facecolor("#eaf0f9")
        cell.set_text_props(fontweight="bold", color=INK)
ax.text(0, 0.12, "GPS は水平と鉛直を別々に書く。\nノイズの段は「サンプリング」の後：\n測った瞬間ごとに1つ、新しい値が足される", fontsize=9, color=INK)

fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.17)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
