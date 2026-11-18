"""Concept figure 5: drift (Gauss-Markov), draft in Japanese. Illustrative data made with numpy, using the
formula of the common drone sensor specification 4.2."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
INK, MUTED, TRUTH, MEAS, BAND = "#14213d", "#8a96a8", "0.45", "#d1242f", "#fbe3e5"
BLUE, GREEN, ORANGE = "#2f6fd6", "#1a7f37", "#b26a00"
rng = np.random.default_rng(11)


def gauss_markov(n, dt, sigma, tau, rng):
    phi = np.exp(-dt / tau)
    sd = sigma * np.sqrt(1 - np.exp(-2 * dt / tau))
    b = np.empty(n)
    b[0] = rng.normal(0, sigma)
    for k in range(1, n):
        b[k] = phi * b[k - 1] + rng.normal(0, sd)
    return b


fig, axes = plt.subplots(1, 3, figsize=(16, 6.0))

# 1. offset vs drift
ax = axes[0]
dt = 0.5
t = np.arange(0, 600, dt)
offset = 0.8
drift = gauss_markov(t.size, dt, 0.5, 100, np.random.default_rng(4))
ax.axhline(0, color=TRUTH, lw=2.5, label="真値（静止：0）")
ax.plot(t, np.full(t.size, offset), color=BLUE, lw=2, label="零点のずれ（④）：ずっと一定")
ax.plot(t, offset + drift, color=MEAS, lw=1.8, label="＋ドリフト：ゆっくりさまよう")
ax.set_xlim(0, 600)
ax.set_ylim(-1.2, 2.6)
ax.set_xlabel("時間 [s]")
ax.set_ylabel("測った値（ノイズは省略）")
ax.set_title("① ずれは一定、ドリフトはゆっくり動く", fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)

# 2. the two settings: sigma (how far) and tau (how fast); compare with a random walk
ax = axes[1]
dt = 2.0
t = np.arange(0, 3000, dt)
sigma = 1.0
ax.axhspan(-sigma, sigma, color=BAND, zorder=0)
ax.axhspan(-2 * sigma, -sigma, color="#fdf1f2", zorder=0)
ax.axhspan(sigma, 2 * sigma, color="#fdf1f2", zorder=0)
ax.axhline(0, color=TRUTH, lw=1.5)
ax.plot(t, gauss_markov(t.size, dt, sigma, 30, np.random.default_rng(3)), color=MEAS, lw=0.7, alpha=0.8,
        label="τ = 30 s：細かく行き来する")
ax.plot(t, gauss_markov(t.size, dt, sigma, 600, np.random.default_rng(36)), color=BLUE, lw=2.4,
        label="τ = 600 s：ゆっくりうねる")
ax.text(3040, sigma / 2, "±σ\n約 68 %", fontsize=9, color=MEAS, va="center")
ax.text(3040, 1.5 * sigma, "±2σ\n約 95 %", fontsize=8.5, color=MUTED, va="center")
ax.text(1500, -3.5, "σ が同じなら、どちらも同じ帯の中をさまよう\n（時間がたっても広がり続けない）", fontsize=8.8,
        color=INK, ha="center")
ax.set_xlim(0, 3400)
ax.set_ylim(-4.0, 3.9)
ax.set_xlabel("時間 [s]")
ax.set_ylabel("ドリフト（σ を 1 とした目盛り）")
ax.set_title("② σ はどこまで、τ はどれだけ速く", fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.3)

# 3. averaging removes noise, not drift
ax = axes[2]
dt = 0.02
t = np.arange(0, 120, dt)
drift = gauss_markov(t.size, dt, 0.6, 40, np.random.default_rng(9))
raw = drift + rng.normal(0, 1.0, t.size)
win = int(2.0 / dt)
avg = np.convolve(raw, np.ones(win) / win, mode="same")
ax.plot(t, raw, color="#f2b8bd", lw=0.4, label="測った値（ノイズ＋ドリフト）")
ax.plot(t[win:-win], avg[win:-win], color=MEAS, lw=2, label="2 秒平均：ノイズは消える")
ax.plot(t, drift, color=INK, lw=1.2, ls="--", label="ドリフトそのもの：平均しても残る")
ax.axhline(0, color=TRUTH, lw=1.5)
ax.set_xlim(0, 120)
ax.set_ylim(-4, 5)
ax.set_xlabel("時間 [s]")
ax.set_ylabel("値")
ax.set_title("③ 平均でノイズは消えるが、ドリフトは残る", fontsize=12.5, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.3)

fig.text(0.04, 0.03,
         "ドリフトは「1 次ガウス・マルコフ過程」：散らばりは σ の帯に留まり（広がり続けない）、τ（相関時間）ほどの時間で入れ替わる。"
         "設定は σ と τ の 2 つ（\"mode\": \"gauss-markov\", \"correlationTimeSec\": τ）。"
         "零点のずれとは別に足される。\n"
         "σ のキー：ジャイロ stddevDegPerSec　加速度 stddevMg　地磁気 stddevNT　気圧 stddevM　"
         "GPS horizontalM・verticalM　｜　データシートの「バイアス不安定性」がジャイロの σ の目安",
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.2, wspace=0.3)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
