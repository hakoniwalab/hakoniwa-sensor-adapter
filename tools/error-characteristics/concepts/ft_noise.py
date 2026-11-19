"""Force-torque concept figure 5: noise, bandwidth and the low-pass filter (draft, Japanese/English).
Illustrative data; the rules are docs/spec/force-torque.md FTS-ADP-130 and 310 to 340."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, MEAS, BLUE, GREEN, BAND = "#14213d", "#8a96a8", "0.55", "#d1242f", "#2f6fd6", "#1a7f37", "#fbe3e5"
rng = np.random.default_rng(4)
fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))


def butterworth(x, fc, h):
    """Second-order Butterworth low-pass, bilinear transform with pre-warping (FTS-ADP-130), steady at x[0]."""
    K = np.tan(np.pi * fc * h)
    q = 1 / np.sqrt(2)
    norm = 1 / (1 + K / q + K * K)
    b0 = K * K * norm
    b1, b2 = 2 * b0, b0
    a1 = 2 * (K * K - 1) * norm
    a2 = (1 - K / q + K * K) * norm
    y = np.empty_like(x)
    x1 = x2 = y1 = y2 = x[0]
    for i, xi in enumerate(x):
        yi = b0 * xi + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1, y2, y1 = x1, xi, y1, yi
        y[i] = yi
    return y


# 1. how datasheets write noise: 1 sigma or 6 sigma peak to peak
ax = axes[0]
t = np.arange(0, 0.5, 0.002)
sigma = 0.15
y = rng.normal(0, sigma, t.size)
ax.axhspan(-sigma, sigma, color=BAND, zorder=0)
ax.axhline(3 * sigma, color=MUTED, lw=1, ls="--")
ax.axhline(-3 * sigma, color=MUTED, lw=1, ls="--")
ax.plot(t * 1000, y, color=MEAS, lw=0.8)
ax.text(505, sigma / 2, "±1σ\n= 0.15 N", fontsize=9, color=MEAS, va="center")
ax.annotate("", xy=(470, 3 * sigma), xytext=(470, -3 * sigma), arrowprops=dict(arrowstyle="<->", color=INK))
ax.text(505, -0.25, L("ピーク・ツー・\nピーク ≈ 6σ\n= 0.9 N", "peak to\npeak ≈ 6σ\n= 0.9 N"), fontsize=9, color=INK, va="center")
ax.set_xlim(0, 640)
ax.set_ylim(-0.7, 0.8)
ax.set_xlabel(L("時間 [ms]", "Time [ms]"))
ax.set_ylabel(L("Fz − 真値 [N]", "Fz − truth [N]"))
ax.set_title(L("① データシートの書き方：1σ か、山から谷か", "① Datasheets: 1σ or peak to peak"), fontsize=12, color=INK, loc="left")
ax.text(10, 0.68, L("\"definition\": \"1sigma\" はそのまま、\"6sigma_pp\" は 6 で割って σ にする",
          "\"definition\": \"1sigma\" is used as is, \"6sigma_pp\" is divided by 6"), fontsize=8.5,
        color=INK)
ax.grid(alpha=0.3)

# 2. the same sensor, a narrower bandwidth: sigma = sigma0 * sqrt(B / B0)
ax = axes[1]
B = np.linspace(10, 500, 200)
sigma0, B0 = 0.15, 500
ax.plot(B, sigma0 * np.sqrt(B / B0), color=INK, lw=1.5, label="σ = σ₀ × √(B ÷ B₀)")
for b, color, label in ((500, MEAS, L("データシートの条件\nB₀ = 500 Hz：0.15 N", "datasheet condition\nB₀ = 500 Hz: 0.15 N")), (100, BLUE, L("フィルタ 100 Hz：\n0.067 N", "filter 100 Hz:\n0.067 N")),
                        (20, GREEN, L("フィルタ 20 Hz：\n0.030 N", "filter 20 Hz:\n0.030 N"))):
    s = sigma0 * np.sqrt(b / B0)
    ax.plot(b, s, "o", color=color, ms=10)
    ax.text(b + 12, s - 0.012, label, fontsize=9, color=color, va="top")
ax.set_xlim(0, 620)
ax.set_ylim(0, 0.2)
ax.set_xlabel(L("帯域 B [Hz]（どこまで速い揺れを通すか）", "Bandwidth B [Hz] (how fast a signal passes)"))
ax.set_ylabel(L("ノイズの σ [N]", "Noise σ [N]"))
ax.set_title(L("② 帯域を狭めると、ノイズが減る", "② Narrower bandwidth, less noise"), fontsize=12, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
ax.grid(alpha=0.3)

# 3. the price: a slower response
ax = axes[2]
h = 0.001
t = np.arange(0, 0.08, h)
truth = np.where(t >= 0.01, 10.0, 0.0)                      # contact: 0 -> 10 N at 10 ms
# as the adapter does it: the filter acts on the truth; the noise is added afterwards with the sigma of the band
base = rng.normal(0, 1, t.size)
ax.step(t * 1000, truth, where="post", color=TRUTH, lw=2.5, label=L("真値（10 ms で 10 N に触れる）", "truth (10 N contact at 10 ms)"))
ax.plot(t * 1000, truth + 0.15 * base, color=MEAS, lw=0.8, alpha=0.7, label=L("フィルタなし（σ 0.15 N）", "no filter (σ 0.15 N)"))
ax.plot(t * 1000, butterworth(truth, 100, h) + 0.067 * base, color=BLUE, lw=2, label="100 Hz（σ 0.067 N）")
ax.plot(t * 1000, butterworth(truth, 20, h) + 0.030 * base, color=GREEN, lw=2, label=L("20 Hz（σ 0.030 N、遅い）", "20 Hz (σ 0.030 N, slow)"))
ax.set_xlabel(L("時間 [ms]", "Time [ms]"))
ax.set_ylabel("Fz [N]")
ax.set_title(L("③ 代わりに、反応が遅れる", "③ The price: a slower response"), fontsize=12, color=INK, loc="left")
ax.legend(loc="lower right", fontsize=8.3)
ax.grid(alpha=0.3)

fig.text(0.04, 0.02,
         L("データシートのノイズは「どの帯域で測ったか」とセットで書かれている（\"rate_hz\" の半分、または \"bandwidth_hz\"）。"
           "箱庭は、使う側の帯域（フィルタの \"lowpass_cutoff_hz\"、なければ周期の半分）に σ を換算する。\n"
           "フィルタ（2 次のバターワース、物理の刻みごと）は真値をなまし、ノイズはその後、帯域に合わせた σ で足す。ノイズを減らすほど、触れた瞬間の検出は遅れる。",
           "Datasheet noise comes with the bandwidth it was measured in (half of \"rate_hz\", or \"bandwidth_hz\"). "
           "Hakoniwa converts σ to the bandwidth in use (the filter's \"lowpass_cutoff_hz\", else half the rate).\n"
           "The filter (2nd-order Butterworth, every physics step) smooths the truth; noise is added after it, with the σ of that band. "
           "Less noise means a later contact detection."),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.2, wspace=0.27)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
