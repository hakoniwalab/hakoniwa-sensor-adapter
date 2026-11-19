"""Force-torque concept figure 3: tare (draft, Japanese/English). Illustrative data made with numpy; the rules are
docs/spec/force-torque.md 2.5 (FTS-ADP-410 to 440)."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, TRUTH, RAW, OUT, TARE, DRIFT = "#14213d", "#8a96a8", "0.55", "#b26a00", "#d1242f", "#1a7f37", "#2f6fd6"
rng = np.random.default_rng(7)
fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))
t = np.arange(0, 10, 0.002)
G = 9.8  # a 1 kg gripper

# 1. why: the tool weight is in the value
ax = axes[0]
contact = np.where((t > 6) & (t < 8), 5.0, 0.0)               # pressing on something: +5 N
raw = -G + contact + rng.normal(0, 0.08, t.size)
tare_at = 3.0
out = np.where(t < tare_at, np.nan, raw + G)  # tare starts here
ax.plot(t, raw, color=RAW, lw=0.8, label=L("センサーの値（風袋引きなし）", "sensor value (no tare)"))
ax.plot(t, out, color=OUT, lw=1.2, label=L("風袋引き後の値", "value after tare"))
ax.axvline(tare_at, color=TARE, lw=1.2, ls="--")
ax.axvspan(tare_at - 0.1, tare_at, color=TARE, alpha=0.25)
ax.text(tare_at + 0.1, -12.5, L("ここで tare()：\n直前 0.1 秒の平均を\n「ゼロ」にする", "tare() here:\nthe mean of the last\n0.1 s becomes zero"), fontsize=9, color=TARE)
ax.text(6.0, 5.8, L("触れた力 +5 N が\nそのまま見える", "the +5 N contact\nshows as is"), fontsize=9, color=OUT)
ax.text(0.2, -8.5, L("グリッパの重さ\n−9.8 N", "gripper weight\n−9.8 N"), fontsize=9, color=RAW)
ax.set_ylim(-14, 9)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel("Fz [N]")
ax.set_title(L("① なぜ要るか：何も触れていなくても重さが出る", "① Why: the weight shows even with no contact"), fontsize=12, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
ax.grid(alpha=0.3)

# 2. caveat: the weight moves to other axes when the wrist turns
ax = axes[1]
angle = np.clip((t - 5) / 2, 0, 1) * np.pi / 2               # the wrist tilts 90 degrees from 5 s to 7 s
fz_true = -G * np.cos(angle)
fx_true = G * np.sin(angle)
ax.plot(t, fz_true + G, color=OUT, lw=2, label=L("Fz（風袋引き後）", "Fz (after tare)"))
ax.plot(t, fx_true, color=DRIFT, lw=2, label=L("Fx（風袋引き後）", "Fx (after tare)"))
ax.axvline(2, color=TARE, lw=1.2, ls="--")
ax.text(2.1, -11.5, L("下向きの姿勢で\ntare()", "tare() while\npointing down"), fontsize=9, color=TARE)
ax.axvspan(5, 7, color="#eef1f6", zorder=0)
ax.text(6, 11.3, L("手首を 90° 傾ける", "wrist tilts 90°"), fontsize=9, color=INK, ha="center")
ax.text(7.2, 5.2, L("何も触れて\nいないのに\n+9.8 N が\n2 つの軸に出る", "no contact, yet\n+9.8 N on two axes"), fontsize=9, color=OUT)
ax.set_ylim(-13, 13)
ax.set_xlabel(L("時間 [s]", "Time [s]"))
ax.set_ylabel(L("力 [N]", "Force [N]"))
ax.set_title(L("② 注意：姿勢が変わると、引いた重さがずれる", "② Caveat: a new pose breaks the tare"), fontsize=12, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
ax.grid(alpha=0.3)

# 3. caveat: tare does not stop the drift
ax = axes[2]
T = np.arange(0, 3600, 1.0)
drift = np.cumsum(np.random.default_rng(3).normal(0, 0.5 * np.sqrt(1 / 3600), T.size))     # random walk, 0.5 N per sqrt(hour)
tares = [0, 1800]
base = np.zeros(T.size)
for k, start in enumerate(tares):
    base[T >= start] = drift[int(start)]
ax.plot(T / 60, drift, color=DRIFT, lw=1.5, label=L("ドリフト（風袋引きしても進む）", "drift (goes on after tare)"))
ax.plot(T / 60, drift - base, color=OUT, lw=1.5, label=L("風袋引き後の値（触れていない）", "value after tare (no contact)"))
for start in tares[1:]:
    ax.axvline(start / 60, color=TARE, lw=1.2, ls="--")
ax.text(31, 0.17, L("30 分後に tare() し直すと\nまたゼロから", "tare() again at 30 min:\nback to zero"), fontsize=9, color=TARE)
ax.set_xlabel(L("時間 [分]", "Time [min]"))
ax.set_ylabel("Fz [N]")
ax.set_title(L("③ 注意：ドリフトは止まらない", "③ Caveat: tare does not stop drift"), fontsize=12, color=INK, loc="left")
ax.legend(loc="upper left", fontsize=8.5)
ax.grid(alpha=0.3)

fig.text(0.04, 0.02,
         L("風袋引き（tare）は、使う人が呼ぶ操作。直前の \"tare_window\"（既定 0.1 秒）の値の平均を覚え、以後その分を引く。"
           "引くのは一定の値だけなので、姿勢が変わったときやドリフトには、そのたびにやり直す。\n"
           "実機でも同じで、つかむ前・作業の前に毎回ゼロ合わせをするのが普通。clear_tare() で元に戻す。",
           "Tare is a user action. It stores the mean over the last \"tare_window\" (default 0.1 s) and subtracts it from then on. "
           "It subtracts a constant only, so redo it after a pose change or drift.\n"
           "Real sensors are the same: zero them before each grasp or task. clear_tare() undoes it."),
         fontsize=9, color=INK)
fig.subplots_adjust(left=0.05, right=0.98, top=0.9, bottom=0.2, wspace=0.25)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
