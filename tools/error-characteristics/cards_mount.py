"""Measurement cards of the mounting section (Japanese): on the left, how the sensor was mounted and moved; on the
right, what was measured, with the axes and the expected values written on the plot.

    python3 cards_mount.py --tool <build>/error-characteristics --out <dir> [--implementation hakoniwa]

The data come from runs of the error-characteristics tool (ec_common.run), the same runs as imu_figures.py and
mag_baro_gps_figures.py. The tool's --mount is in body FLU axes: x forward, y left, z up.
"""
import argparse
from pathlib import Path

import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle

import ec_common
from ec_common import plt

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
INK, MUTED, TRUTH, BODY = "#14213d", "#8a96a8", "0.55", "#c9d6ea"
C0, C1, C2 = "#14213d", "#2f6fd6", "#d1242f"
EARTH_R = 6378137.0
LAT0, LON0 = 35.6812, 139.7671


# ------------------------------------------------------------------------------------------------ drawing helpers
def arrow(ax, a, b, color=INK, lw=1.6, style="-|>", scale=14, **kw):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=scale, color=color, lw=lw, **kw))


def airframe(ax, size=0.25, alpha=1.0):
    """A quadcopter seen from above, forward up."""
    for ang in (45, 135, 225, 315):
        a = np.radians(ang)
        tip = (size * np.sin(a), size * np.cos(a))
        ax.plot([0, tip[0]], [0, tip[1]], color=BODY, lw=7, solid_capstyle="round", alpha=alpha, zorder=1)
        ax.add_patch(plt.Circle(tip, size * 0.28, fc="none", ec=BODY, lw=2, alpha=alpha))
    ax.add_patch(Rectangle((-0.06, -0.06), 0.12, 0.12, fc=BODY, ec="none", alpha=alpha, zorder=1))


def spin_arrow(ax, radius, text, color=INK):
    ax.add_patch(Arc((0, 0), 2 * radius, 2 * radius, theta1=40, theta2=140, color=color, lw=1.8))
    a = np.radians(140)
    arrow(ax, (radius * np.cos(np.radians(132)), radius * np.sin(np.radians(132))),
          (radius * np.cos(a + 0.12), radius * np.sin(a + 0.12)), color=color, scale=16)
    ax.text(0, radius + 0.03, text, ha="center", va="bottom", fontsize=10, color=color)


def sketch_axes(ax, xlim=(-0.45, 0.45), ylim=(-0.42, 0.5)):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")


def card(title):
    fig, (left, right) = plt.subplots(1, 2, figsize=(12.5, 4.6), gridspec_kw={"width_ratios": [0.85, 1.15]})
    fig.suptitle(title, fontsize=13.5, color=INK, x=0.02, ha="left", fontweight="bold")
    return fig, left, right


def save(fig, args, name):
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    path = Path(args.out) / f"{name}.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"wrote {path}")


def run(args, sensor, name, config, motion, duration, **extra):
    return ec_common.run(args, sensor, name, config, motion, duration, **extra)


# ------------------------------------------------------------------------------------------------ the cards
def lever_arm(args):
    rate = np.pi
    fig, left, right = card("取り付け位置：旋回中の加速度計（てこの腕）")
    sketch_axes(left)
    left.set_title("測り方（上から見た図、前が上）", fontsize=11, color=INK, loc="left")
    airframe(left)
    spin_arrow(left, 0.4, "その場で 180°/s で回し続ける")
    for r, color in ((0.0, C0), (0.1, C1), (0.2, C2)):
        y = 1.4 * r  # drawn longer than to scale
        left.add_patch(Rectangle((-0.018, y - 0.018), 0.036, 0.036, fc=color, ec="none", zorder=6))
        left.text(0.04, y, f"IMU を中心から {r:.1f} m 前", fontsize=9, color=color, va="center", zorder=7,
                  bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.8))
        if r > 0:
            arrow(left, (-0.05, y), (-0.05, y - 0.07 * r / 0.1), color=color, lw=2, scale=12)
    left.plot(0, 0, marker="x", color=INK, ms=9, mew=2, zorder=7)
    left.text(-0.09, 0.2, "中心向きの\n加速度 ω²r\n（遠いほど大きい）", fontsize=8.5, color=C2, ha="right", va="center",
              bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.8))
    left.text(-0.43, -0.34, "長さは縮尺どおりではない", fontsize=8.5, color=MUTED)
    left.text(-0.43, -0.4, "それ以外の誤差は入れない（真値だけの設定）", fontsize=8.5, color=MUTED)

    for r, color in ((0.0, C0), (0.1, C1), (0.2, C2)):
        d = run(args, "imu", f"mount-r{r}", "truth.json", "turn", 1.0, rate=rate, mount=f"{r},0,0,0,0,0")
        right.plot(d["time"], d["ax"], color=color, lw=2.2)
        expected = -rate * rate * r + 0.0
        right.axhline(expected, color=color, lw=0.8, ls="--")
        right.text(1.02, expected, f"{r:.1f} m：{d['ax'].mean() + 0.0:+.3f}\n（計算 {expected:+.3f}）", fontsize=8.5,
                   color=color, va="center")
    right.set_xlim(0, 1.32)
    right.set_ylim(-2.4, 0.5)
    right.set_xlabel("時間 [s]")
    right.set_ylabel("加速度計 x（前向き）[m/s²]")
    right.set_title("結果：離れるほど、後ろ向き（中心向き）の加速度が出る", fontsize=11, color=INK, loc="left")
    right.text(0.02, 0.3, "マイナス＝後ろ向き。実線（測定）が破線（−ω²r）に重なれば正しい", fontsize=8.5, color=INK)
    right.grid(alpha=0.3)
    save(fig, args, "card-mount-lever-arm")


def gyro_orientation(args):
    fig, left, right = card("取り付けの向き：ヨー 90° 回して付けたジャイロ")
    sketch_axes(left, xlim=(-0.5, 0.5), ylim=(-0.45, 0.5))
    left.set_title("測り方（上から見た図、前が上）", fontsize=11, color=INK, loc="left")
    for cx, turned, color, label in ((-0.25, False, C1, "まっすぐ付けた IMU"), (0.25, True, C2, "ヨー 90° 回して付けた IMU")):
        left.add_patch(Rectangle((cx - 0.04, -0.04), 0.08, 0.08, fc=color, ec="none", alpha=0.85))
        if turned:  # FLU, yaw +90 deg: sensor x points to the body's left, sensor y to the back
            arrow(left, (cx, 0), (cx - 0.17, 0), color=color)
            arrow(left, (cx, 0), (cx, -0.17), color=color)
            left.text(cx - 0.19, 0.03, "センサー x", fontsize=8.5, color=color, va="bottom", ha="right")
            left.text(cx + 0.02, -0.19, "センサー y", fontsize=8.5, color=color, va="top")
        else:
            arrow(left, (cx, 0), (cx, 0.17), color=color)
            arrow(left, (cx, 0), (cx - 0.17, 0), color=color)
            left.text(cx, 0.19, "センサー x", fontsize=8.5, color=color, ha="center")
            left.text(cx - 0.19, 0, "センサー y", fontsize=8.5, color=color, va="center", ha="right")
        left.text(cx, 0.24 if turned else -0.12, label, fontsize=9, color=color, ha="center")
    left.text(0, 0.4, "機体の x（前）まわりに回す：1 秒から 1 rad/s", fontsize=9.5, color=INK, ha="center")
    left.text(-0.48, -0.3, "機体の軸：前が x、左が y（FLU）。左に 90° 回して付けると、\n"
              "センサーの x は機体の左、センサーの y は機体の後ろを向く。\n"
              "だから機体の前まわりの回転は、センサーの −y に出る",
              fontsize=8.5, color=MUTED, va="top")
    left.set_xlim(-0.5, 0.5)
    # FLU: yaw +90 turns sensor x to the body's left; draw it as computed below, the text says what the plot shows
    d0 = run(args, "imu", "mount-yaw0", "truth.json", "step", 2.0, mount="0,0,0,0,0,0")
    d90 = run(args, "imu", "mount-yaw90", "truth.json", "step", 2.0, mount="0,0,0,0,0,90")
    right.plot(d0["time"], d0["gx"], color=C1, lw=2.4, label="まっすぐ：センサー x")
    right.plot(d90["time"], d90["gy"], color=C2, lw=2.2, ls="--", label="90° 回した：センサー y")
    right.plot(d90["time"], d90["gx"], color=C2, lw=1.4, ls=":", label="90° 回した：センサー x")
    right.set_ylim(-1.4, 1.5)
    right.set_xlabel("時間 [s]")
    right.set_ylabel("ジャイロの角速度 [rad/s]")
    right.set_title("結果：同じ回転が、回して付けると別の軸に出る", fontsize=11, color=INK, loc="left")
    right.legend(loc="upper left", fontsize=8.5)
    right.text(1.05, -1.25, f"90° 回したセンサーでは y に {d90['gy'][-1]:+.2f}、x は {d90['gx'][-1]:+.2f}",
               fontsize=8.5, color=C2)
    right.grid(alpha=0.3)
    save(fig, args, "card-mount-gyro-orientation")


def mag_orientation(args):
    turn = 2 * np.pi / 4.0
    fig, left, right = card("取り付けの向き：ヨー 90° 回して付けた地磁気センサー")
    sketch_axes(left)
    left.set_title("測り方（上から見た図、前が上）", fontsize=11, color=INK, loc="left")
    airframe(left, alpha=0.7)
    spin_arrow(left, 0.4, "4 秒で一回転（90°/s）")
    left.add_patch(Rectangle((-0.12, -0.02), 0.04, 0.04, fc=C1, ec="none", zorder=6))
    arrow(left, (-0.1, 0), (-0.1, 0.13), color=C1, lw=1.4, scale=10)
    left.text(-0.13, -0.08, "まっすぐ", fontsize=9, color=C1, ha="center")
    left.add_patch(Rectangle((0.08, -0.02), 0.04, 0.04, fc=C2, ec="none", zorder=6))
    arrow(left, (0.1, 0), (-0.03, 0), color=C2, lw=1.4, scale=10)
    left.text(0.13, -0.08, "ヨー 90°", fontsize=9, color=C2, ha="center")
    left.text(-0.43, -0.25, "矢印：センサーの x 軸の向き", fontsize=8.5, color=MUTED)
    left.text(-0.43, -0.36, "地磁気は北を向く。どちらも同じ磁場を測る", fontsize=8.5, color=MUTED)
    a = run(args, "mag", "mount-yaw0", "truth.json", "turn", 4.0, rate=turn)
    b = run(args, "mag", "mount-yaw90", "truth.json", "turn", 4.0, rate=turn, mount="0,0,0,0,0,90")
    right.plot(a["time"], a["mx"] / 1000, color=C1, lw=2.2, label="まっすぐ：センサー x")
    right.plot(b["time"], b["mx"] / 1000, color=C2, lw=2.2, ls="--", label="ヨー 90°：センサー x")
    low_a = a["time"][np.argmin(a["mx"])]
    low_b = b["time"][np.argmin(b["mx"])]
    bottom = a["mx"].min() / 1000
    right.annotate("", xy=(low_b, bottom - 4), xytext=(low_a, bottom - 4), arrowprops=dict(arrowstyle="<->", color=INK))
    right.text((low_a + low_b) / 2, bottom - 5.5, f"{abs(low_a - low_b):.2f} 秒＝1/4 回転ずれる", fontsize=9,
               color=INK, ha="center", va="top")
    right.set_ylim(-48, 45)
    right.set_xlabel("時間 [s]")
    right.set_ylabel("センサー x 軸の磁場 [µT]")
    right.set_title("結果：同じ磁場を、1/4 回転ずれて見る", fontsize=11, color=INK, loc="left")
    right.legend(loc="upper center", fontsize=8.5, ncol=2)
    right.grid(alpha=0.3)
    save(fig, args, "card-mount-mag-orientation")


def baro_height(args):
    fig, left, right = card("取り付け位置：高く付けた気圧センサー")
    left.set_xlim(-0.6, 0.6)
    left.set_ylim(-0.35, 1.35)
    left.set_aspect("equal")
    left.axis("off")
    left.set_title("測り方（横から見た図、静止）", fontsize=11, color=INK, loc="left")
    left.plot([-0.35, 0.35], [0, 0], color=BODY, lw=8, solid_capstyle="round")
    for x in (-0.35, 0.35):
        left.plot([x - 0.12, x + 0.12], [0.06, 0.06], color=BODY, lw=3)
    left.plot([0, 0], [0, 1.0], color=BODY, lw=3)
    left.plot(0, 0, marker="x", color=INK, ms=9, mew=2, zorder=7)
    left.text(-0.06, -0.12, "機体の原点", fontsize=9, color=INK, ha="right")
    for z, color in ((0.0, C0), (0.5, C1), (1.0, C2)):
        left.add_patch(Rectangle((-0.03, z - 0.03), 0.06, 0.06, fc=color, ec="none", zorder=6))
        left.text(0.07, z, f"原点から {z:.1f} m 上", fontsize=9, color=color, va="center")
    left.text(-0.58, -0.3, "高いところほど空気が薄く、気圧が低い", fontsize=8.5, color=MUTED)
    names, values = [], []
    for z in (0.0, 0.5, 1.0):
        d = run(args, "baro", f"mount-{z}", "truth.json", "rest", 0.1, mount=f"0,0,{z},0,0,0")
        names.append(f"{z:.1f} m 上")
        values.append(d["pressure"][-1] - 100845.4 + 0.0)
    bars = right.bar(names, values, color=[C0, C1, C2], width=0.55)
    for bar, v in zip(bars, values):
        right.text(bar.get_x() + bar.get_width() / 2, min(v, 0) - 0.4, f"{(0.0 if abs(v) < 0.005 else v):.2f} Pa", ha="center", va="top", fontsize=9.5,
                   color=INK)
    rho_g = 1.2203 * 9.80665  # air density at 40 m in the standard atmosphere
    xs = np.array([-0.4, 2.4])
    right.plot(xs, -rho_g * 0.5 * xs, color=MUTED, lw=1, ls="--")
    right.text(-0.35, -13.8, "破線：計算 −ρ·g·h（ρ ≈ 1.22 kg/m³）。棒の先が破線に乗れば正しい", fontsize=8.5,
               color=INK)
    right.axhline(0, color=INK, lw=0.8)
    right.set_ylim(-15, 3)
    right.set_ylabel("原点に付けたときとの気圧の差 [Pa]")
    right.set_title("結果：0.5 m 高いと約 6 Pa 低く読む", fontsize=11, color=INK, loc="left")
    right.grid(alpha=0.3, axis="y")
    save(fig, args, "card-mount-baro-height")


def gps_antenna(args):
    turn = 2 * np.pi / 8.0
    fig, left, right = card("取り付け位置：前に付けた GPS アンテナ")
    sketch_axes(left)
    left.set_title("測り方（上から見た図、前が上）", fontsize=11, color=INK, loc="left")
    airframe(left, alpha=0.7)
    spin_arrow(left, 0.4, "その場で 8 秒に一回転")
    left.add_patch(plt.Circle((0, 0), 0.025, fc=C0, ec="none", zorder=6))
    left.text(0.05, -0.04, "中心のアンテナ", fontsize=9, color=C0, va="top")
    left.add_patch(plt.Circle((0, 0.2), 0.025, fc=C2, ec="none", zorder=6))
    left.text(0.05, 0.2, "0.5 m 前のアンテナ\n（図は縮めて描いている）", fontsize=9, color=C2, va="center")
    left.add_patch(plt.Circle((0, 0), 0.2, fc="none", ec=C2, ls="--", lw=1.2))
    left.text(-0.43, -0.38, "機体の原点は動かない。毎秒 10 回測位", fontsize=8.5, color=MUTED)
    for r, color, label in ((0.0, C0, "中心のアンテナ"), (0.5, C2, "0.5 m 前のアンテナ")):
        d = ec_common.sampled(run(args, "gps", f"mount-{r}", "truth.json", "turn", 8.0, rate=turn,
                                  mount=f"{r},0,0,0,0,0"))
        n = np.radians(d["lat"] - LAT0) * EARTH_R
        e = np.radians(d["lon"] - LON0) * EARTH_R * np.cos(np.radians(LAT0))
        right.plot(e, n, "o-", ms=4, lw=1, color=color, label=label)
        if r > 0:
            radius = np.hypot(n, e).mean()
            right.text(0.0, -0.05, f"半径 {radius:.3f} m", fontsize=9.5, color=color, ha="center", va="top")
    right.plot(0, 0, marker="o", color=C0, ms=9)
    right.set_aspect("equal")
    right.set_xlim(-0.75, 0.75)
    right.set_ylim(-0.65, 0.7)
    right.set_xlabel("東 [m]（出発点から）")
    right.set_ylabel("北 [m]（出発点から）")
    right.set_title("結果：GPS が測った位置。前のアンテナは円を描く", fontsize=11, color=INK, loc="left")
    right.legend(loc="upper right", fontsize=8.5)
    right.grid(alpha=0.3)
    save(fig, args, "card-mount-gps-antenna")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tool", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--implementation", default="hakoniwa")
    args = parser.parse_args()
    Path(args.out).mkdir(parents=True, exist_ok=True)
    for make in (lever_arm, gyro_orientation, mag_orientation, baro_height, gps_antenna):
        make(args)


if __name__ == "__main__":
    main()
