"""Concept figure 2: the mounting pose (draft). Top views, x forward (up), y right: the FRD axes of the
drone sensor settings."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, HOST, IMU, GPS, BODY = "#14213d", "#8a96a8", "#2f6fd6", "#d1242f", "#1a7f37", "#c9d6ea"


def p(x_fwd, y_right):
    """FRD top view to plot coordinates: forward is up, right is right."""
    return (y_right, x_fwd)


def arrow(ax, a, b, color=INK, lw=1.6, style="-|>", scale=14, **kw):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=scale, color=color, lw=lw, **kw))


def airframe(ax, alpha=1.0):
    for ang in (45, 135, 225, 315):
        a = np.radians(ang)
        ax.plot([0, 0.25 * np.sin(a)], [0, 0.25 * np.cos(a)], color=BODY, lw=7, solid_capstyle="round", alpha=alpha,
                zorder=1)
        ax.add_patch(plt.Circle((0.25 * np.sin(a), 0.25 * np.cos(a)), 0.07, fc="none", ec=BODY, lw=2, alpha=alpha))
    ax.add_patch(Rectangle((-0.06, -0.06), 0.12, 0.12, fc=BODY, ec="none", alpha=alpha, zorder=1))


def body_axes(ax, origin=(0, 0), length=0.12, label=None, color=INK):
    if label is None:
        label = (L("x 前", "x fwd"), L("y 右", "y right"))
    o = np.array(origin)
    arrow(ax, o, o + (0, length), color=color)
    arrow(ax, o, o + (length, 0), color=color)
    ax.text(o[0], o[1] + length + 0.02, label[0], ha="center", va="bottom", fontsize=9, color=color)
    ax.text(o[0] + length + 0.02, o[1], label[1], ha="left", va="center", fontsize=9, color=color)


fig, axes = plt.subplots(1, 3, figsize=(16, 6.2))
for ax in axes:
    ax.set_xlim(-0.42, 0.42)
    ax.set_ylim(-0.38, 0.46)
    ax.set_aspect("equal")
    ax.axis("off")

# 1. where the sensors are
ax = axes[0]
ax.set_title(L("① どこに付いているか（上から見た図）", "① Where the sensors are (top view)"), fontsize=13, color=INK, loc="left")
airframe(ax)
ax.plot(*p(0, 0), marker="x", color=INK, ms=11, mew=2.5, zorder=5)
ax.text(*p(-0.035, 0.01), L("機体の原点", "vehicle origin"), fontsize=9, color=INK, va="top", ha="left")
ax.plot(*p(-0.02, -0.03), marker="o", color=MUTED, ms=7, zorder=5)
ax.text(*p(-0.02, -0.075), L("重心", "center of gravity"), fontsize=8.5, color=MUTED, ha="right", va="center")
imu = p(0.10, 0.05)
ax.add_patch(Rectangle((imu[0] - 0.02, imu[1] - 0.02), 0.04, 0.04, fc=IMU, ec="none", zorder=6))
ax.text(imu[0] + 0.035, imu[1], L("IMU（ジャイロ・加速度）\n地磁気も同じ姿勢", "IMU (gyro, accel)\nmag: same pose"), fontsize=8.5, color=IMU, va="center")
arrow(ax, p(0, 0), (imu[0] - 0.02, imu[1] - 0.02), color=IMU, lw=1.2, scale=10)
gps = p(-0.12, -0.04)
ax.add_patch(plt.Circle(gps, 0.022, fc=GPS, ec="none", zorder=6))
ax.text(gps[0] - 0.03, gps[1] - 0.04, L("GPS アンテナ", "GPS antenna"), fontsize=8.5, color=GPS, ha="right")
arrow(ax, p(0, 0), (gps[0] + 0.015, gps[1] + 0.015), color=GPS, lw=1.2, scale=10)
body_axes(ax, origin=(-0.38, -0.33), length=0.08)
ax.text(-0.4, 0.42,
        L("位置は「機体の原点から、前・右・下に何 m」で書く\n"
          "  \"mount\": {\"positionM\": [0.10, 0.05, -0.02], ...}\n"
          "重心は別物（オートパイロットへ渡す設定を作るときだけ使う）",
          "Position = meters forward, right, down from the vehicle origin\n"
          "  \"mount\": {\"positionM\": [0.10, 0.05, -0.02], ...}\n"
          "The center of gravity is separate (used only to make\nthe autopilot settings)"),
        fontsize=8.3, color=INK, va="top", family="Hiragino Sans")

# 2. position: turning
ax = axes[1]
ax.set_title(L("② 位置の影響：機体が回ると", "② Effect of position: when the vehicle turns"), fontsize=13, color=INK, loc="left")
airframe(ax, alpha=0.6)
ax.plot(0, 0, marker="x", color=INK, ms=10, mew=2.5, zorder=5)
ax.add_patch(Arc((0, 0), 0.76, 0.76, theta1=35, theta2=145, color=INK, lw=1.8))
arrow(ax, (0.38 * np.cos(np.radians(140)), 0.38 * np.sin(np.radians(140))),
      (0.38 * np.cos(np.radians(150)), 0.38 * np.sin(np.radians(150))), color=INK, scale=16)
ax.text(0, 0.4, L("ω（その場で回る）", "ω (turning in place)"), fontsize=10, color=INK, ha="center")
r_imu = np.array(p(0.10, 0.05))
ax.add_patch(Rectangle((r_imu[0] - 0.02, r_imu[1] - 0.02), 0.04, 0.04, fc=IMU, ec="none", zorder=6))
arrow(ax, r_imu, r_imu * 0.25, color=IMU, lw=2.2, scale=16)
ax.text(0.33, 0.05, L("加速度計：\n中心向きに\nω² × r が乗る", "Accel:\nadds ω² × r\ntoward\nthe center"), fontsize=9, color=IMU, ha="left", va="center")
arrow(ax, (0.32, 0.06), (r_imu[0] + 0.03, r_imu[1]), color=IMU, lw=0.8, style="-", scale=6)
gps_c = np.array(p(-0.12, -0.04))
ax.add_patch(plt.Circle((0, 0), np.hypot(*gps_c), fc="none", ec=GPS, ls="--", lw=1.4))
ax.add_patch(plt.Circle(gps_c, 0.022, fc=GPS, ec="none", zorder=6))
ax.text(-0.36, 0.05, L("GPS：\nアンテナは\n円を描く\n（原点は\n動かなくても）", "GPS:\nthe antenna\nmoves on\na circle\n(even if the\norigin stays)"), fontsize=9, color=GPS, ha="center", va="center")
ax.text(-0.4, -0.36, L("ジャイロは位置に関係ない：剛体のどこでも、角速度は同じ",
                         "Gyro: position does not matter; the angular rate is the same\nanywhere on a rigid body"), fontsize=9, color=INK, va="bottom")

# 3. orientation
ax = axes[2]
ax.set_title(L("③ 向きの影響：軸が入れ替わる", "③ Effect of orientation: the axes swap"), fontsize=13, color=INK, loc="left")
# the body's axes
o = np.array((-0.3, -0.05))
arrow(ax, o, o + (0, 0.2)); arrow(ax, o, o + (0.2, 0))
ax.text(o[0], o[1] + 0.22, L("x（前）", "x (fwd)"), ha="center", fontsize=9.5, color=INK)
ax.text(o[0] + 0.21, o[1], L("y（右）", "y (right)"), va="center", fontsize=9.5, color=INK)
ax.text(o[0] + 0.1, o[1] - 0.1, L("機体の軸", "Vehicle axes"), ha="center", fontsize=10.5, color=INK, fontweight="bold")
# the sensor's axes, turned 90 deg in yaw
q = np.array((0.12, 0.08))
ax.add_patch(Rectangle((q[0] - 0.03, q[1] - 0.03), 0.06, 0.06, fc=IMU, ec="none", zorder=6, alpha=0.85))
arrow(ax, q, q + (0.2, 0), color=IMU); arrow(ax, q, q + (0, -0.2), color=IMU)
ax.text(q[0] + 0.21, q[1], "x", va="center", fontsize=10, color=IMU)
ax.text(q[0], q[1] - 0.23, "y", ha="center", fontsize=10, color=IMU)
ax.text(q[0] + 0.1, q[1] - 0.33, L("センサーの軸（ヨー 90° で取り付け）", "Sensor axes (mounted at yaw 90°)"), ha="center", fontsize=10.5, color=IMU,
        fontweight="bold")
ax.add_patch(Arc(q, 0.16, 0.16, theta1=0, theta2=90, color=IMU, lw=1.2, ls="--"))
ax.text(q[0] + 0.07, q[1] + 0.1, "90°", fontsize=9, color=IMU)
ax.text(-0.4, 0.42,
        L("\"orientationDeg\": {\"yaw\": 90} で取り付けると\n"
          "センサーの x は機体の右、センサーの y は機体の後ろを向く。\n"
          "だから、機体のロール（機体 x まわりの回転）は\nセンサーの −y 軸まわりに出る",
          "Mounted with \"orientationDeg\": {\"yaw\": 90},\n"
          "the sensor x points to the vehicle's right and\nthe sensor y to the vehicle's rear.\n"
          "So a vehicle roll (about vehicle x) shows up\nabout the sensor's −y axis"),
        fontsize=8.8, color=INK, va="top")

fig.text(0.02, 0.03,
         L("取り付けの使い方：ジャイロ＝向きだけ　加速度＝位置と向き　地磁気＝向き（IMU と同じ）　気圧＝高さだけ　GPS＝位置だけ"
           "　｜　この変換はシミュレータ側の共通部品が行い、センサーの実装には取り付け済みの真値が渡る",
           "Mounting use: gyro = orientation only   accel = position and orientation   mag = orientation (same as IMU)"
           "   baro = height only   GPS = position only\n"
           "The simulator's common parts do this conversion; the sensor implementation receives the truth "
           "already at the mounting pose"),
         fontsize=9, color=HOST)
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
