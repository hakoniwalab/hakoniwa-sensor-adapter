"""Force-torque concept figure 1: where the sensor sits and what it measures (draft, Japanese/English)."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, FancyArrowPatch, FancyBboxPatch, Rectangle

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
LANG = sys.argv[2] if len(sys.argv) > 2 else "ja"


def L(ja, en):
    return ja if LANG == "ja" else en

INK, MUTED, ROBOT, SENSOR, TOOL, FORCE, TORQUE = "#14213d", "#8a96a8", "#c9d6ea", "#d1242f", "#7c8aa3", "#2f6fd6", "#1a7f37"


def arrow(ax, a, b, color=INK, lw=2, scale=16, style="-|>"):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=scale, color=color, lw=lw))


def wrist(ax, x0=0.0, top=1.0, obj=None, obj_dx=0.0, contact=False):
    """A wrist seen from the side: the robot link above, the sensor disk, the gripper below."""
    ax.add_patch(Rectangle((x0 - 0.12, top), 0.24, 0.45, fc=ROBOT, ec="none"))           # robot link
    ax.add_patch(Rectangle((x0 - 0.16, top - 0.08), 0.32, 0.08, fc=SENSOR, ec="none"))    # sensor
    ax.add_patch(Rectangle((x0 - 0.1, top - 0.28), 0.2, 0.2, fc=TOOL, ec="none"))         # gripper body
    for dx in (-0.08, 0.08):
        ax.add_patch(Rectangle((x0 + dx - 0.02, top - 0.48), 0.04, 0.2, fc=TOOL, ec="none"))  # fingers
    if obj is not None and obj_dx == 0:
        ax.add_patch(Rectangle((x0 - 0.06, top - 0.46), 0.12, 0.12, fc=obj, ec="none"))  # held between the fingers
    elif obj is not None:
        ax.add_patch(Rectangle((x0 - 0.06, top - 0.42), obj_dx + 0.06, 0.04, fc=obj, ec="none"))  # a bar held forward
        ax.add_patch(Rectangle((x0 + obj_dx - 0.06, top - 0.5), 0.12, 0.12, fc=obj, ec="none"))
    if contact:
        ax.plot([x0 - 0.4, x0 + 0.4], [top - 0.49, top - 0.49], color=INK, lw=2)
        for k in np.linspace(x0 - 0.38, x0 + 0.38, 9):
            ax.plot([k, k - 0.04], [top - 0.49, top - 0.55], color=INK, lw=0.8)


fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), gridspec_kw={"width_ratios": [1, 0.9, 1.5]})
for ax in axes:
    ax.set_aspect("equal")
    ax.axis("off")

# 1. where it is
ax = axes[0]
ax.set_xlim(-0.75, 0.75)
ax.set_ylim(0.0, 1.75)
ax.set_title(L("① どこに付いているか（横から見た図）", "① Where it sits (side view)"), fontsize=12.5, color=INK, loc="left")
wrist(ax, obj="#b26a00")
ax.text(0.17, 1.25, L("ロボット側\n（手首）", "Robot side\n(wrist)"), fontsize=10, color=INK, va="center")
ax.text(0.2, 0.96, L("力覚センサー", "Force-torque\nsensor"), fontsize=11 if LANG == "ja" else 10, color=SENSOR, fontweight="bold", va="center")
ax.text(0.17, 0.72, L("工具側\n（グリッパと\n　つかんだ物）", "Tool side\n(gripper and\nheld object)"), fontsize=10, color=INK, va="center")
ax.annotate("", xy=(0.15, 0.97), xytext=(0.19, 0.97))
ax.text(-0.72, 0.3, L("センサーは、工具側がロボット側を\n押したり引いたりする力を測る",
          "The sensor measures how the tool side\npushes and pulls on the robot side"),
        fontsize=10, color=INK)
ax.text(-0.72, 0.12, L("これは手首に付ける 6 軸タイプ（このページ）。\n指先に付けて、触れた力を測るタイプもある",
          "This page: the 6-axis wrist type.\nFingertip sensors that measure touch are a different kind"),
        fontsize=9, color=MUTED)

# 2. the six axes
ax = axes[1]
ax.set_xlim(-0.75, 0.75)
ax.set_ylim(-0.75, 0.95)
ax.set_title(L("② 6 つの値（センサーの軸）", "② Six values (sensor axes)"), fontsize=12.5, color=INK, loc="left")
ax.add_patch(plt.Circle((0, 0), 0.2, fc=SENSOR, ec="none", alpha=0.85))
arrow(ax, (0, 0), (0.5, -0.18), FORCE)
arrow(ax, (0, 0), (-0.42, -0.22), FORCE)
arrow(ax, (0, 0), (0, 0.55), FORCE)
ax.text(0.52, -0.25, L("Fx（前）", "Fx (forward)"), fontsize=10, color=FORCE)
ax.text(-0.72, -0.3, L("Fy（左）", "Fy (left)"), fontsize=10, color=FORCE)
ax.text(0.04, 0.58, L("Fz（上）", "Fz (up)"), fontsize=10, color=FORCE)
for (cx, cy, w, h, label, pos) in ((0.32, -0.12, 0.28, 0.12, "Tx", (0.36, -0.03)),
                                    (-0.27, -0.14, 0.28, 0.12, "Ty", (-0.42, -0.04)),
                                    (0, 0.36, 0.3, 0.1, "Tz", (0.18, 0.4))):
    ax.add_patch(Arc((cx, cy), w, h, theta1=200, theta2=520, color=TORQUE, lw=1.6))
    ax.text(*pos, label, fontsize=10, color=TORQUE)
ax.text(-0.72, -0.62, L("力 F（3 つ、単位 N）：押す・引く\nモーメント T（3 つ、単位 N·m）：ねじる・傾ける",
          "Force F (3 values, N): push, pull\nMoment T (3 values, N·m): twist, tilt"),
        fontsize=9.5, color=INK)

# 3. what goes into it: three scenes
ax = axes[2]
ax.set_xlim(-0.6, 2.6)
ax.set_ylim(-0.25, 1.75)
ax.set_title(L("③ 何が値になるか（例：グリッパ 1 kg、物 0.5 kg）", "③ What goes into the value (gripper 1 kg, object 0.5 kg)"), fontsize=12.5, color=INK, loc="left")
scenes = [
    (0.0, None, 0.0, False, L("何も持たない", "Holding nothing"), L("Fz = −9.8 N\n（グリッパの重さ）", "Fz = −9.8 N\n(gripper weight)")),
    (1.0, "#b26a00", 0.0, False, L("物を持つ", "Holding an object"), L("Fz = −14.7 N\n（物の重さが足される）", "Fz = −14.7 N\n(object weight added)")),
    (2.0, "#b26a00", 0.1, False, L("前寄りに持つ", "Holding it forward"), L("Fz = −14.7 N\nTy = +0.49 N·m\n（0.1 m 前の重さで傾ける）", "Fz = −14.7 N\nTy = +0.49 N·m\n(weight 0.1 m forward tilts it)")),
]
for x0, obj, dx, contact, title, value in scenes:
    wrist(ax, x0=x0, obj=obj, obj_dx=dx * 3.0, contact=contact)
    ax.text(x0, 1.52, title, fontsize=10.5, color=INK, ha="center", fontweight="bold")
    ax.text(x0, 0.32, value, fontsize=9.5, color=FORCE, ha="center", va="top")
    arrow(ax, (x0 - 0.24, 0.92), (x0 - 0.24, 0.72 if obj is None else 0.62), FORCE, lw=2.2)
ax.add_patch(Arc((2.0, 0.96), 0.5, 0.5, theta1=200, theta2=340, color=TORQUE, lw=1.8))
# y points into the page (x forward to the right, z up), so a positive Ty turns clockwise here:
# the weight in front tips the tool forward.
arrow(ax, (1.8, 0.82), (1.765, 0.86), TORQUE, lw=1.8, scale=14)
ax.text(1.62, 0.66, "Ty", fontsize=10, color=TORQUE)
ax.text(1.2, -0.22, L("重さは下向き。センサーの z は上向きなので、Fz はマイナスになる。\n"
                      "実際の値には、物に触れたときの力や、腕を動かしたときの慣性の力も入る",
                      "Weight points down and the sensor z points up, so Fz is negative.\n"
                      "Real values also include contact forces and inertial forces when the arm moves"),
        fontsize=9, color=MUTED, ha="center")

fig.subplots_adjust(left=0.02, right=0.99, top=0.92, bottom=0.02, wspace=0.12)
fig.savefig(sys.argv[1], dpi=110)
print("wrote", sys.argv[1])
