import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

model = mujoco.MjModel.from_xml_path("D:\Misc\Mart\mujoco/test_ak x8.xml")
data  = mujoco.MjData(model)

# ── config ────────────────────────────────────────────────────────────────────
NUM_WHISKERS = 8
HISTORY      = 200
PLOT_EVERY   = 30
PRINT_EVERY  = 500
MAX_DEG      = 30

# ── sensor helpers ────────────────────────────────────────────────────────────
def get_tip_vector(model, data, index):
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"tip_pos_{index}")
    adr = model.sensor_adr[sid]
    return data.sensordata[adr:adr+3].copy()

def get_whisker_deflection(model, data, index, rest_tip):
    tip  = get_tip_vector(model, data, index)
    rest = rest_tip[index]

    cos_angle = np.dot(tip, rest) / (np.linalg.norm(tip) * np.linalg.norm(rest) + 1e-12)
    magnitude = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))

    delta          = tip - rest
    whisker_length = np.linalg.norm(rest) + 1e-12
    angle_x = np.degrees(np.arcsin(np.clip(delta[0] / whisker_length, -1, 1)))
    angle_y = np.degrees(np.arcsin(np.clip(delta[1] / whisker_length, -1, 1)))

    return angle_x, angle_y, magnitude

# ── capture rest positions ────────────────────────────────────────────────────
mujoco.mj_forward(model, data)
rest_tip = {i: get_tip_vector(model, data, i) for i in range(1, NUM_WHISKERS + 1)}

# ── polar plot setup ──────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 4, figsize=(14, 7),
                         subplot_kw={"projection": "polar"})
fig.suptitle("Live Whisker Deflection", fontsize=13, fontweight="bold", color="white")
fig.patch.set_facecolor("#1e1e1e")
axes_flat = axes.flatten()

trail_plots = []
arrow_plots = []

for i, ax in enumerate(axes_flat):
    w = i + 1
    ax.set_facecolor("#2a2a2a")
    ax.set_title(f"W{w}", color="white", fontsize=10, pad=8)
    ax.set_ylim(0, MAX_DEG)
    ax.tick_params(colors="gray", labelsize=7)
    ax.spines["polar"].set_color("#444")
    ax.yaxis.set_tick_params(labelcolor="gray")
    ax.set_rlabel_position(45)
    ax.grid(color="#444444", linewidth=0.5)
    ax.set_rticks([5, 10, 15, 20, 25, 30])
    ax.text(np.radians(45), MAX_DEG * 1.15, "deg",
            color="gray", fontsize=6, ha="center")

    trail, = ax.plot([], [], color="steelblue", alpha=0.3, linewidth=1)
    arrow, = ax.plot([], [], color="tomato", linewidth=2.5,
                     marker=">", markersize=8, markevery=[-1], zorder=5)

    trail_plots.append(trail)
    arrow_plots.append(arrow)

plt.tight_layout()
plt.ion()
plt.show()

# ── history buffers ───────────────────────────────────────────────────────────
trail_theta = [deque(maxlen=HISTORY) for _ in range(NUM_WHISKERS)]
trail_r     = [deque(maxlen=HISTORY) for _ in range(NUM_WHISKERS)]

# ── plot update ───────────────────────────────────────────────────────────────
def update_plots(x_all, y_all, mag_all):
    for i in range(NUM_WHISKERS):
        w     = i + 1
        theta = np.arctan2(y_all[w], x_all[w])
        r     = mag_all[w]

        trail_theta[i].append(theta)
        trail_r[i].append(r)

        trail_plots[i].set_data(list(trail_theta[i]), list(trail_r[i]))

        # arrow: line from center (0) to current deflection (r)
        arrow_plots[i].set_data([theta, theta], [0, r])

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# ── simulation loop ───────────────────────────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0
    while viewer.is_running():
        mujoco.mj_step(model, data)

        x_all, y_all, mag_all = {}, {}, {}
        for i in range(1, NUM_WHISKERS + 1):
            x, y, mag    = get_whisker_deflection(model, data, i, rest_tip)
            x_all[i]     = x
            y_all[i]     = y
            mag_all[i]   = mag

        if step % PLOT_EVERY == 0:
            update_plots(x_all, y_all, mag_all)

        if step % PRINT_EVERY == 0:
            row = f"{data.time:>8.3f} |"
            for i in range(1, NUM_WHISKERS + 1):
                row += f"  {x_all[i]:>7.3f}  {y_all[i]:>7.3f}  {mag_all[i]:>7.3f} |"
            print(row)

        step += 1
        viewer.sync()