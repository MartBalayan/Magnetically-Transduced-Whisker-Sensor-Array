import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

model = mujoco.MjModel.from_xml_path("D:\Misc\Mart\mujoco/test_ak x16.xml")
data  = mujoco.MjData(model)

# ── config ────────────────────────────────────────────────────────────────────
NUM_WHISKERS = 16
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
# 4 rows x 4 cols matching the physical whisker grid layout
# Row 4 (top, z=0.265): W13 W14 W15 W16
# Row 3      (z=0.255): W9  W10 W11 W12
# Row 2      (z=0.245): W5  W6  W7  W8
# Row 1 (bot, z=0.235): W1  W2  W3  W4
GRID_ROWS = 4
GRID_COLS = 4

fig, axes = plt.subplots(GRID_ROWS, GRID_COLS, figsize=(16, 16),
                         subplot_kw={"projection": "polar"})
fig.suptitle("Live Whisker Deflection (4×4)", fontsize=14, fontweight="bold", color="white")
fig.patch.set_facecolor("#1e1e1e")

# Map plot position (row, col) → whisker index so the grid matches the
# physical layout: row 0 of the figure = top row = highest z.
# Whisker numbering: row r (0=bottom), col c (0=left) → index = r*4 + c + 1
# We invert rows so row 0 in the figure shows the highest-z whiskers.
whisker_index = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)
for r in range(GRID_ROWS):
    for c in range(GRID_COLS):
        physical_row = GRID_ROWS - 1 - r   # invert so top of figure = high z
        whisker_index[r, c] = physical_row * GRID_COLS + c + 1

trail_plots = {}
arrow_plots = {}

for r in range(GRID_ROWS):
    for c in range(GRID_COLS):
        w  = whisker_index[r, c]
        ax = axes[r, c]

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

        trail_plots[w] = trail
        arrow_plots[w] = arrow

plt.tight_layout()
plt.ion()
plt.show()

# ── history buffers ───────────────────────────────────────────────────────────
trail_theta = {i: deque(maxlen=HISTORY) for i in range(1, NUM_WHISKERS + 1)}
trail_r     = {i: deque(maxlen=HISTORY) for i in range(1, NUM_WHISKERS + 1)}

# ── plot update ───────────────────────────────────────────────────────────────
def update_plots(x_all, y_all, mag_all):
    for w in range(1, NUM_WHISKERS + 1):
        theta = np.arctan2(y_all[w], x_all[w])
        r     = mag_all[w]

        trail_theta[w].append(theta)
        trail_r[w].append(r)

        trail_plots[w].set_data(list(trail_theta[w]), list(trail_r[w]))
        arrow_plots[w].set_data([theta, theta], [0, r])

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# ── simulation loop ───────────────────────────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0
    while viewer.is_running():
        mujoco.mj_step(model, data)

        x_all, y_all, mag_all = {}, {}, {}
        for i in range(1, NUM_WHISKERS + 1):
            x, y, mag  = get_whisker_deflection(model, data, i, rest_tip)
            x_all[i]   = x
            y_all[i]   = y
            mag_all[i] = mag

        if step % PLOT_EVERY == 0:
            update_plots(x_all, y_all, mag_all)

        if step % PRINT_EVERY == 0:
            # Print header on first line, then data
            if step == 0:
                hdr = "    time   |"
                for i in range(1, NUM_WHISKERS + 1):
                    hdr += f"    W{i:<2} x      W{i:<2} y      W{i:<2} mag  |"
                print(hdr)
            row = f"{data.time:>8.3f} |"
            for i in range(1, NUM_WHISKERS + 1):
                row += f"  {x_all[i]:>7.3f}  {y_all[i]:>7.3f}  {mag_all[i]:>7.3f} |"
            print(row)

        step += 1
        viewer.sync()