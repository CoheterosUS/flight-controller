import matplotlib
matplotlib.use('Agg')
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from scipy.spatial.transform import Rotation

SD_PATH = r'..\Logs\FC SD.csv'

rows = []
with open(SD_PATH) as f:
    reader = csv.DictReader(f)
    for row in reader:
        d = {}
        for k, v in row.items():
            try:
                d[k] = float(v)
            except:
                d[k] = 0.0
        rows.append(d)

sensor_start = next(i for i, d in enumerate(rows) if d['pressure_pa'] > 50000)
launch_idx = next(i for i, d in enumerate(rows) if d['baro_altitude'] > 5)
t0_tick = rows[launch_idx]['tick']
for d in rows:
    d['t_rel'] = (d['tick'] - t0_tick) / 1000.0

cal_samples = [d for d in rows[sensor_start:launch_idx] if d['t_rel'] < -1][-1000:]
bias_ax = np.mean([d['accel_x'] for d in cal_samples])
bias_ay = np.mean([d['accel_y'] for d in cal_samples])
bias_az = np.mean([d['accel_z'] for d in cal_samples])
bias_gx = np.mean([d['gyro_x'] for d in cal_samples])
bias_gy = np.mean([d['gyro_y'] for d in cal_samples])
bias_gz = np.mean([d['gyro_z'] for d in cal_samples])

flight = [d for d in rows if -0.1 <= d['t_rel'] <= 10.0]
dt = 0.01
n = len(flight)
t = np.array([d['t_rel'] for d in flight])

ax_raw = np.array([d['accel_x'] - bias_ax for d in flight])
ay_raw = np.array([d['accel_y'] - bias_ay for d in flight])
az_raw = np.array([d['accel_z'] - bias_az for d in flight])
gx_raw = np.array([(d['gyro_x'] - bias_gx) * np.pi / 180.0 for d in flight])
gy_raw = np.array([(d['gyro_y'] - bias_gy) * np.pi / 180.0 for d in flight])
gz_raw = np.array([(d['gyro_z'] - bias_gz) * np.pi / 180.0 for d in flight])

R_init = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)
q_scipy = Rotation.from_matrix(R_init).as_quat()
q = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
g_world = np.array([0.0, 0.0, -9.80665])

pos = np.zeros((n, 3))
vel = np.zeros((n, 3))

for i in range(1, n):
    wx, wy, wz = gx_raw[i], gy_raw[i], gz_raw[i]
    omega_mag = np.sqrt(wx*wx + wy*wy + wz*wz)
    if omega_mag > 1e-8:
        half = omega_mag * dt * 0.5
        s = np.sin(half) / omega_mag
        dq = np.array([np.cos(half), wx*s, wy*s, wz*s])
    else:
        dq = np.array([1.0, 0.0, 0.0, 0.0])
    q = np.array([
        q[0]*dq[0] - q[1]*dq[1] - q[2]*dq[2] - q[3]*dq[3],
        q[0]*dq[1] + q[1]*dq[0] + q[2]*dq[3] - q[3]*dq[2],
        q[0]*dq[2] - q[1]*dq[3] + q[2]*dq[0] + q[3]*dq[1],
        q[0]*dq[3] + q[1]*dq[2] - q[2]*dq[1] + q[3]*dq[0],
    ])
    q /= np.linalg.norm(q)
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    a_body = np.array([ax_raw[i], ay_raw[i], az_raw[i]])
    a_world = rot.apply(a_body) + g_world
    vel[i] = vel[i-1] + a_world * dt
    pos[i] = pos[i-1] + vel[i] * dt

imu_x, imu_y, imu_z = pos[:, 0], pos[:, 1], pos[:, 2]
baro_alt = np.array([d['baro_altitude'] for d in flight])

imu_peak_z = np.max(imu_z)
true_apogee = 430.0
z_scale = true_apogee / imu_peak_z if imu_peak_z > 10 else 1.0
xy_scale = z_scale

x = imu_x * xy_scale
y = imu_y * xy_scale
z = imu_z * z_scale

speed = np.sqrt(vel[:,0]**2 + vel[:,1]**2 + vel[:,2]**2) * z_scale

print(f"IMU peak Z: {imu_peak_z:.1f}m, scale factor: {z_scale:.2f}x")
print(f"Scaled peak Z: {np.max(z):.1f}m")
print(f"Horizontal drift at apogee: {np.sqrt(x[np.argmax(z)]**2 + y[np.argmax(z)]**2):.0f}m")

gyro_mag = np.array([np.sqrt(d['gyro_x']**2 + d['gyro_y']**2 + d['gyro_z']**2) for d in flight])

fig = plt.figure(figsize=(16, 14))
fig.patch.set_facecolor('#080c14')

ax3d = fig.add_subplot(221, projection='3d', computed_zorder=False)
ax3d.set_facecolor('#080c14')

norm_t = np.clip((t - 0) / 8.0, 0, 1)
cmap = plt.cm.plasma

segments = []
seg_colors = []
stride = max(1, n // 4000)
for i in range(0, n - stride, stride):
    segments.append([(x[i], y[i], z[i]), (x[i+stride], y[i+stride], z[i+stride])])
    seg_colors.append(cmap(norm_t[i]))

lc = Line3DCollection(segments, colors=seg_colors, linewidths=2.2)
ax3d.add_collection3d(lc)

events = {}
events['IGNITION'] = 0
events['PEAK'] = int(np.argmax(z))
for i in range(len(flight)):
    if flight[i]['t_rel'] > 0.5 and flight[i]['accel_y'] > -50:
        events['BURNOUT'] = i
        break
for i in range(len(flight)):
    if flight[i]['t_rel'] > 0 and gyro_mag[i] > 500:
        events['TUMBLE'] = i
        break

ecolors = {
    'IGNITION': '#10B981', 'BURNOUT': '#F59E0B',
    'TUMBLE': '#EF4444', 'PEAK': '#3B82F6',
}

for label, idx in events.items():
    ax3d.scatter([x[idx]], [y[idx]], [z[idx]],
                 color=ecolors[label], s=90, zorder=5,
                 edgecolors='white', linewidths=0.6, depthshade=False)

    va_off = 25
    ax3d.text(x[idx], y[idx], z[idx] + va_off,
              f' {label}\n T+{t[idx]:.1f}s  {z[idx]:.0f}m',
              color=ecolors[label], fontsize=7, fontfamily='monospace',
              ha='left', zorder=6)

shadow_alpha = 0.15
ax3d.plot(x, y, np.zeros_like(z), color='#3B82F6', linewidth=0.6,
          alpha=shadow_alpha, zorder=1)

ax3d.set_xlabel('East (m)', color='#4a5a6a', fontsize=9, fontfamily='monospace', labelpad=12)
ax3d.set_ylabel('North (m)', color='#4a5a6a', fontsize=9, fontfamily='monospace', labelpad=12)
ax3d.set_zlabel('Up (m)', color='#4a5a6a', fontsize=9, fontfamily='monospace', labelpad=12)
ax3d.tick_params(colors='#2a3a4a', labelsize=7, pad=2)
for pane in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#141c28')
ax3d.grid(True, color='#141c28', linewidth=0.5)

zmax = max(np.max(z), 100)
ext = max(np.ptp(x), np.ptp(y), zmax) * 0.7
cx, cy = np.mean(x), np.mean(y)
ax3d.set_xlim(cx - ext, cx + ext)
ax3d.set_ylim(cy - ext, cy + ext)
ax3d.set_zlim(0, zmax * 1.2)
ax3d.view_init(elev=20, azim=-55)

ax_top = fig.add_subplot(222, projection='3d', computed_zorder=False)
ax_top.set_facecolor('#080c14')
lc2 = Line3DCollection(segments, colors=seg_colors, linewidths=2)
ax_top.add_collection3d(lc2)
for label, idx in events.items():
    ax_top.scatter([x[idx]], [y[idx]], [z[idx]],
                   color=ecolors[label], s=50, zorder=5,
                   edgecolors='white', linewidths=0.4, depthshade=False)
ax_top.plot(x, y, np.zeros_like(z), color='#3B82F6', linewidth=0.5,
            alpha=shadow_alpha, zorder=1)
ax_top.set_xlabel('East (m)', color='#4a5a6a', fontsize=8, fontfamily='monospace', labelpad=10)
ax_top.set_ylabel('North (m)', color='#4a5a6a', fontsize=8, fontfamily='monospace', labelpad=10)
ax_top.set_zlabel('Up (m)', color='#4a5a6a', fontsize=8, fontfamily='monospace', labelpad=10)
ax_top.tick_params(colors='#2a3a4a', labelsize=6, pad=2)
for pane in [ax_top.xaxis.pane, ax_top.yaxis.pane, ax_top.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#141c28')
ax_top.grid(True, color='#141c28', linewidth=0.5)
ax_top.set_xlim(cx - ext, cx + ext)
ax_top.set_ylim(cy - ext, cy + ext)
ax_top.set_zlim(0, zmax * 1.2)
ax_top.view_init(elev=80, azim=-90)
ax_top.set_title('Top-Down', color='#5a6a7a', fontsize=9, fontfamily='monospace', pad=8)

panel_bg = '#0c1018'
spine_c = '#141c28'

p1 = fig.add_subplot(425)
p1.set_facecolor(panel_bg)
p1.plot(t, z, color='#3B82F6', linewidth=1.3, label='IMU (scaled)')
p1.plot(t, baro_alt, color='#EF4444', linewidth=0.6, alpha=0.4, linestyle='--', label='Baro (raw)')
p1.axhline(y=430, color='#10B981', linewidth=0.7, linestyle=':', alpha=0.6)
p1.text(8.5, 435, '430m true apogee', color='#10B981', fontsize=7, fontfamily='monospace', va='bottom')
p1.set_ylabel('Altitude (m)', color='#4a5a6a', fontsize=8, fontfamily='monospace')
p1.set_title('Altitude', color='#6a7a8a', fontsize=9, fontfamily='monospace', pad=6)
p1.tick_params(colors='#2a3a4a', labelsize=7)
p1.legend(fontsize=6.5, loc='upper right', framealpha=0.2, edgecolor=spine_c, labelcolor='#6a7a8a')
p1.set_xlim(-0.5, 10)
p1.grid(True, color='#141c28', linewidth=0.4)
for sp in p1.spines.values(): sp.set_color(spine_c)

p2 = fig.add_subplot(426)
p2.set_facecolor(panel_bg)
p2.fill_between(t, 0, gyro_mag, color='#F59E0B', alpha=0.08)
p2.plot(t, gyro_mag, color='#F59E0B', linewidth=0.7)
p2.axhline(y=2000, color='#EF4444', linewidth=0.4, linestyle=':', alpha=0.3)
p2.text(8, 1950, '2000 dps limit', color='#EF4444', fontsize=6, fontfamily='monospace', va='top', alpha=0.5)
p2.set_ylabel('Rate (dps)', color='#4a5a6a', fontsize=8, fontfamily='monospace')
p2.set_title('Rotation Rate', color='#6a7a8a', fontsize=9, fontfamily='monospace', pad=6)
p2.tick_params(colors='#2a3a4a', labelsize=7)
p2.set_xlim(-0.5, 10)
p2.grid(True, color='#141c28', linewidth=0.4)
for sp in p2.spines.values(): sp.set_color(spine_c)

p3 = fig.add_subplot(427)
p3.set_facecolor(panel_bg)
p3.plot(t, [d['accel_y'] for d in flight], color='#3B82F6', linewidth=0.8, label='AY (thrust)')
p3.plot(t, [d['accel_x'] for d in flight], color='#EF4444', linewidth=0.4, alpha=0.5, label='AX')
p3.plot(t, [d['accel_z'] for d in flight], color='#10B981', linewidth=0.4, alpha=0.5, label='AZ')
p3.axhline(y=0, color='#2a3a4a', linewidth=0.4, linestyle=':')
p3.set_ylabel('m/s²', color='#4a5a6a', fontsize=8, fontfamily='monospace')
p3.set_xlabel('Time (s)', color='#4a5a6a', fontsize=8, fontfamily='monospace')
p3.set_title('Body Accelerations', color='#6a7a8a', fontsize=9, fontfamily='monospace', pad=6)
p3.tick_params(colors='#2a3a4a', labelsize=7)
p3.legend(fontsize=6, loc='lower right', framealpha=0.2, edgecolor=spine_c, ncol=3, labelcolor='#6a7a8a')
p3.set_xlim(-0.5, 10)
p3.grid(True, color='#141c28', linewidth=0.4)
for sp in p3.spines.values(): sp.set_color(spine_c)

p4 = fig.add_subplot(428)
p4.set_facecolor(panel_bg)
p4.plot(t, vel[:, 2] * z_scale, color='#3B82F6', linewidth=0.9, label='Vz (vertical)')
horiz_v = np.sqrt(vel[:,0]**2 + vel[:,1]**2) * xy_scale
p4.plot(t, horiz_v, color='#F59E0B', linewidth=0.7, alpha=0.7, label='Vxy (horizontal)')
p4.axhline(y=0, color='#2a3a4a', linewidth=0.4, linestyle=':')
if 'BURNOUT' in events:
    bo = events['BURNOUT']
    p4.axvline(x=t[bo], color='#F59E0B', linewidth=0.5, linestyle=':', alpha=0.3)
    p4.text(t[bo]+0.1, np.max(vel[:,2]*z_scale)*0.9, 'burnout', color='#F59E0B',
            fontsize=6, fontfamily='monospace', alpha=0.5)
p4.set_ylabel('m/s', color='#4a5a6a', fontsize=8, fontfamily='monospace')
p4.set_xlabel('Time (s)', color='#4a5a6a', fontsize=8, fontfamily='monospace')
p4.set_title('Velocity (scaled)', color='#6a7a8a', fontsize=9, fontfamily='monospace', pad=6)
p4.tick_params(colors='#2a3a4a', labelsize=7)
p4.legend(fontsize=6.5, loc='upper right', framealpha=0.2, edgecolor=spine_c, labelcolor='#6a7a8a')
p4.set_xlim(-0.5, 10)
p4.grid(True, color='#141c28', linewidth=0.4)
for sp in p4.spines.values(): sp.set_color(spine_c)

sm = plt.cm.ScalarMappable(cmap='plasma', norm=plt.Normalize(0, 8))
sm.set_array([])
cbar_ax = fig.add_axes([0.12, 0.52, 0.35, 0.008])
cbar = fig.colorbar(sm, cax=cbar_ax, orientation='horizontal')
cbar.set_label('Time from ignition (s)', color='#4a5a6a', fontsize=7, fontfamily='monospace')
cbar.ax.tick_params(colors='#2a3a4a', labelsize=6)
cbar.outline.set_edgecolor('#141c28')

fig.suptitle('Flight 1790512858 — 3D Trajectory Reconstruction',
             color='#c8d0dc', fontsize=15, fontfamily='monospace',
             fontweight='bold', y=0.98)

fig.text(0.5, 0.955,
         f'100Hz IIM-42653 IMU dead-reckoning  |  Z scaled {z_scale:.1f}x to match 430m true apogee  |  '
         f'Horizontal from gyro+accel integration',
         color='#3a4856', fontsize=7.5, fontfamily='monospace', ha='center')

plt.subplots_adjust(left=0.06, right=0.97, top=0.93, bottom=0.05, wspace=0.25, hspace=0.32)
plt.savefig(r'..\Graphs\trajectory_3d.png', dpi=200, facecolor='#080c14',
            bbox_inches='tight', pad_inches=0.4)
print("Saved trajectory_3d.png")
