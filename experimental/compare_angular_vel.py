import pickle
import numpy as np
import matplotlib.pyplot as plt

data = pickle.load(open('scan_data.pkl', 'rb'))

scan_stamps = np.array(data['scan_stamps'])
icp_transforms = np.array(data['icp_transforms'])  # shape (N, 3, 3)
t265_stamps = np.array(data['t265_stamps'])
t265_angular_vel = np.array(data['t265_angular_vel'])

# --- ICP angular velocity ---
# icp_transforms[i] maps scan[i+1] onto scan[i], so its rotation angle is
# the yaw change the robot made between those two scans.
icp_angles = np.arctan2(icp_transforms[:, 1, 0], icp_transforms[:, 0, 0])
dt = np.diff(scan_stamps)  # time between consecutive scans
icp_ang_vel = icp_angles / dt  # rad/s

# Filter spikes: a spike at index i is when the value jumps far from both
# its left and right neighbors (large diff in, large diff out, opposite signs).
# Replace such values with the average of their neighbors.
def despike(v, threshold_factor=5.0):
    v = v.copy()
    diffs = np.diff(v)
    mad = np.median(np.abs(diffs - np.median(diffs)))
    threshold = threshold_factor * mad
    for i in range(1, len(v) - 1):
        left = diffs[i - 1]   # v[i] - v[i-1]
        right = diffs[i]      # v[i+1] - v[i]
        if abs(left) > threshold and abs(right) > threshold and left * right < 0:
            v[i] = (v[i - 1] + v[i + 1]) / 2
    return v

icp_ang_vel = despike(icp_ang_vel)

# Timestamps for ICP: midpoint of each scan pair
icp_timestamps = (scan_stamps[:-1] + scan_stamps[1:]) / 2

# --- Normalize time to start from 0 ---
t0 = min(scan_stamps[0], t265_stamps[0])
icp_t = icp_timestamps - t0
t265_t = t265_stamps - t0

# --- Rolling average ---
WINDOW_SEC = 1.0  # rolling window width in seconds

def rolling_avg(t, v, window_sec):
    out = np.empty_like(v)
    for i in range(len(t)):
        mask = (t >= t[i] - window_sec / 2) & (t <= t[i] + window_sec / 2)
        out[i] = v[mask].mean()
    return out

icp_smooth = rolling_avg(icp_t, icp_ang_vel, WINDOW_SEC)
t265_smooth = rolling_avg(t265_t, t265_angular_vel, WINDOW_SEC)

# --- Phase difference via cross-correlation on a common time grid ---
# Interpolate both smoothed signals onto a uniform grid
dt_grid = 0.05  # 20 Hz
t_start = max(icp_t[0], t265_t[0])
t_end   = min(icp_t[-1], t265_t[-1])
t_grid  = np.arange(t_start, t_end, dt_grid)

icp_interp  = np.interp(t_grid, icp_t, icp_smooth)
t265_interp = np.interp(t_grid, t265_t, t265_smooth)

# Normalize before cross-correlating
def normalize(v):
    v = v - v.mean()
    std = v.std()
    return v / std if std > 0 else v

xcorr = np.correlate(normalize(icp_interp), normalize(t265_interp), mode='full')
lags  = np.arange(-(len(t_grid) - 1), len(t_grid)) * dt_grid
lag_sec = lags[np.argmax(xcorr)]  # positive = ICP leads T265

# --- Plot ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

# Top: raw + smoothed signals
ax1.plot(t265_t, t265_angular_vel, alpha=0.25, linewidth=0.8, color='tab:blue')
ax1.plot(t265_t, t265_smooth, label='T265 (smoothed)', linewidth=1.8, color='tab:blue')
ax1.plot(icp_t, icp_ang_vel, alpha=0.25, linewidth=0.8, color='tab:orange')
ax1.plot(icp_t, icp_smooth, label='ICP (smoothed)', linewidth=1.8, color='tab:orange')
ax1.set_ylabel('Angular velocity (rad/s)')
ax1.set_title(f'ICP vs T265 Angular Velocity  |  Phase lag: {lag_sec*1000:.1f} ms (+ = ICP leads)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Bottom: cross-correlation
ax2.plot(lags, xcorr / len(t_grid))
ax2.axvline(lag_sec, color='red', linestyle='--', label=f'Peak lag = {lag_sec*1000:.1f} ms')
ax2.set_xlabel('Lag (s)')
ax2.set_ylabel('Normalized cross-correlation')
ax2.set_title('Cross-correlation (ICP vs T265)')
ax2.legend()
ax2.grid(True, alpha=0.3)
# Zoom to ±5 s around centre for readability
ax2.set_xlim(-5, 5)

plt.tight_layout()
plt.savefig('angular_vel_comparison.png', dpi=150)
plt.show()
print(f"Phase lag: {lag_sec*1000:.1f} ms  (positive = ICP leads T265)")
print("Saved angular_vel_comparison.png")
