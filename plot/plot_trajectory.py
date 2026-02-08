import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection


def read_csv(filename):
    """Read the ballistic CSV file, skipping comment lines."""
    meta = {}
    rows = []

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#'):
                parts = line[1:].strip().split()
                for part in parts:
                    if '=' in part:
                        k, v = part.split('=', 1)
                        try:
                            meta[k] = float(v)
                        except ValueError:
                            meta[k] = v
            elif line and not line.startswith('time'):
                vals = line.split(',')
                rows.append([float(v) for v in vals])

    if not rows:
        print("No data found in CSV.")
        sys.exit(1)

    data = np.array(rows)
    cols = {
        'time': 0, 'range': 1, 'crossrange': 2, 'altitude': 3,
        'velocity': 4, 'mach': 5, 'spin_rpm': 6, 'pitch': 7,
        'yaw': 8, 'roll': 9, 'alpha': 10, 'beta': 11
    }
    return data, cols, meta


def make_colored_segments(x, y, mach, cmap, norm):
    """Create line segments colored by Mach number."""
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(mach[:-1])
    lc.set_linewidth(2)
    return lc


def plot(filename, save=False):
    data, cols, meta = read_csv(filename)

    rng = data[:, cols['range']]
    alt = data[:, cols['altitude']]
    drift = data[:, cols['crossrange']]
    mach = data[:, cols['mach']]

    # convert units
    rng_yd = rng * 1.09361
    alt_m = alt
    drift_m = drift

    # mach color normalization
    cmap = plt.cm.plasma
    norm = plt.Normalize(vmin=mach.min(), vmax=mach.max())

    # dark style
    plt.style.use('dark_background')
    plt.rcParams['font.family'] = 'monospace'

    # build title string from metadata
    muzzle_vel = meta.get('muzzle_vel', 0)
    elev = meta.get('elev_deg', 0)
    tof = meta.get('tof', 0)
    title_str = f'6-DOF  |  V0={muzzle_vel:.0f} m/s  |  Elev={elev:.1f} deg  |  ToF={tof:.2f} s'

    # ---- Figure 1: Side View ----
    fig1, ax1 = plt.subplots(figsize=(14, 7), num='Side View')
    fig1.suptitle(title_str, fontsize=12, fontweight='bold')

    lc1 = make_colored_segments(rng_yd, alt_m, mach, cmap, norm)
    ax1.add_collection(lc1)
    ax1.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')

    apex_idx = np.argmax(alt_m)
    ax1.plot(rng_yd[apex_idx], alt_m[apex_idx], 'w^', markersize=10)
    ax1.annotate(f'  Apogee: {alt_m[apex_idx]:.1f} m  ({alt_m[apex_idx]*3.28084:.0f} ft)',
                 xy=(rng_yd[apex_idx], alt_m[apex_idx]),
                 color='white', fontsize=9)

    # mark impact
    ax1.plot(rng_yd[-1], alt_m[-1], 'rv', markersize=10)
    ax1.annotate(f'  Impact: {alt_m[-1]:.1f} m',
                 xy=(rng_yd[-1], alt_m[-1]),
                 color='#ff4444', fontsize=9)

    ax1.set_xlim(rng_yd.min(), rng_yd.max() * 1.02)
    ax1.set_ylim(min(alt_m.min() - 0.5, -0.5), alt_m.max() * 1.15 + 0.5)
    ax1.set_xlabel('Range (yd)', fontsize=11)
    ax1.set_ylabel('Altitude (m)', fontsize=11)
    ax1.set_title('Side View  —  Range vs Altitude', fontsize=13)
    cb1 = plt.colorbar(lc1, ax=ax1, pad=0.02)
    cb1.set_label('Mach')
    ax1.grid(True, alpha=0.15)
    fig1.tight_layout(rect=[0, 0, 1, 0.95])

    # ---- Figure 2: Top View ----
    fig2, ax2 = plt.subplots(figsize=(14, 7), num='Top View')
    fig2.suptitle(title_str, fontsize=12, fontweight='bold')

    lc2 = make_colored_segments(rng_yd, drift_m, mach, cmap, norm)
    ax2.add_collection(lc2)
    ax2.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')

    ax2.plot(rng_yd[-1], drift_m[-1], 'rv', markersize=10)
    ax2.annotate(f'  Drift: {drift_m[-1]:.2f} m  ({drift_m[-1]*39.3701:.1f} in)',
                 xy=(rng_yd[-1], drift_m[-1]),
                 color='#ff4444', fontsize=9)

    ax2.set_xlim(rng_yd.min(), rng_yd.max() * 1.02)
    drift_pad = max(abs(drift_m.min()), abs(drift_m.max())) * 1.4 + 0.5
    ax2.set_ylim(-drift_pad, drift_pad)
    ax2.set_xlabel('Range (yd)', fontsize=11)
    ax2.set_ylabel('Drift (m)', fontsize=11)
    ax2.set_title('Top View  —  Range vs Drift', fontsize=13)
    cb2 = plt.colorbar(lc2, ax=ax2, pad=0.02)
    cb2.set_label('Mach')
    ax2.grid(True, alpha=0.15)

    if meta.get('wind_y', 0) != 0 or meta.get('wind_x', 0) != 0:
        wx = meta.get('wind_x', 0)
        wy = meta.get('wind_y', 0)
        ax2.annotate('', xy=(0.92, 0.88), xytext=(0.85, 0.88),
                     xycoords='axes fraction',
                     arrowprops=dict(arrowstyle='->', color='cyan', lw=2))
        ax2.text(0.88, 0.92, f'Wind: ({wx:.1f}, {wy:.1f}) m/s',
                 transform=ax2.transAxes, fontsize=8, color='cyan', ha='center')

    fig2.tight_layout(rect=[0, 0, 1, 0.95])

    # ---- Figure 3: 3D Trajectory ----
    fig3 = plt.figure(figsize=(14, 10), num='3D Trajectory')
    fig3.suptitle(title_str, fontsize=12, fontweight='bold')
    ax3 = fig3.add_subplot(111, projection='3d')

    points_3d = np.array([rng_yd, drift_m, alt_m]).T.reshape(-1, 1, 3)
    segments_3d = np.concatenate([points_3d[:-1], points_3d[1:]], axis=1)
    lc3 = Line3DCollection(segments_3d, cmap=cmap, norm=norm)
    lc3.set_array(mach[:-1])
    lc3.set_linewidth(2)
    ax3.add_collection3d(lc3)

    # ground shadow
    ax3.plot(rng_yd, drift_m, np.zeros_like(alt_m), color='gray', alpha=0.3, linewidth=1)

    # vertical drop lines
    for i in range(0, len(rng_yd), max(1, len(rng_yd) // 20)):
        ax3.plot([rng_yd[i], rng_yd[i]], [drift_m[i], drift_m[i]],
                 [0, alt_m[i]], color='gray', alpha=0.12, linewidth=0.5)

    # mark apogee and impact in 3D
    ax3.scatter([rng_yd[apex_idx]], [drift_m[apex_idx]], [alt_m[apex_idx]],
                color='white', marker='^', s=80, zorder=5)
    ax3.scatter([rng_yd[-1]], [drift_m[-1]], [alt_m[-1]],
                color='red', marker='v', s=80, zorder=5)

    ax3.set_xlim(rng_yd.min(), rng_yd.max())
    ax3.set_ylim(-drift_pad, drift_pad)
    ax3.set_zlim(min(alt_m.min() - 0.5, -0.5), alt_m.max() * 1.15 + 0.5)
    ax3.set_xlabel('Range (yd)')
    ax3.set_ylabel('Drift (m)')
    ax3.set_zlabel('Altitude (m)')
    ax3.set_title('3D Trajectory')
    ax3.view_init(elev=25, azim=-60)

    fig3.tight_layout(rect=[0, 0, 1, 0.95])

    if save:
        fig1.savefig('side_view.png', dpi=300, bbox_inches='tight')
        fig2.savefig('top_view.png', dpi=300, bbox_inches='tight')
        fig3.savefig('3d_trajectory.png', dpi=300, bbox_inches='tight')
        print("Plots saved: side_view.png, top_view.png, 3d_trajectory.png")

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python plot_trajectory.py <csv_file> [--save]")
        sys.exit(1)

    fname = sys.argv[1]
    do_save = '--save' in sys.argv
    plot(fname, save=do_save)
