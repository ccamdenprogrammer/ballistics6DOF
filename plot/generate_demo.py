#!/usr/bin/env python3
"""
Generate demo images (2 PNGs + 1 animated GIF) from a ballistic trajectory CSV.
Usage: python generate_demo.py <csv_file> <output_prefix>
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def read_csv(filename):
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
    return np.array(rows), meta


def generate_demo(csv_path, output_prefix):
    data, meta = read_csv(csv_path)

    t     = data[:, 0]
    rng   = data[:, 1]
    cross = data[:, 2]
    alt   = data[:, 3]
    vel   = data[:, 4]
    mach  = data[:, 5]

    rng_yd  = rng   * 1.09361
    alt_ft  = alt   * 3.28084
    cross_in = cross * 39.3701
    vel_fps = vel   * 3.28084

    plt.style.use('dark_background')

    # ── Side View ──
    fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=(10, 6))
    fig1.subplots_adjust(hspace=0.38)

    ax1a.plot(rng_yd, alt_ft, color='#00ccff', lw=2)
    ax1a.fill_between(rng_yd, 0, alt_ft, alpha=0.08, color='#00ccff')
    ax1a.set_xlabel('Range (yd)')
    ax1a.set_ylabel('Altitude (ft)')
    ax1a.set_title('Trajectory  –  Side View', fontsize=13, fontweight='bold')
    ax1a.grid(True, alpha=0.2)
    ax1a.set_ylim(bottom=0)

    ax1b.plot(rng_yd, vel_fps, color='#ff6b35', lw=2, label='Velocity')
    ax1b.set_xlabel('Range (yd)')
    ax1b.set_ylabel('Velocity (fps)', color='#ff6b35')
    ax1b.grid(True, alpha=0.2)
    ax1b_t = ax1b.twinx()
    ax1b_t.plot(rng_yd, mach, color='#35ff6b', lw=2, label='Mach')
    ax1b_t.set_ylabel('Mach', color='#35ff6b')
    ax1b.set_title('Velocity & Mach  vs  Range', fontsize=13, fontweight='bold')

    fig1.savefig(f'{output_prefix}_side.png', dpi=150, bbox_inches='tight',
                 facecolor=fig1.get_facecolor())
    plt.close(fig1)
    print(f"  saved {output_prefix}_side.png")

    # ── Top View ──
    fig2, ax2 = plt.subplots(figsize=(10, 4))

    sc = ax2.scatter(rng_yd, cross_in, c=mach, cmap='turbo', s=10, alpha=0.85)
    ax2.plot(rng_yd, cross_in, color='white', lw=0.4, alpha=0.25)
    plt.colorbar(sc, ax=ax2, label='Mach', shrink=0.8)
    ax2.axhline(0, color='white', lw=0.5, alpha=0.25, ls='--')
    ax2.set_xlabel('Range (yd)')
    ax2.set_ylabel('Drift (in)')
    ax2.set_title('Drift  –  Top View', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.2)

    fig2.savefig(f'{output_prefix}_top.png', dpi=150, bbox_inches='tight',
                 facecolor=fig2.get_facecolor())
    plt.close(fig2)
    print(f"  saved {output_prefix}_top.png")

    # ── 3D Animation GIF ──
    fig3 = plt.figure(figsize=(8, 6))
    ax3 = fig3.add_subplot(111, projection='3d')
    ax3.set_facecolor('#0b0d10')
    fig3.set_facecolor('#0b0d10')

    for a in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        a.pane.fill = False

    # ghost path
    ax3.plot(rng, cross, alt, color='white', lw=0.5, alpha=0.15)
    # ground shadow
    ax3.plot(rng, cross, np.zeros_like(alt), color='white', lw=0.4, alpha=0.08)

    trail, = ax3.plot([], [], [], color='#00ccff', lw=2.2, alpha=0.85)
    marker, = ax3.plot([], [], [], 'o', color='#ff3333', markersize=7, zorder=10)

    ax3.set_xlabel('Range (m)', fontsize=8)
    ax3.set_ylabel('Crossrange (m)', fontsize=8)
    ax3.set_zlabel('Altitude (m)', fontsize=8)
    ax3.set_xlim(rng.min(), rng.max())

    y_pad = max(1.0, (cross.max() - cross.min()) * 0.3)
    ax3.set_ylim(cross.min() - y_pad, cross.max() + y_pad)
    ax3.set_zlim(0, max(alt.max() * 1.15, 1.0))
    ax3.view_init(elev=22, azim=-65)

    info = ax3.text2D(0.03, 0.94, '', transform=ax3.transAxes,
                      fontsize=9, color='white', family='monospace')

    n_frames = 150
    indices = np.linspace(0, len(t) - 1, n_frames).astype(int)

    def update(frame):
        idx = indices[frame]
        trail.set_data(rng[:idx+1], cross[:idx+1])
        trail.set_3d_properties(alt[:idx+1])
        marker.set_data([rng[idx]], [cross[idx]])
        marker.set_3d_properties([alt[idx]])
        info.set_text(
            f't={t[idx]:.2f}s  Mach {mach[idx]:.2f}  '
            f'{vel_fps[idx]:,.0f} fps  alt {alt_ft[idx]:,.0f} ft'
        )
        return trail, marker, info

    anim = FuncAnimation(fig3, update, frames=n_frames, interval=50, blit=False)
    anim.save(f'{output_prefix}_3d.gif',
              writer=PillowWriter(fps=20), dpi=100,
              savefig_kwargs={'facecolor': fig3.get_facecolor()})
    plt.close(fig3)
    print(f"  saved {output_prefix}_3d.gif")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python generate_demo.py <csv_file> <output_prefix>")
        sys.exit(1)
    generate_demo(sys.argv[1], sys.argv[2])
