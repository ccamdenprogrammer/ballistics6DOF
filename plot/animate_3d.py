#!/usr/bin/env python3
"""
Professional 3D Ballistic Projectile Animation (Matplotlib)
==========================================================

Upgrades vs original:
- No per-frame remove/add of Poly3DCollection (prevents flicker, faster).
- Smooth-follow camera with damping.
- Ground plane + better grid styling.
- Mach-colored trail (Line3DCollection) + optional ghost.
- Clean telemetry panel with boxed styling.
- Widgets: Play/Pause button, time scrub slider, speed slider.
- Simple face "lighting" to make the projectile pop.
"""

import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection


# ---------------------------------------------------------------------------
# Disable ALL default matplotlib keybindings so they don't eat our events
# ---------------------------------------------------------------------------
for _k in list(plt.rcParams.keys()):
    if _k.startswith('keymap.'):
        try:
            plt.rcParams[_k] = []
        except Exception:
            pass


# ---------------------------------------------------------------------------
# CSV reader
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# Trajectory interpolation
# ---------------------------------------------------------------------------
DT_INTERP = 0.005   # 5 ms between interpolated frames

def interpolate_trajectory(data, cols):
    """Resample trajectory to uniform time steps for smooth animation."""
    t = data[:, cols['time']]
    t_new = np.arange(t[0], t[-1], DT_INTERP)

    result = {}
    for name, idx in cols.items():
        if name in ('pitch', 'yaw', 'roll'):
            result[name] = np.interp(t_new, t, np.unwrap(data[:, idx]))
        else:
            result[name] = np.interp(t_new, t, data[:, idx])

    return result, t_new


# ---------------------------------------------------------------------------
# Bullet mesh (unit length, nose = +X, centred at CG)
# ---------------------------------------------------------------------------
def create_bullet_faces(cal_m, len_m, n_circ=20):
    """
    Mesh is unit-length (1.0) with radius matching the real cal/len ratio.
    Returns: faces (N,4,3), base_colors (N,) hex strings
    """
    radius = (cal_m / 2.0) / len_m

    nose_len, body_len = 0.38, 0.42
    tail_radius = 0.78 * radius

    # nose
    n_nose = 8
    x_n = np.linspace(0, nose_len, n_nose + 1)
    r_n = radius * np.power(x_n / nose_len, 0.6)
    r_n[0] = 0.001 * radius

    # cylinder
    n_body = 4
    x_b = np.linspace(nose_len, nose_len + body_len, n_body + 1)[1:]
    r_b = np.full(len(x_b), radius)

    # boat tail
    n_tail = 4
    x_t = np.linspace(nose_len + body_len, 1.0, n_tail + 1)[1:]
    r_t = np.linspace(radius, tail_radius, n_tail + 1)[1:]

    x_all = np.concatenate([x_n, x_b, x_t])
    r_all = np.concatenate([r_n, r_b, r_t])
    x_all = 0.55 - x_all  # flip so nose is at +X (flight direction)

    theta = np.linspace(0, 2 * np.pi, n_circ + 1)
    pts = np.zeros((len(x_all), n_circ + 1, 3))
    for i in range(len(x_all)):
        pts[i, :, 0] = x_all[i]
        pts[i, :, 1] = r_all[i] * np.cos(theta)
        pts[i, :, 2] = r_all[i] * np.sin(theta)

    faces, colors = [], []
    for i in range(len(x_all) - 1):
        # slightly more refined material palette
        if i < n_nose:
            c = '#c58b2b'   # copper
        elif i < n_nose + len(x_b):
            c = '#bfa65a'   # brass-ish
        else:
            c = '#7b5e18'   # darker tail
        for j in range(n_circ):
            faces.append(np.array([pts[i, j], pts[i+1, j], pts[i+1, j+1], pts[i, j+1]]))
            colors.append(c)

    # base cap
    base = np.array([x_all[-1], 0.0, 0.0])
    for j in range(n_circ):
        faces.append(np.array([base, pts[-1, j], pts[-1, j+1], base]))
        colors.append('#2b2b2b')

    return np.array(faces), np.array(colors)


AXIS_BODY = np.array([[-0.55, 0, 0], [0.85, 0, 0]])


# ---------------------------------------------------------------------------
# Rotation / transforms
# ---------------------------------------------------------------------------
def rotation_matrix_zyx(yaw, pitch, roll):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr ]])

def transform_faces(faces_body, scale, R, pos):
    n, v, _ = faces_body.shape
    flat = faces_body.reshape(-1, 3) * scale
    out = (R @ flat.T).T + pos
    return out.reshape(n, v, 3)

def face_normals(quads):
    # quads: (N,4,3)
    v0 = quads[:, 1] - quads[:, 0]
    v1 = quads[:, 2] - quads[:, 0]
    n = np.cross(v0, v1)
    nn = np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    return n / nn


# ---------------------------------------------------------------------------
# Animator
# ---------------------------------------------------------------------------
class BulletAnimator:

    def __init__(self, filename):
        data, cols, meta = read_csv(filename)

        self.real_len = meta.get('len_m', 0.02896)
        self.real_cal = meta.get('cal_m', 0.00762)

        traj, t_new = interpolate_trajectory(data, cols)
        self.traj     = traj
        self.t        = t_new
        self.n_frames = len(t_new)

        self.px = traj['range']
        self.py = traj['crossrange']
        self.pz = traj['altitude']

        self.pitch_arr = traj['pitch']
        self.yaw_arr   = traj['yaw']
        self.roll_arr  = traj['roll']

        self.vel_arr   = traj['velocity']
        self.mach_arr  = traj['mach']
        self.spin_arr  = traj['spin_rpm']
        self.alpha_arr = traj['alpha']

        # Mesh
        self.faces_body, base_hex_colors = create_bullet_faces(
            self.real_cal, self.real_len, n_circ=22
        )
        # Precompute RGBA from hex (avoid per-frame string parsing)
        self.base_face_rgba = np.array(
            [matplotlib.colors.to_rgba(c) for c in base_hex_colors]
        )

        # Extent / scaling
        self.x_span = float(self.px.max() - self.px.min())
        self.y_span = float(self.py.max() - self.py.min())
        self.z_span = float(self.pz.max() - self.pz.min())
        self.extent = max(self.x_span, self.y_span, self.z_span, 1.0)

        # Playback
        self.display_fps   = 30
        self.speed         = 1.0 / (self.display_fps * DT_INTERP)  # ~real-time
        self.default_speed = self.speed
        self.frame_idx     = 0.0
        self.paused        = False

        # Visual
        self.show_trail = True
        self.show_ghost = True
        self.trail_len  = 700

        # Bullet visual size in world coords
        self.bullet_vis_size = self.extent * 0.020

        # Camera / zoom
        self.zoom_radius = self.extent * 0.55
        self.min_zoom    = self.bullet_vis_size * 0.35
        self.max_zoom    = self.extent * 1.75

        # Smooth follow
        self.cam_center = np.array([self.px[0], self.py[0], self.pz[0]], dtype=float)
        self.cam_smoothing = 0.20  # higher = snappier

        self._setup_figure()

        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('scroll_event',    self._on_scroll)

    # ------------------------------------------------------------------
    def _setup_figure(self):
        plt.style.use('dark_background')
        plt.rcParams['font.family'] = 'DejaVu Sans'
        plt.rcParams['figure.dpi'] = 120

        self.fig = plt.figure(figsize=(16, 10), num='6-DOF Projectile Animation (Pro)')

        # Main 3D axis
        self.ax = self.fig.add_axes([0.02, 0.05, 0.70, 0.92], projection='3d')
        self.ax.set_facecolor('#0b0d10')

        # panes off
        for a in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            a.pane.fill = False

        self.ax.set_xlabel('Range (m)', fontsize=9, labelpad=6)
        self.ax.set_ylabel('Crossrange (m)', fontsize=9, labelpad=6)
        self.ax.set_zlabel('Altitude (m)', fontsize=9, labelpad=6)

        self.ax.view_init(elev=20, azim=-70)

        # Light, subtle grid
        self.ax.grid(True)
        try:
            self.ax.xaxis._axinfo["grid"]["color"] = (0.5, 0.5, 0.5, 0.15)
            self.ax.yaxis._axinfo["grid"]["color"] = (0.5, 0.5, 0.5, 0.15)
            self.ax.zaxis._axinfo["grid"]["color"] = (0.5, 0.5, 0.5, 0.15)
        except Exception:
            pass

        # Ground plane
        self._add_ground_plane()

        # Disconnect mpl's built-in 3D scroll zoom
        for cid, func in list(self.ax.figure.canvas.callbacks.callbacks.get('scroll_event', {}).items()):
            if func and hasattr(func, '__self__') and isinstance(func.__self__, type(self.ax)):
                self.ax.figure.canvas.mpl_disconnect(cid)
                break

        # Ghost trajectory (thin)
        self.ghost_line, = self.ax.plot(
            self.px, self.py, self.pz,
            lw=1.2, alpha=0.35
        )
        self.ghost_shadow, = self.ax.plot(
            self.px, self.py, np.zeros_like(self.pz),
            lw=0.8, alpha=0.18
        )

        # Trail as colored segments (dummy segment to avoid empty-transpose crash)
        _dummy = np.array([[[0, 0, 0], [0, 0, 0]]])
        self.trail_coll = Line3DCollection(_dummy, linewidths=2.4, alpha=0.9)
        self.ax.add_collection3d(self.trail_coll)
        self.trail_coll.set_segments([])

        self.trail_shadow_coll = Line3DCollection(_dummy, linewidths=1.0, alpha=0.25)
        self.ax.add_collection3d(self.trail_shadow_coll)
        self.trail_shadow_coll.set_segments([])

        # Bullet collection created ONCE; we only update verts + colors
        pos0 = np.array([self.px[0], self.py[0], self.pz[0]])
        R0   = rotation_matrix_zyx(self.yaw_arr[0], self.pitch_arr[0], self.roll_arr[0])
        faces_world0 = transform_faces(self.faces_body, self.bullet_vis_size, R0, pos0)

        self.bullet_col = Poly3DCollection(
            faces_world0,
            edgecolors=(0, 0, 0, 0.25),
            linewidths=0.25
        )
        self.ax.add_collection3d(self.bullet_col)

        # Body axis indicator
        axis_w0 = (R0 @ (AXIS_BODY * self.bullet_vis_size).T).T + pos0
        self.axis_line, = self.ax.plot(
            axis_w0[:, 0], axis_w0[:, 1], axis_w0[:, 2],
            lw=2.6, alpha=0.9
        )

        # Right-side panel (HUD + widgets)
        self.panel = self.fig.add_axes([0.74, 0.05, 0.24, 0.92])
        self.panel.set_facecolor('#0b0d10')
        self.panel.set_xticks([]); self.panel.set_yticks([])
        for sp in self.panel.spines.values():
            sp.set_color((1, 1, 1, 0.08))

        self.panel.text(0.06, 0.96, "TELEMETRY", fontsize=14, fontweight='bold',
                        transform=self.panel.transAxes)

        # HUD text (monospace inside panel)
        self.hud_text = self.panel.text(
            0.06, 0.90, "", fontsize=10, family='monospace',
            va='top', transform=self.panel.transAxes
        )

        self.status_text = self.panel.text(
            0.06, 0.27, "", fontsize=10, family='monospace',
            va='top', transform=self.panel.transAxes, alpha=0.9
        )

        # Widgets area
        self._setup_widgets()

        # Colormap for mach trail
        self.cmap = plt.get_cmap('turbo')
        self.mach_min = float(np.nanmin(self.mach_arr))
        self.mach_max = float(np.nanmax(self.mach_arr))
        if self.mach_max <= self.mach_min + 1e-9:
            self.mach_max = self.mach_min + 1.0

    def _add_ground_plane(self):
        # a subtle ground plane centered on trajectory mid
        xmid = 0.5 * (self.px.min() + self.px.max())
        ymid = 0.5 * (self.py.min() + self.py.max())
        span = self.extent * 1.2
        xs = np.array([xmid - span, xmid + span])
        ys = np.array([ymid - span, ymid + span])
        XX, YY = np.meshgrid(xs, ys)
        ZZ = np.zeros_like(XX)

        self.ax.plot_surface(
            XX, YY, ZZ,
            rstride=1, cstride=1,
            linewidth=0, antialiased=True,
            alpha=0.08
        )

    def _setup_widgets(self):
        # Play/pause button
        ax_btn = self.fig.add_axes([0.77, 0.16, 0.18, 0.05])
        self.btn_play = Button(ax_btn, "Play / Pause")
        self.btn_play.on_clicked(lambda _e: self._toggle_pause())

        # Time scrub slider
        ax_time = self.fig.add_axes([0.77, 0.10, 0.18, 0.03])
        self.s_time = Slider(ax_time, "Time", float(self.t[0]), float(self.t[-1]),
                             valinit=float(self.t[0]))
        self.s_time.on_changed(self._on_time_slider)

        # Speed slider (sim-rate)
        ax_spd = self.fig.add_axes([0.77, 0.05, 0.18, 0.03])
        self.s_speed = Slider(ax_spd, "Speed", 0.1, 8.0, valinit=1.0)  # multiplier
        self.s_speed.on_changed(self._on_speed_slider)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def _toggle_pause(self):
        self.paused = not self.paused

    def _on_time_slider(self, val):
        # jump frame_idx to nearest time (keep fractional nice)
        i = int(np.searchsorted(self.t, val))
        i = max(0, min(self.n_frames - 1, i))
        self.frame_idx = float(i)

    def _on_speed_slider(self, mult):
        # mult applies to real-time base
        self.speed = self.default_speed * float(mult)

    def _on_key(self, event):
        k = event.key
        if k in ('up', 'right', '+', '='):
            self.speed = min(self.speed * 2.0, self.n_frames / 2)
            self._sync_speed_slider()
        elif k in ('down', 'left', '-', '_'):
            self.speed = max(self.speed * 0.5, 0.1)
            self._sync_speed_slider()
        elif k == ' ':
            self.paused = not self.paused
        elif k == 'r':
            self.speed = self.default_speed
            self.frame_idx = 0.0
            self.paused = False
            self._sync_speed_slider()
            self.s_time.set_val(float(self.t[0]))
        elif k == 't':
            self.show_trail = not self.show_trail
        elif k == 'g':
            self.show_ghost = not self.show_ghost
        elif k in ('q', 'escape'):
            plt.close(self.fig)

    def _sync_speed_slider(self):
        mult = self.speed / self.default_speed
        # clamp to slider range to avoid visual glitches
        mult = float(np.clip(mult, self.s_speed.valmin, self.s_speed.valmax))
        # avoid recursive slider callbacks
        self.s_speed.eventson = False
        self.s_speed.set_val(mult)
        self.s_speed.eventson = True

    def _on_scroll(self, event):
        factor = 1.25
        if event.button == 'up':
            self.zoom_radius = max(self.min_zoom, self.zoom_radius / factor)
        elif event.button == 'down':
            self.zoom_radius = min(self.max_zoom, self.zoom_radius * factor)

    # ------------------------------------------------------------------
    # Visual helpers
    # ------------------------------------------------------------------
    def _mach_color(self, mach):
        u = (mach - self.mach_min) / (self.mach_max - self.mach_min)
        u = float(np.clip(u, 0.0, 1.0))
        return self.cmap(u)

    def _shade_faces(self, faces_world):
        """
        Simple lighting: face color * (ambient + diffuse*dot(n,light_dir))
        Returns RGBA array for each face.
        """
        # Light coming from "camera-ish" direction
        light_dir = np.array([0.25, -0.45, 0.85], dtype=float)
        light_dir /= np.linalg.norm(light_dir) + 1e-12

        n = face_normals(faces_world)
        diff = np.clip(np.abs(n @ light_dir), 0.0, 1.0)  # two-sided lighting

        ambient = 0.35
        diffuse = 0.75
        gain = ambient + diffuse * diff

        # Use precomputed RGBA
        base_rgba = self.base_face_rgba.copy()
        base_rgba[:, :3] *= gain[:, None]
        base_rgba[:, 3] = 0.95
        base_rgba[:, :3] = np.clip(base_rgba[:, :3], 0.0, 1.0)
        return base_rgba

    # ------------------------------------------------------------------
    # Frame update
    # ------------------------------------------------------------------
    def update(self, _frame_num):
        if not self.paused:
            self.frame_idx += self.speed
            if self.frame_idx >= self.n_frames:
                self.frame_idx = 0.0

        i = int(self.frame_idx) % self.n_frames

        pos = np.array([self.px[i], self.py[i], self.pz[i]], dtype=float)
        R   = rotation_matrix_zyx(self.yaw_arr[i], self.pitch_arr[i], self.roll_arr[i])

        # Update bullet vertices (NO remove/add)
        faces_world = transform_faces(self.faces_body, self.bullet_vis_size, R, pos)
        self.bullet_col.set_verts(faces_world)
        self.bullet_col.set_facecolor(self._shade_faces(faces_world))

        # Update axis line
        axis_w = (R @ (AXIS_BODY * self.bullet_vis_size).T).T + pos
        self.axis_line.set_data(axis_w[:, 0], axis_w[:, 1])
        self.axis_line.set_3d_properties(axis_w[:, 2])
        self.axis_line.set_color((1.0, 0.25, 0.25, 0.9))

        # Trail + shadow (colored by mach)
        if self.show_trail:
            i0 = max(0, i - self.trail_len)
            P = np.column_stack([self.px[i0:i+1], self.py[i0:i+1], self.pz[i0:i+1]])
            if len(P) >= 2:
                segs = np.stack([P[:-1], P[1:]], axis=1)  # (M,2,3)
                mach_seg = self.mach_arr[i0+1:i+1]
                colors = np.array([self._mach_color(m) for m in mach_seg])
                self.trail_coll.set_segments(segs)
                self.trail_coll.set_color(colors)

                # shadow on ground
                Pg = P.copy()
                Pg[:, 2] = 0.0
                segs_g = np.stack([Pg[:-1], Pg[1:]], axis=1)
                self.trail_shadow_coll.set_segments(segs_g)
                self.trail_shadow_coll.set_color(colors * np.array([1, 1, 1, 0.35]))
            else:
                self.trail_coll.set_segments([])
                self.trail_shadow_coll.set_segments([])
        else:
            self.trail_coll.set_segments([])
            self.trail_shadow_coll.set_segments([])

        self.ghost_line.set_visible(self.show_ghost)
        self.ghost_shadow.set_visible(self.show_ghost)

        # Camera locked to projectile
        self.cam_center = pos.copy()

        elev, azim = self.ax.elev, self.ax.azim
        r = self.zoom_radius
        c = self.cam_center
        self.ax.set_xlim3d(c[0] - r, c[0] + r)
        self.ax.set_ylim3d(c[1] - r, c[1] + r)
        self.ax.set_zlim3d(max(-0.05*r, c[2] - r), c[2] + r)
        self.ax.view_init(elev=elev, azim=azim)

        # Update HUD
        v_fps    = self.vel_arr[i] * 3.28084
        alt_ft   = self.pz[i] * 3.28084
        rng_yd   = self.px[i] * 1.09361
        drift_in = self.py[i] * 39.3701
        pit_deg  = np.degrees(self.pitch_arr[i])
        yaw_deg  = np.degrees(self.yaw_arr[i])
        aoa_deg  = np.degrees(self.alpha_arr[i])
        mach     = float(self.mach_arr[i])

        hud = (
            f"t      {self.t[i]:8.3f}  s\n"
            f"range  {rng_yd:8.1f}  yd\n"
            f"alt    {self.pz[i]:8.1f}  m   ({alt_ft:,.0f} ft)\n"
            f"drift  {drift_in:+8.1f}  in\n"
            f"\n"
            f"v      {self.vel_arr[i]:8.1f}  m/s  ({v_fps:,.0f} fps)\n"
            f"Mach   {mach:8.3f}\n"
            f"spin   {self.spin_arr[i]:8.0f}  RPM\n"
            f"\n"
            f"pitch  {pit_deg:+8.2f}  deg\n"
            f"yaw    {yaw_deg:+8.2f}  deg\n"
            f"AoA    {aoa_deg:+8.3f}  deg\n"
        )
        self.hud_text.set_text(hud)

        sim_rate = self.speed * self.display_fps * DT_INTERP
        paused = "PAUSED" if self.paused else "PLAYING"
        self.status_text.set_text(
            f"Status: {paused}\n"
            f"Sim rate: {sim_rate:4.1f}x\n"
            f"Zoom r: {self.zoom_radius:,.1f} m\n"
            f"Trail: {'ON' if self.show_trail else 'OFF'}   "
            f"Ghost: {'ON' if self.show_ghost else 'OFF'}"
        )

        # Keep sliders in sync with current time (without feedback loop)
        self.s_time.eventson = False
        self.s_time.set_val(float(self.t[i]))
        self.s_time.eventson = True

        return []

    # ------------------------------------------------------------------
    def run(self):
        interval = int(1000 / self.display_fps)
        self.anim = FuncAnimation(
            self.fig, self.update,
            interval=interval,
            blit=False,
            cache_frame_data=False
        )
        plt.show()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python animate_3d_pro.py <csv_file>")
        print("\nControls:")
        print("  Mouse drag      Orbit camera")
        print("  Scroll wheel    Zoom in/out")
        print("  Up/Right arrow  Speed up (2x)")
        print("  Down/Left arrow Slow down (0.5x)")
        print("  Space           Pause/Resume")
        print("  T               Toggle trail")
        print("  G               Toggle ghost path")
        print("  R               Reset & rewind")
        print("  Q / Escape      Quit")
        sys.exit(1)

    animator = BulletAnimator(sys.argv[1])
    animator.run()
