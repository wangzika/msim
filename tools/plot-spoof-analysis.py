#!/usr/bin/env python3
"""Generate 3D spatial trajectory view and ENU time-series analysis plots for GNSS spoofing runs."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Configure CJK fonts for clear Chinese rendering across systems
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Noto Serif CJK JP', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def find_latest_run(root_dir: Path) -> Path:
    """Find the latest run directory containing injected-gps.jsonl."""
    runs_dir = root_dir / 'runs'
    if not runs_dir.is_dir():
        raise FileNotFoundError(f"Runs directory not found: {runs_dir}")
    candidates = []
    for p in runs_dir.glob('uav-px4-*'):
        if p.is_dir() and (p / 'injected-gps.jsonl').is_file():
            candidates.append(p)
    if not candidates:
        raise FileNotFoundError("No run directory containing injected-gps.jsonl found under runs/")
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def load_run_data(run_dir: Path):
    """Load injected-gps.jsonl and route_plan.json from the run directory."""
    gps_file = run_dir / 'injected-gps.jsonl'
    if not gps_file.is_file():
        raise FileNotFoundError(f"Missing injected-gps.jsonl in {run_dir}")

    rows = []
    with gps_file.open('r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))

    if not rows:
        raise ValueError(f"Empty data in {gps_file}")

    route = []
    route_file = run_dir / 'route_plan.json'
    if not route_file.is_file():
        # Fallback to default route_plan.json in adapter directory
        default_route = run_dir.parents[1] / 'adapters/ros1_noetic/fast_glio_uav/route_plan.json'
        if default_route.is_file():
            route_file = default_route

    if route_file.is_file():
        try:
            with route_file.open('r', encoding='utf-8') as f:
                route = json.load(f)
        except Exception:
            route = []

    return rows, route


def get_theme_colors(theme: str):
    """Get color palette for dark or light theme."""
    if theme == 'dark':
        return {
            'bg_fig': '#0b1a27',
            'bg_card': '#0e2233',
            'bg_legend': '#132738',
            'border_legend': '#284454',
            'text_main': '#dbe6ed',
            'text_dim': '#8da7b7',
            'grid': '#1c3144',
            'spine': '#284454',
            'truth': '#dbe6ed',        # Bright white/silver
            'gps': '#69baff',          # Sky blue
            'est': '#54d6a1',          # Mint green
            'plan': '#ffd044',         # Yellow
            'ref': '#d88aff',          # Purple / Magenta
            'alert': '#ff7676',        # Coral red
            'warn': '#e8a86c',         # Orange
            'axis_e': '#e98d7a',       # Salmon pink
            'axis_n': '#86d4d4',       # Cyan-teal
            'axis_u': '#dbe6ed',       # Light silver
        }
    else:  # Light / academic paper theme
        return {
            'bg_fig': '#ffffff',
            'bg_card': '#f8fafd',
            'bg_legend': '#ffffff',
            'border_legend': '#cfd9e0',
            'text_main': '#1a2733',
            'text_dim': '#5c6f84',
            'grid': '#e1e8ed',
            'spine': '#aab8c2',
            'truth': '#243b53',        # Dark slate navy
            'gps': '#0070f3',          # Classic blue
            'est': '#10b981',          # Emerald green
            'plan': '#d97706',         # Amber gold
            'ref': '#9333ea',          # Vibrant purple
            'alert': '#dc2626',        # Crimson red
            'warn': '#ea580c',         # Dark orange
            'axis_e': '#dc2626',
            'axis_n': '#0284c7',
            'axis_u': '#334155',
        }


def plot_3d_spatial(rows, route, output_path: Path, theme: str = 'dark', dpi: int = 300,
                   yaw: float = -0.68, elevation: float = 0.62):
    """Generate 3D spatial perspective view matching the dashboard 3D visualization."""
    c = get_theme_colors(theme)

    first = rows[0]
    base_east = first['east_m']
    base_north = first['north_m']
    first_alt = first['alt_m']
    altitude = lambda r: r.get('up_m', r['alt_m'] - first_alt)

    route_pts = [[base_east, base_north, 0.0]] + [
        [base_east + p['east_m'], base_north + p['north_m'], 8.0] for p in route
    ] if route else []

    truth_pts = [[r['east_m'], r['north_m'], altitude(r)] for r in rows]
    gps_pts = [[r['gps_east_m'], r['gps_north_m'], altitude(r)] for r in rows]
    est_pts = [[r['estimate_east_m'], r['estimate_north_m'], altitude(r)] for r in rows]
    ref_pts = [
        [base_east + r.get('east_bias_m', 0) + r.get('attack_east_m', 0),
         r['north_m'] + r.get('north_bias_m', 0) + r.get('attack_north_m', 0),
         altitude(r)] if r.get('attack_active') else None
        for r in rows
    ]

    all_valid = [p for p in (route_pts + truth_pts + gps_pts + est_pts + [p for p in ref_pts if p]) if p]
    xs = [p[0] for p in all_valid]
    ys = [p[1] for p in all_valid]
    zs = [p[2] for p in all_valid]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(0, min(zs)), max(8, max(zs))

    center = [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0, (min_z + max_z) / 2.0]
    radius = max(8.0, math.hypot(max_x - min_x, max_y - min_y, (max_z - min_z) * 1.3) / 2.0)

    cy, sy = math.cos(yaw), math.sin(yaw)
    ce, se = math.cos(elevation), math.sin(elevation)

    def project(p):
        x = p[0] - center[0]
        y = p[1] - center[1]
        z = p[2] - center[2]
        across = cy * x - sy * y
        depth = sy * x + cy * y
        return across, -(depth * se - z * ce)

    def proj_line(pts):
        px, py = [], []
        for p in pts:
            if p is None:
                px.append(np.nan)
                py.append(np.nan)
            else:
                x, y = project(p)
                px.append(x)
                py.append(y)
        return px, py

    fig, ax = plt.subplots(figsize=(10.5, 8), facecolor=c['bg_fig'])
    ax.set_facecolor(c['bg_fig'])

    # 1. Draw coordinate axes tripod at takeoff origin
    axis_origin = [base_east, base_north, 0]
    reach = max(5.0, radius * 0.38)
    axes_def = [
        ([axis_origin, [axis_origin[0] + reach, axis_origin[1], 0]], c['axis_e'], '东 E'),
        ([axis_origin, [axis_origin[0], axis_origin[1] + reach, 0]], c['axis_n'], '北 N'),
        ([axis_origin, [axis_origin[0], axis_origin[1], reach]], c['axis_u'], '高 U'),
    ]

    for seg, col, label in axes_def:
        px, py = proj_line(seg)
        ax.plot(px, py, color=col, lw=2.2, zorder=2)
        end_p = proj_line([seg[1]])
        ax.text(end_p[0][0] + 0.35, end_p[1][0] + 0.35, label, color=col,
                fontsize=13, fontweight='bold', zorder=10)

    # 2. Draw planned route
    if route_pts:
        px, py = proj_line(route_pts)
        ax.plot(px, py, color=c['plan'], ls='--', lw=2.4, label='预设航线 (Planned Route)', zorder=3)
        end_r = proj_line([route_pts[-1]])
        ax.scatter(end_r[0], end_r[1], color=c['plan'], s=65, zorder=6)

    # 3. Draw spoof offset reference (purple dashed trail)
    px, py = proj_line(ref_pts)
    ax.plot(px, py, color=c['ref'], ls='--', lw=2.4, label='注入偏移示意 (Spoof Offset Reference)', zorder=4)

    # 4. Draw truth trajectory (white)
    px, py = proj_line(truth_pts)
    ax.plot(px, py, color=c['truth'], lw=3.0, label='飞机真值 (Truth Trajectory)', zorder=5)
    if truth_pts:
        end_t = proj_line([truth_pts[-1]])
        ax.scatter(end_t[0], end_t[1], color=c['truth'], s=75, zorder=7)

    # 5. Draw spoofed GPS received by PX4 (cyan)
    px, py = proj_line(gps_pts)
    ax.plot(px, py, color=c['gps'], lw=2.4, label='PX4 接收的假 GPS (Spoofed GPS)', zorder=5)
    if gps_pts:
        end_g = proj_line([gps_pts[-1]])
        ax.scatter(end_g[0], end_g[1], color=c['gps'], s=75, zorder=7)

    # 6. Draw detector 2D estimate (mint green)
    px, py = proj_line(est_pts)
    ax.plot(px, py, color=c['est'], lw=1.6, alpha=0.75, label=' 滤波器估计 (EKF Detector 2D Est)', zorder=4)

    # Annotations and layout
    ax.set_aspect('equal')
    ax.axis('off')
    legend = ax.legend(loc='upper right', facecolor=c['bg_legend'], edgecolor=c['border_legend'],
                       labelcolor=c['text_main'], fontsize=10, framealpha=0.92)
    legend.get_frame().set_linewidth(1.0)

    # Compute key stats for annotation
    attacks = [r for r in rows if r.get('attack_active')]
    max_dev = max(abs(r['east_m'] - base_east) for r in rows) if rows else 0.0
    drift_rate = rows[-1].get('attack_east_m', 0.0) / max(1.0, (rows[-1]['sim_time_ns'] - attacks[0]['sim_time_ns'])/1e9) if attacks else 0.0

    info_text = f"攻击启动: {attacks[0]['sim_time_ns']/1e9 - first['sim_time_ns']/1e9:.1f}s | 最大横向偏离: {max_dev:.2f}m" if attacks else "无欺骗注入"
    ax.text(0.02, 0.03, info_text, transform=ax.transAxes, color=c['text_dim'],
            fontsize=10.5, family='sans-serif')

    plt.title('无人机 3D 空间飞行与 GNSS 欺骗轨迹', color=c['text_main'], fontsize=16, pad=18, fontweight='bold')
    plt.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=dpi, facecolor=c['bg_fig'])
    plt.close()
    print(f"[+] Saved 3D spatial plot: {output_path}")


def plot_enu_timeseries(rows, route, output_path: Path, theme: str = 'dark', dpi: int = 300):
    """Generate 4-panel aligned ENU time-series position & spoofing detection analysis plot."""
    c = get_theme_colors(theme)

    t0 = rows[0]['sim_time_ns'] / 1e9
    t = np.array([r['sim_time_ns'] / 1e9 - t0 for r in rows])

    base_east = rows[0]['east_m']
    base_north = rows[0]['north_m']
    first_alt = rows[0]['alt_m']

    east_truth = np.array([r['east_m'] for r in rows])
    north_truth = np.array([r['north_m'] for r in rows])
    up_truth = np.array([r.get('up_m', r['alt_m'] - first_alt) for r in rows])

    east_gps = np.array([r['gps_east_m'] for r in rows])
    north_gps = np.array([r['gps_north_m'] for r in rows])

    east_est = np.array([r['estimate_east_m'] for r in rows])
    north_est = np.array([r['estimate_north_m'] for r in rows])

    attack_active = np.array([r.get('attack_active', False) for r in rows])
    attack_east = np.array([r.get('attack_east_m', 0.0) for r in rows])

    chi2 = np.array([r.get('innovation_chi2') or 0.0 for r in rows])
    win_chi2 = np.array([r.get('window_chi2') or 0.0 for r in rows])
    inertial_chi2 = np.array([r.get('inertial_chi2') or 0.0 for r in rows])
    thresh = rows[0].get('threshold', 18.42068)

    attack_indices = np.where(attack_active)[0]
    attack_start_t = t[attack_indices[0]] if len(attack_indices) else None

    # First alarm time
    alarm_records = [r for r in rows if r.get('detector_status') == 'alarm']
    alarm_start_t = (alarm_records[0]['sim_time_ns'] / 1e9 - t0) if alarm_records else None

    fig, axes = plt.subplots(4, 1, figsize=(11.5, 12.5), facecolor=c['bg_fig'], sharex=True)

    for ax in axes:
        ax.set_facecolor(c['bg_card'])
        ax.tick_params(colors=c['text_dim'], labelsize=10)
        for spine in ax.spines.values():
            spine.set_color(c['spine'])
        ax.grid(True, color=c['grid'], linestyle='--', alpha=0.7)
        if attack_start_t is not None:
            ax.axvspan(attack_start_t, t[-1], color=c['alert'], alpha=0.07)
            ax.axvline(attack_start_t, color=c['alert'], linestyle=':', lw=1.5, alpha=0.85)

    # ---------------- 1. East E(t) ----------------
    ax1 = axes[0]
    ax1.plot(t, east_truth, color=c['truth'], lw=2.4, label='真实东向位置 (Truth)')
    ax1.plot(t, east_gps, color=c['gps'], lw=2.0, label='PX4 接收的假 GPS (Spoofed GPS)')
    ax1.plot(t, east_est, color=c['est'], lw=1.8, ls='-.', alpha=0.85, label='滤波器估计 (Detector Est)')
    ax1.axhline(base_east, color=c['plan'], lw=1.8, ls='--', label=f'预设航线目标 (Route E={base_east:.1f}m)')
    if np.any(attack_active):
        ax1.plot(t[attack_active], (base_east + attack_east)[attack_active], color=c['ref'],
                 lw=1.8, ls=':', label='注入东向偏移 (Injected East Offset)')

    ax1.set_ylabel('东向 E (米)', color=c['text_main'], fontsize=11, fontweight='bold')
    ax1.legend(loc='upper left', facecolor=c['bg_legend'], edgecolor=c['border_legend'],
               labelcolor=c['text_main'], fontsize=9, ncol=3)
    if attack_start_t is not None:
        ax1.text(attack_start_t + 0.6, ax1.get_ylim()[0] + (ax1.get_ylim()[1] - ax1.get_ylim()[0]) * 0.08,
                 f'欺骗注入开始: {attack_start_t:.1f}s', color=c['alert'], fontsize=10, fontweight='bold')

    # ---------------- 2. North N(t) ----------------
    ax2 = axes[1]
    ax2.plot(t, north_truth, color=c['truth'], lw=2.4, label='真实北向位置 (Truth)')
    ax2.plot(t, north_gps, color=c['gps'], lw=2.0, label='PX4 接收的假 GPS')
    ax2.plot(t, north_est, color=c['est'], lw=1.8, ls='-.', alpha=0.85, label='滤波器估计 (Detector Est)')
    ax2.set_ylabel('北向 N (米)', color=c['text_main'], fontsize=11, fontweight='bold')
    ax2.legend(loc='upper left', facecolor=c['bg_legend'], edgecolor=c['border_legend'],
               labelcolor=c['text_main'], fontsize=9, ncol=3)

    # ---------------- 3. Up U(t) ----------------
    ax3 = axes[2]
    ax3.plot(t, up_truth, color=c['truth'], lw=2.4, label='真实飞行高度 (Truth Up)')
    ax3.axhline(8.0, color=c['plan'], lw=1.8, ls='--', label='预设巡航高度 (8.0m)')
    ax3.set_ylabel('天向 U / 高度 (米)', color=c['text_main'], fontsize=11, fontweight='bold')
    ax3.legend(loc='upper left', facecolor=c['bg_legend'], edgecolor=c['border_legend'],
               labelcolor=c['text_main'], fontsize=9, ncol=2)
    ax3.set_xlabel('仿真时间 (秒)', color=c['text_main'], fontsize=11, fontweight='bold')


    # ---------------- 4. Cross-track Deviation & Chi-square Detection ----------------
    ax4 = axes[3]
    cross_track = np.abs(east_truth - base_east)
    l1 = ax4.plot(t, cross_track, color=c['warn'], lw=2.2, label='真实航线横向偏离 (Cross-Track Error)')
    l2 = ax4.plot(t, np.abs(attack_east), color=c['ref'], lw=2.0, ls=':', label='累积注入攻击偏差 (|Injected Offset|)')
    ax4.set_ylabel('偏离距离 (米)', color=c['warn'], fontsize=11, fontweight='bold')
    ax4.tick_params(axis='y', colors=c['warn'])

    # Twin axis for Chi2
    ax4_twin = ax4.twinx()
    ax4_twin.set_facecolor('none')
    l3 = ax4_twin.plot(t, np.minimum(chi2, 80), color=c['gps'], lw=1.5, alpha=0.75, label='单历元创新卡方 (Innovation χ²)')
    l4 = ax4_twin.plot(t, np.minimum(win_chi2, 80), color='#e8a86c', lw=1.8, label='10历元滑动趋势 (Windowed χ²)')
    if np.any(inertial_chi2 > 0):
        l4_b = ax4_twin.plot(t, np.minimum(inertial_chi2, 80), color='#bca6ff', lw=1.6, ls='--', label='纯惯导参考 (Inertial χ²)')
    else:
        l4_b = []
    l5 = ax4_twin.axhline(thresh, color=c['alert'], lw=1.8, ls='--', label=f'告警阈值 (Threshold={thresh:.1f})')
    ax4_twin.set_ylabel('检测卡方值 χ²', color=c['alert'], fontsize=11, fontweight='bold')
    ax4_twin.tick_params(axis='y', colors=c['alert'])
    ax4_twin.spines['right'].set_color(c['alert'])
    ax4_twin.spines['left'].set_color(c['spine'])

    if alarm_start_t is not None:
        ax4_twin.axvline(alarm_start_t, color='#f2c66d', linestyle='--', lw=1.5, alpha=0.9)
        ax4_twin.text(alarm_start_t + 0.6, thresh * 1.05, f'首次告警: {alarm_start_t:.1f}s',
                      color='#f2c66d', fontsize=9.5, fontweight='bold')

    all_lines = l1 + l2 + l3 + l4 + l4_b + [l5]
    all_labels = [l.get_label() for l in all_lines]
    ax4.legend(all_lines, all_labels, loc='upper left', facecolor=c['bg_legend'], edgecolor=c['border_legend'],
               labelcolor=c['text_main'], fontsize=8.8, ncol=3)
    ax4.set_xlabel('仿真时间 (秒)', color=c['text_main'], fontsize=11, fontweight='bold')

    plt.suptitle('无人机 GNSS 欺骗注入下 ENU 时序位置与检测响应分析', color=c['text_main'], fontsize=15.5, y=0.995, fontweight='bold')
    plt.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=dpi, facecolor=c['bg_fig'])
    plt.close()
    print(f"[+] Saved ENU time-series plot: {output_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, help='Path to run directory containing injected-gps.jsonl')
    parser.add_argument('--latest', action='store_true', help='Automatically analyze the latest run directory')
    parser.add_argument('--output-dir', type=Path, help='Output directory for generated plots (default: <run>/plots)')
    parser.add_argument('--theme', choices=('dark', 'light', 'both'), default='dark',
                        help="Visual color theme: 'dark' (matches dashboard), 'light', or 'both'")
    parser.add_argument('--dpi', type=int, default=300, help='Plot resolution DPI (default: 300)')
    parser.add_argument('--yaw', type=float, default=-0.68, help='3D perspective yaw angle in radians (default: -0.68)')
    parser.add_argument('--elevation', type=float, default=0.62, help='3D perspective elevation angle in radians (default: 0.62)')

    args = parser.parse_args()

    root_dir = Path(__file__).resolve().parents[1]
    if args.latest:
        run_dir = find_latest_run(root_dir)
        print(f"[*] Selected latest run: {run_dir}")
    elif args.run:
        run_dir = args.run.resolve()
    else:
        # Default to latest run if neither is specified
        try:
            run_dir = find_latest_run(root_dir)
            print(f"[*] No run specified, using latest run: {run_dir}")
        except Exception:
            parser.error("Please specify a run directory with --run <PATH> or use --latest")

    out_dir = args.output_dir.resolve() if args.output_dir else (run_dir / 'plots')
    out_dir.mkdir(parents=True, exist_ok=True)

    rows, route = load_run_data(run_dir)
    print(f"[*] Loaded {len(rows)} records from {run_dir.name}")

    themes = ['dark', 'light'] if args.theme == 'both' else [args.theme]
    for th in themes:
        suffix = f"_{th}" if args.theme == 'both' else ""
        p3d = out_dir / f"spoof_trajectory_3d{suffix}.png"
        penu = out_dir / f"spoof_enu_timeseries{suffix}.png"

        plot_3d_spatial(rows, route, p3d, theme=th, dpi=args.dpi, yaw=args.yaw, elevation=args.elevation)
        plot_enu_timeseries(rows, route, penu, theme=th, dpi=args.dpi)

    print(f"[*] All plots generated successfully in: {out_dir}")


if __name__ == '__main__':
    main()
