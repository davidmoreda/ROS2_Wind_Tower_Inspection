#!/usr/bin/env python3

"""Plot IMU yaw/roll/pitch grouped by mission state.

Reads debug_runs/<timestamp>/imu.jsonl produced by capture_inspection_debug.py and
generates a PNG with colored segments by state-machine phase.
"""

import argparse
import json
import math
import pathlib
import sys


def _load_imu(path: pathlib.Path):
    samples = []
    with path.open('r', encoding='ascii', errors='replace') as f:
        for raw in f:
            raw = raw.strip()
            if not raw:
                continue
            try:
                rec = json.loads(raw)
            except json.JSONDecodeError:
                continue
            t = rec.get('t')
            rpy = rec.get('rpy_deg') or {}
            state = rec.get('state') or 'UNKNOWN'
            if t is None:
                continue
            roll = rpy.get('roll')
            pitch = rpy.get('pitch')
            yaw = rpy.get('yaw')
            if roll is None or pitch is None or yaw is None:
                continue
            samples.append({
                't': float(t),
                'state': str(state),
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw),
            })
    return samples


def _segments(samples):
    if not samples:
        return []
    out = []
    start = 0
    for i in range(1, len(samples)):
        if samples[i]['state'] != samples[i - 1]['state']:
            out.append((start, i - 1, samples[start]['state']))
            start = i
    out.append((start, len(samples) - 1, samples[start]['state']))
    return out


def _state_color_map(states):
    palette = [
        '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
        '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
    ]
    return {s: palette[i % len(palette)] for i, s in enumerate(states)}


def _latest_run(root: pathlib.Path) -> pathlib.Path:
    runs = sorted([p for p in root.iterdir() if p.is_dir()])
    if not runs:
        raise FileNotFoundError(f'No runs found in {root}')
    return runs[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=str, default='', help='Path to debug_runs/<timestamp>.')
    parser.add_argument('--output', type=str, default='imu_by_state_yaw_roll_pitch.png', help='Output PNG filename (inside run dir unless absolute).')
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception:
        print('ERROR: matplotlib is required. Install with: pip install matplotlib', file=sys.stderr)
        return 2

    if args.run:
        run_dir = pathlib.Path(args.run).expanduser().resolve()
    else:
        run_dir = _latest_run(pathlib.Path('debug_runs').resolve())

    imu_path = run_dir / 'imu.jsonl'
    if not imu_path.exists():
        print(f'ERROR: missing file {imu_path}', file=sys.stderr)
        return 2

    samples = _load_imu(imu_path)
    if not samples:
        print(f'ERROR: no valid IMU samples in {imu_path}', file=sys.stderr)
        return 2

    segs = _segments(samples)
    state_order = []
    seen = set()
    for _, _, state in segs:
        if state not in seen:
            seen.add(state)
            state_order.append(state)
    colors = _state_color_map(state_order)

    t0 = samples[0]['t']
    ts = [s['t'] - t0 for s in samples]
    roll = [s['roll'] for s in samples]
    pitch = [s['pitch'] for s in samples]
    yaw = [s['yaw'] for s in samples]

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle('IMU angles by state (yaw, roll, pitch)')

    channels = [
        ('yaw', yaw, axes[0]),
        ('roll', roll, axes[1]),
        ('pitch', pitch, axes[2]),
    ]

    for name, arr, ax in channels:
        for si, ei, state in segs:
            x = ts[si:ei + 1]
            y = arr[si:ei + 1]
            label = state if name == 'yaw' else None
            ax.plot(x, y, color=colors[state], linewidth=1.2, label=label)
        ax.set_ylabel(f'{name} (deg)')
        ax.grid(True, alpha=0.25)

    axes[-1].set_xlabel('time since start (s)')

    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        uniq = {}
        for h, l in zip(handles, labels):
            if l and l not in uniq:
                uniq[l] = h
        axes[0].legend(uniq.values(), uniq.keys(), loc='upper right', ncol=2, fontsize=8)

    if pathlib.Path(args.output).is_absolute():
        out = pathlib.Path(args.output)
    else:
        out = run_dir / args.output

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'OK: wrote {out}')

    summary = run_dir / 'imu_plot_summary.txt'
    with summary.open('w', encoding='ascii', errors='replace') as f:
        f.write(f'run_dir={run_dir}\n')
        f.write(f'samples={len(samples)}\n')
        f.write('states=' + ','.join(state_order) + '\n')
        for name, arr, _ in channels:
            f.write(f'{name}_min={min(arr)}\n')
            f.write(f'{name}_max={max(arr)}\n')
            mean = sum(arr) / max(1, len(arr))
            f.write(f'{name}_mean={mean}\n')
    print(f'OK: wrote {summary}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
