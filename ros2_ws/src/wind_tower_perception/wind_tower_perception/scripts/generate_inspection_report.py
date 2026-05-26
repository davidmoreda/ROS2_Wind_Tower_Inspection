"""Aggregate a mission's detections and ask Claude to draft the inspection report.

Inputs
------
A run directory created by ``image_capture_node`` containing:

* ``manifest.json``
* ``detections.ndjson`` — one detection per line, with cylindrical pose
* ``frames/frame_NNNNNN.jpg`` + ``frames/frame_NNNNNN.json``

What this script does
---------------------
1. Loads ``detections.ndjson`` and clusters detections that are close in
   (x_axial, theta_surface) into unique defects (the same logic used by
   ``defect_mapper_node``, kept here so the script can run fully offline).
2. Builds a compact JSON summary of the run (counts per class, per-defect
   pose, observations, max confidence, representative frame).
3. Calls the Claude API with that summary and a structured prompt to draft a
   Markdown inspection report. Optionally attaches representative defect
   thumbnails so the model can describe them in plain language.
4. Writes:
   - ``report/inspection_report.md``
   - ``report/inspection_summary.json``

Authentication
--------------
The script expects the environment variable ``ANTHROPIC_API_KEY``. The model
defaults to ``claude-opus-4-7`` and can be overridden with ``--model``.

Run with ``--dry-run`` to skip the API call and only emit the JSON summary;
useful in CI or when you do not have an API key handy.
"""

import argparse
import base64
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional


DEFAULT_MODEL = 'claude-opus-4-7'
DEFAULT_MAX_TOKENS = 4096


@dataclass
class Cluster:
    cluster_id: int
    class_id: str
    x_axial_m: float
    theta_surface_deg: float
    observations: int = 1
    max_score: float = 0.0
    frames: List[str] = field(default_factory=list)
    representative_frame: Optional[str] = None
    representative_score: float = -1.0


def _angle_diff_deg(a: float, b: float) -> float:
    diff = ((a % 360.0) - (b % 360.0) + 540.0) % 360.0 - 180.0
    return abs(diff)


def _cluster_detections(records, x_tol_m=0.30, theta_tol_deg=5.0) -> List[Cluster]:
    clusters: List[Cluster] = []
    next_id = 1
    for rec in records:
        det = rec.get('detection') or {}
        cyl = rec.get('cylindrical_pose') or {}
        if not cyl:
            continue
        class_id = str(det.get('class_id', 'unknown'))
        score = float(det.get('score', 0.0))
        x_axial = float(cyl.get('x_m', 0.0))
        # The capture-time pose is the ROBOT's pose at the bottom; the projection
        # to the wall happens in defect_mapper_node. For the offline report we
        # use whatever is recorded as the most informative coordinate available.
        theta_surface = float(cyl.get('theta_surface_deg', 0.0))
        image_path = rec.get('image_path')

        best: Optional[Cluster] = None
        best_cost = math.inf
        for c in clusters:
            if c.class_id != class_id:
                continue
            dx = abs(c.x_axial_m - x_axial)
            dtheta = _angle_diff_deg(c.theta_surface_deg, theta_surface)
            if dx <= x_tol_m and dtheta <= theta_tol_deg:
                cost = dx / max(x_tol_m, 1e-6) + dtheta / max(theta_tol_deg, 1e-6)
                if cost < best_cost:
                    best_cost = cost
                    best = c

        if best is None:
            best = Cluster(
                cluster_id=next_id,
                class_id=class_id,
                x_axial_m=x_axial,
                theta_surface_deg=theta_surface,
                max_score=score,
            )
            next_id += 1
            clusters.append(best)
        else:
            n = best.observations
            best.x_axial_m = (best.x_axial_m * n + x_axial) / (n + 1)
            best.theta_surface_deg = (
                (best.theta_surface_deg * n + theta_surface) / (n + 1)
            ) % 360.0
            best.observations = n + 1
            best.max_score = max(best.max_score, score)
        if image_path:
            best.frames.append(image_path)
            if score > best.representative_score:
                best.representative_frame = image_path
                best.representative_score = score
    return clusters


def _build_summary(manifest: dict, clusters: List[Cluster]) -> dict:
    per_class: Dict[str, int] = {}
    for c in clusters:
        per_class[c.class_id] = per_class.get(c.class_id, 0) + 1
    clusters_payload = []
    for c in clusters:
        clusters_payload.append({
            'id': c.cluster_id,
            'class_id': c.class_id,
            'x_axial_m': round(c.x_axial_m, 3),
            'theta_surface_deg': round(c.theta_surface_deg, 2),
            'observations': c.observations,
            'max_score': round(c.max_score, 3),
            'representative_frame': c.representative_frame,
        })
    return {
        'run_id': manifest.get('run_id'),
        'run_dir': manifest.get('run_dir'),
        'started_at_iso': manifest.get('started_at_iso'),
        'updated_at_iso': manifest.get('updated_at_iso'),
        'total_frames_saved': manifest.get('total_frames_saved', 0),
        'total_unique_defects': len(clusters),
        'defects_per_class': per_class,
        'defects': clusters_payload,
    }


def _load_image_b64(path: str) -> Optional[str]:
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, 'rb') as fh:
            data = fh.read()
        return base64.standard_b64encode(data).decode('ascii')
    except OSError:
        return None


def _build_messages(
    summary: dict,
    run_dir: str,
    *,
    attach_thumbnails: int,
):
    content = [
        {
            'type': 'text',
            'text': (
                'You are drafting an inspection report for a wind-tower internal '
                'inspection mission. The robot scanned the inside of a tube and '
                'detected circular defects (rust, pitting, through-holes). '
                'Below is a JSON summary of the mission and (optionally) sample '
                'detection thumbnails. Produce a Markdown report with:\n\n'
                '1. Executive summary (1 paragraph).\n'
                '2. Per-class counts and severity assessment.\n'
                '3. Table of all defects with columns: id, class, x_axial (m), '
                'theta_surface (deg), observations, max_score.\n'
                '4. Notable findings — pick 3-5 defects that look the most '
                'concerning based on confidence and class, describe them '
                'briefly, and reference their representative frame path.\n'
                '5. Recommendations for follow-up (manual review, NDT, etc).\n\n'
                'Be concise and engineer-readable. Do not invent measurements '
                'that are not in the JSON. If a field is missing, say so.'
            ),
        },
        {
            'type': 'text',
            'text': '## Mission summary (JSON)\n```json\n' + json.dumps(
                summary, indent=2) + '\n```',
        },
    ]

    attached = 0
    for cluster in summary['defects']:
        if attached >= attach_thumbnails:
            break
        rel = cluster.get('representative_frame')
        if not rel:
            continue
        abs_path = os.path.join(run_dir, rel)
        b64 = _load_image_b64(abs_path)
        if not b64:
            continue
        content.append({
            'type': 'image',
            'source': {
                'type': 'base64',
                'media_type': 'image/jpeg',
                'data': b64,
            },
        })
        content.append({
            'type': 'text',
            'text': (
                f'^ Defect id {cluster["id"]} ({cluster["class_id"]}) at '
                f'x_axial={cluster["x_axial_m"]} m, '
                f'theta_surface={cluster["theta_surface_deg"]} deg.'
            ),
        })
        attached += 1

    return [{'role': 'user', 'content': content}]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True,
                        help='Path to a mission run directory '
                             '(e.g. ~/ROS2_Wind_Tower_Inspection/inspections/run_YYYYMMDD_HHMMSS).')
    parser.add_argument('--output-dir', default=None,
                        help='Where to write the report. Defaults to '
                             '<run-dir>/report/.')
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--max-tokens', type=int, default=DEFAULT_MAX_TOKENS)
    parser.add_argument('--cluster-x-tol-m', type=float, default=0.30)
    parser.add_argument('--cluster-theta-tol-deg', type=float, default=5.0)
    parser.add_argument('--attach-thumbnails', type=int, default=5,
                        help='Number of representative defect frames to send '
                             'as images alongside the JSON summary.')
    parser.add_argument('--dry-run', action='store_true',
                        help='Skip the API call; only emit inspection_summary.json.')
    args = parser.parse_args(argv)

    run_dir = os.path.expanduser(args.run_dir)
    manifest_path = os.path.join(run_dir, 'manifest.json')
    detections_path = os.path.join(run_dir, 'detections.ndjson')
    if not os.path.isfile(manifest_path):
        print(f'[generate_inspection_report] missing manifest: {manifest_path}',
              file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(detections_path):
        print(
            f'[generate_inspection_report] missing detections log: '
            f'{detections_path}',
            file=sys.stderr,
        )
        sys.exit(1)

    with open(manifest_path, 'r', encoding='utf-8') as fh:
        manifest = json.load(fh)
    records = []
    with open(detections_path, 'r', encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue

    clusters = _cluster_detections(
        records,
        x_tol_m=args.cluster_x_tol_m,
        theta_tol_deg=args.cluster_theta_tol_deg,
    )
    summary = _build_summary(manifest, clusters)

    output_dir = args.output_dir or os.path.join(run_dir, 'report')
    output_dir = os.path.expanduser(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    summary_path = os.path.join(output_dir, 'inspection_summary.json')
    report_path = os.path.join(output_dir, 'inspection_report.md')
    with open(summary_path, 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, indent=2)
    print(f'[generate_inspection_report] summary: {summary_path}')

    if args.dry_run:
        print('[generate_inspection_report] --dry-run set; skipping API call.')
        return

    api_key = os.environ.get('ANTHROPIC_API_KEY')
    if not api_key:
        print(
            '[generate_inspection_report] ANTHROPIC_API_KEY not set. Skipping '
            'the LLM call. The JSON summary has been written; rerun without '
            '--dry-run once the key is configured.',
            file=sys.stderr,
        )
        sys.exit(2)

    try:
        import anthropic
    except ImportError:
        print(
            '[generate_inspection_report] the `anthropic` Python SDK is not '
            'installed. Install with `pip install anthropic`.',
            file=sys.stderr,
        )
        sys.exit(1)

    client = anthropic.Anthropic(api_key=api_key)
    messages = _build_messages(
        summary, run_dir, attach_thumbnails=args.attach_thumbnails)
    response = client.messages.create(
        model=args.model,
        max_tokens=args.max_tokens,
        messages=messages,
    )
    text_parts = [
        block.text for block in response.content
        if getattr(block, 'type', None) == 'text'
    ]
    report_text = '\n'.join(text_parts).strip() or '(empty response)'
    with open(report_path, 'w', encoding='utf-8') as fh:
        fh.write(report_text + '\n')
    print(f'[generate_inspection_report] report: {report_path}')


if __name__ == '__main__':
    main()
