"""Genera un informe de inspección de torre eólica con Gemini.

Inputs
------
Un directorio de misión creado por ``image_capture_node`` con:

* ``manifest.json``
* ``detections.ndjson`` — una detección por línea con pose cilíndrica
* ``frames/frame_NNNNNN.jpg`` + ``frames/frame_NNNNNN.json``

Qué hace
--------
1. Carga las detecciones y las agrupa por proximidad en (x_axial, theta_surface)
   en defectos únicos.
2. Genera un **resumen Markdown** con cabecera, conteos por clase y tabla de
   defectos → ``report/inspection_summary.md``.
3. Genera un **mapa visual** con matplotlib de los defectos sobre el cilindro
   desplegado → ``report/defect_map.png``.
4. Llama a Gemini con (a) el resumen Markdown, (b) el mapa, (c) las imágenes
   representativas de los defectos más confiables, y le pide un informe en
   español → ``report/inspection_report.md``.

Autenticación
-------------
Variable de entorno ``GEMINI_API_KEY``. Modelo por defecto:
``gemini-2.5-flash`` (override con ``--model``).

Con ``--dry-run`` solo se generan el summary y el mapa, sin llamar a la API.
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional


DEFAULT_MODEL = 'gemini-2.5-flash'

CLASS_COLORS = {
    'rust': '#D2691E',          # naranja óxido
    'pitting': '#606060',       # gris oscuro
    'through_hole': '#000000',  # negro
}
DEFAULT_COLOR = '#808080'


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


def _build_text_summary(manifest: dict, clusters: List[Cluster]) -> str:
    """Resumen Markdown que se envía a Gemini."""
    per_class: Dict[str, int] = {}
    for c in clusters:
        per_class[c.class_id] = per_class.get(c.class_id, 0) + 1

    lines = []
    lines.append('# Resumen de misión de inspección')
    lines.append('')
    lines.append(f'- **Run ID**: {manifest.get("run_id", "desconocido")}')
    lines.append(f'- **Inicio**: {manifest.get("started_at_iso", "?")}')
    lines.append(f'- **Última actualización**: {manifest.get("updated_at_iso", "?")}')
    lines.append(f'- **Frames guardados**: {manifest.get("total_frames_saved", 0)}')
    lines.append(f'- **Defectos únicos detectados**: {len(clusters)}')
    lines.append('')

    if per_class:
        lines.append('## Conteo por clase')
        lines.append('')
        for cls, count in sorted(per_class.items()):
            lines.append(f'- **{cls}**: {count}')
        lines.append('')

    if clusters:
        lines.append('## Tabla de defectos')
        lines.append('')
        lines.append('| ID | Clase | x_axial (m) | θ_surface (°) | Observ. | Score máx | Frame representativo |')
        lines.append('|---|---|---|---|---|---|---|')
        for c in sorted(clusters, key=lambda c: c.cluster_id):
            frame = c.representative_frame or '-'
            lines.append(
                f'| {c.cluster_id} | {c.class_id} | '
                f'{c.x_axial_m:.3f} | {c.theta_surface_deg:.2f} | '
                f'{c.observations} | {c.max_score:.3f} | `{frame}` |'
            )
        lines.append('')
    else:
        lines.append('## Tabla de defectos')
        lines.append('')
        lines.append('No se detectaron defectos durante la misión.')
        lines.append('')

    return '\n'.join(lines)


def _build_defect_map(clusters: List[Cluster], output_path: str) -> None:
    """Mapa scatter 2D del cilindro desplegado."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 6))

    if not clusters:
        ax.text(0.5, 0.5, 'Sin defectos detectados',
                ha='center', va='center', transform=ax.transAxes, fontsize=14)
    else:
        by_class: Dict[str, List[Cluster]] = {}
        for c in clusters:
            by_class.setdefault(c.class_id, []).append(c)

        for class_id, group in sorted(by_class.items()):
            xs = [c.x_axial_m for c in group]
            ys = [c.theta_surface_deg for c in group]
            sizes = [60 + c.observations * 25 for c in group]
            color = CLASS_COLORS.get(class_id, DEFAULT_COLOR)
            ax.scatter(xs, ys, c=color, s=sizes, alpha=0.75,
                       edgecolors='black', linewidths=0.6,
                       label=f'{class_id} ({len(group)})')

            for c in group:
                ax.annotate(str(c.cluster_id),
                            (c.x_axial_m, c.theta_surface_deg),
                            textcoords='offset points', xytext=(8, 4),
                            fontsize=8, color='black')

        ax.legend(loc='upper right', fontsize=10, framealpha=0.9)

    ax.set_xlabel('x_axial (m) — posición longitudinal en la torre', fontsize=11)
    ax.set_ylabel('θ_surface (°) — ángulo alrededor del cilindro', fontsize=11)
    ax.set_title('Mapa de defectos — cilindro desplegado', fontsize=13)
    ax.set_ylim(-5, 365)
    ax.set_yticks([0, 90, 180, 270, 360])
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close(fig)


def _build_prompt() -> str:
    return (
        'Eres un ingeniero redactando un informe de inspección interna de una '
        'torre eólica. Un robot inspeccionó el interior del tubo de la torre y '
        'detectó defectos circulares: óxido (rust), picaduras (pitting) y '
        'agujeros pasantes (through_hole).\n\n'
        'A continuación recibes:\n'
        '1. Un resumen Markdown de la misión con métricas y la tabla de defectos.\n'
        '2. Una imagen del mapa visual con la distribución de los defectos '
        'sobre el cilindro desplegado (eje X = posición longitudinal, '
        'eje Y = ángulo en la superficie 0-360°).\n'
        '3. Imágenes representativas de los defectos más confiables.\n\n'
        'Redacta un **informe en español** en formato Markdown con esta '
        'estructura exacta:\n\n'
        '## 1. Resumen ejecutivo\n'
        'Un párrafo con el resultado global de la inspección.\n\n'
        '## 2. Severidad por clase\n'
        'Análisis del riesgo asociado a cada tipo de defecto encontrado.\n\n'
        '## 3. Tabla de defectos\n'
        'Reproduce íntegramente la tabla del resumen.\n\n'
        '## 4. Análisis del mapa de defectos\n'
        'Describe qué se observa en el mapa visual: zonas con concentración, '
        'distribución a lo largo del eje axial, agrupaciones angulares, etc.\n\n'
        '## 5. Hallazgos destacados\n'
        'Selecciona 3-5 defectos críticos (por confianza y clase) y descríbelos '
        'brevemente, indicando posición y referencia al frame representativo.\n\n'
        '## 6. Recomendaciones\n'
        'Acciones de seguimiento sugeridas: revisión manual, ensayos no '
        'destructivos, reparación, monitorización continua, etc.\n\n'
        'Sé conciso, técnico y orientado a un ingeniero de mantenimiento. No '
        'inventes mediciones que no estén en los datos proporcionados.'
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True,
                        help='Directorio de misión '
                             '(p.ej. ~/ROS2_Wind_Tower_Inspection/inspections/run_YYYYMMDD_HHMMSS).')
    parser.add_argument('--output-dir', default=None,
                        help='Dónde escribir el informe. Por defecto <run-dir>/report/.')
    parser.add_argument('--model', default=DEFAULT_MODEL,
                        help=f'Modelo Gemini (default: {DEFAULT_MODEL}).')
    parser.add_argument('--cluster-x-tol-m', type=float, default=0.30)
    parser.add_argument('--cluster-theta-tol-deg', type=float, default=5.0)
    parser.add_argument('--attach-thumbnails', type=int, default=5,
                        help='Nº de imágenes representativas a adjuntar al prompt.')
    parser.add_argument('--dry-run', action='store_true',
                        help='No llamar a Gemini; solo generar summary y mapa.')
    args = parser.parse_args(argv)

    run_dir = os.path.expanduser(args.run_dir)
    manifest_path = os.path.join(run_dir, 'manifest.json')
    detections_path = os.path.join(run_dir, 'detections.ndjson')
    if not os.path.isfile(manifest_path):
        print(f'[report] no encuentro el manifest: {manifest_path}', file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(detections_path):
        print(f'[report] no encuentro detections.ndjson: {detections_path}',
              file=sys.stderr)
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

    output_dir = args.output_dir or os.path.join(run_dir, 'report')
    output_dir = os.path.expanduser(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    summary_text = _build_text_summary(manifest, clusters)
    summary_path = os.path.join(output_dir, 'inspection_summary.md')
    with open(summary_path, 'w', encoding='utf-8') as fh:
        fh.write(summary_text)
    print(f'[report] resumen: {summary_path}')

    map_path = os.path.join(output_dir, 'defect_map.png')
    _build_defect_map(clusters, map_path)
    print(f'[report] mapa:    {map_path}')

    if args.dry_run:
        print('[report] --dry-run activo; no llamo a la API.')
        return

    api_key = os.environ.get('GEMINI_API_KEY')
    if not api_key:
        print('[report] GEMINI_API_KEY no configurada. Resumen y mapa generados; '
              'relanza sin --dry-run cuando tengas la key.', file=sys.stderr)
        sys.exit(2)

    try:
        from google import genai
        from google.genai import types
    except ImportError:
        print('[report] SDK `google-genai` no instalado. Instala con:\n'
              '    pip install google-genai --break-system-packages',
              file=sys.stderr)
        sys.exit(1)

    content_parts: list = [
        _build_prompt(),
        '\n\n--- DATOS DE LA MISIÓN ---\n\n' + summary_text,
        '\n\n--- MAPA VISUAL DE DEFECTOS ---\n',
    ]
    with open(map_path, 'rb') as fh:
        map_bytes = fh.read()
    content_parts.append(types.Part.from_bytes(data=map_bytes, mime_type='image/png'))

    attached = 0
    ordered = sorted(clusters, key=lambda c: -c.max_score)
    for cluster in ordered:
        if attached >= args.attach_thumbnails:
            break
        if not cluster.representative_frame:
            continue
        abs_path = os.path.join(run_dir, cluster.representative_frame)
        if not os.path.isfile(abs_path):
            continue
        with open(abs_path, 'rb') as fh:
            img_bytes = fh.read()
        content_parts.append(
            f'\n\n### Defecto {cluster.cluster_id} ({cluster.class_id}) '
            f'en x_axial={cluster.x_axial_m:.2f} m, '
            f'θ_surface={cluster.theta_surface_deg:.1f}°\n'
        )
        content_parts.append(
            types.Part.from_bytes(data=img_bytes, mime_type='image/jpeg'))
        attached += 1

    print(f'[report] llamando a Gemini ({args.model}) con {attached} thumbnails…')
    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model=args.model,
        contents=content_parts,
    )

    report_text = (response.text or '').strip() or '(respuesta vacía)'
    report_path = os.path.join(output_dir, 'inspection_report.md')
    with open(report_path, 'w', encoding='utf-8') as fh:
        fh.write(report_text + '\n')
    print(f'[report] informe: {report_path}')


if __name__ == '__main__':
    main()
