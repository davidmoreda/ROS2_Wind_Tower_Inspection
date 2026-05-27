# PERCEPTION_PIPELINE — Detección de defectos circulares + informe LLM

> Estado del paquete `wind_tower_perception`: **PROTOTIPO INTEGRADO**.
> Detector clásico (HoughCircles) activo desde el día 0; backend YOLO listo a la espera de pesos entrenados; pipeline de dataset sintético y generador de informe ya operativos.

---

## 1. Objetivo

Detectar defectos circulares (óxido, picaduras, agujeros pasantes) sobre la pared interior del tubo, proyectarlos a coordenadas cilíndricas `(x_axial, θ_surface)` y agregarlos para producir un informe de inspección en Markdown generado por un LLM.

## 2. Diagrama

```text
[Gazebo cámara]
   └─► /inspection/camera/image_raw ─┐
                                     ▼
                            ┌────────────────┐
                            │ defect_detector│  (HoughCircles | YOLO)
                            └───────┬────────┘
                                    │
            /inspection/detections/raw  (vision_msgs/Detection2DArray)
            /inspection/detections/text (std_msgs/String JSON)
            /inspection/detections/image_annotated
                                    │
                ┌───────────────────┼─────────────────────────────┐
                ▼                   ▼                             ▼
       ┌───────────────┐   ┌────────────────┐         ┌────────────────────┐
       │ image_capture │   │ defect_mapper  │         │  RViz / debug      │
       │  (sidecars +  │   │ (TF + ray-cyl  │         │  /inspection/      │
       │   ndjson)     │   │  intersección) │         │  detections/       │
       └───────┬───────┘   └────────┬───────┘         │  image_annotated   │
               │                    │                 └────────────────────┘
   ~/wind_tower_inspections/  /inspection/defects/{cylindrical,cumulative}
        run_*/                       (JSON acumulado clustered)
        frames/*.jpg + *.json
        detections.ndjson
        manifest.json

   ↓ offline
   python3 -m wind_tower_perception.scripts.generate_inspection_report
        --run-dir ~/wind_tower_inspections/run_*
   ↓ (Claude API)
   report/inspection_report.md
   report/inspection_summary.json
```

## 3. Nodos

| Nodo | Función | Backend | Salida principal |
|---|---|---|---|
| `defect_detector` | Detecta círculos en cada frame de la cámara | HoughCircles (default) o `ultralytics.YOLO` | `Detection2DArray` + JSON + imagen anotada |
| `image_capture` | Guarda frames + metadatos cilíndricos + ndjson de detecciones | I/O Python puro | Directorio `run_*/` |
| `defect_mapper` | Back-proyección píxel → mundo → cilindro; clustering por proximidad | `tf2_ros` + intersección rayo-cilindro analítica | JSON con `(x_axial, θ_surface)` por detección y por cluster |
| `synthetic_capture` (opcional) | Autolabela un dataset YOLO mientras el robot conduce en un mundo sintético | Proyección 3D conocida → bbox 2D | `images/{train,val}/*.jpg` + `labels/{train,val}/*.txt` |

## 4. Geometría

El cilindro vive en el frame `world` (publicado como estático por `perception.launch.py` a partir del spawn pose). Defaults:

- Eje del cilindro: dirección **+Y mundo**, atraviesa `(0, 0, 4)`.
- Radio interior: **3.925 m** (parametrizable, debe casar con el STL real).
- Longitud útil: `world_y ∈ [-15, +15]`.

Para cada detección con centro de bbox en píxel `(u, v)`:

1. `ray_cam = K^{-1} · (u, v, 1)` en el frame óptico de la cámara.
2. `ray_world = R_{world←cam} · ray_cam`, `origin_world = t_{world←cam}`.
3. Resolver la cuadrática `||(origin + t·dir)_⊥||² = R²` con `(·)_⊥` proyectado al plano normal al eje del tubo.
4. Punto de intersección → `x_axial = P.y`, `θ_world = atan2(P.x, 4 - P.z)`.
5. **Posición sobre la superficie rotante**: `θ_surface = (θ_world + θ_tube) mod 360°`. Invariante en el tiempo aunque el virador esté girando.

## 5. Convención angular (alineada con `cylindrical_map_node`)

- `θ = 0°` ↔ generatriz inferior del tubo (donde el robot trabaja).
- `θ = 90°` ↔ lateral hacia world +X.
- `θ = 180°` ↔ generatriz superior.
- `θ = 270°` ↔ lateral hacia world -X.

`cylindrical_map_node` publica `theta_surface_deg = theta_tube_deg + lane_zero_offset` para la posición del robot (siempre en θ_world≈0). El defect_mapper usa la misma convención para la posición proyectada del defecto.

## 6. Backends de detección

### 6.1 HoughCircles (default, sin entrenamiento)

Sirve como baseline desde la primera ejecución. Parámetros relevantes en `perception_params.yaml` (`defect_detector.hough.*`):

- `min_radius_px`, `max_radius_px`: rango de tamaños esperados en píxeles.
- `min_dist_px`: separación mínima entre círculos.
- `accumulator_threshold`: cuanto más bajo, más detecciones (también más falsos positivos).
- `score_constant`: confianza nominal que se asigna (Hough no produce un score probabilístico real).

Limitaciones conocidas: degrada con texturas marcadas, con iluminación irregular y con defectos elípticos por perspectiva. Está pensado **solo** como puente hasta tener pesos YOLO entrenados.

### 6.2 YOLOv8 (preferido cuando hay pesos)

Activar con:

```bash
ros2 launch wind_tower_perception perception.launch.py \
    backend:=yolo yolo_model_path:=/ruta/a/best.pt
```

Si el archivo no existe o `ultralytics` no está instalado, el nodo cae automáticamente a HoughCircles y emite un warning. Esto significa que `perception.launch.py` nunca se queda sin detector.

## 7. Generación del dataset sintético

### 7.1 Conceptualmente

- Se cargan defectos como esferas con coloración tipo óxido / negro / gris oscuro, **fijos en el mundo** (no hijos del tubo), insertados como `<model>` adicionales dentro del `<world>` del SDF.
- Durante la captura el virador permanece en `θ_tube = 0` (la superficie etiquetada coincide con la mundial). El robot conduce en modo manual o autónomo y la cámara captura imágenes.
- Para cada frame, conocemos la pose mundial de cada defecto y la pose mundial de la cámara (vía TF). Proyectamos cada defecto a píxeles, calculamos el radio aparente con `r_px ≈ fx · tan(asin(r_m / z))` y construimos un bbox cuadrado.
- Si el bbox cae dentro de la imagen y es suficientemente grande, escribimos un label YOLO; si no, se descarta esa instancia.

### 7.2 Pipeline operativo (paso a paso)

```bash
# 1) Generar el mundo sintético + ground truth
python3 -m wind_tower_perception.scripts.generate_synthetic_world \
    --base-world  ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world.sdf \
    --output-world ~/wind_tower_synthetic/wind_tower_world_synthetic.sdf \
    --ground-truth ~/wind_tower_synthetic/defects_ground_truth.yaml \
    --num-defects 100 --seed 42

# 2) Lanzar simulación apuntando al SDF sintético
#    (sustituir wind_tower_world.sdf en la copia de trabajo o ajustar
#    la variable GZ_SIM_RESOURCE_PATH para servirlo)
ros2 launch wind_tower_bringup simulation.launch.py

# 3) Lanzar perception con synthetic_capture activo
ros2 launch wind_tower_perception perception.launch.py \
    use_synthetic_capture:=true \
    ground_truth_path:=~/wind_tower_synthetic/defects_ground_truth.yaml \
    use_detector:=false \
    use_image_capture:=false \
    use_defect_mapper:=false

# 4) Conducir manualmente el robot (DualSense) por todo el tubo. El nodo
#    irá guardando imágenes etiquetadas en
#    ~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset/.

# 5) Copiar el dataset.yaml a la raíz del dataset
cp ros2_ws/src/wind_tower_perception/config/dataset.yaml \
   ~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset/

# 6) Entrenar
python3 -m wind_tower_perception.scripts.train_yolo \
    --dataset ~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset/dataset.yaml \
    --weights yolov8n.pt --epochs 80 --imgsz 640 --device 0
```

Los pesos finales aparecen en `~/wind_tower_runs/wind_tower_defects/weights/best.pt`.

## 8. Informe de inspección con LLM

Tras una misión:

```bash
export ANTHROPIC_API_KEY="sk-..."
pip install anthropic     # solo la primera vez

python3 -m wind_tower_perception.scripts.generate_inspection_report \
    --run-dir ~/wind_tower_inspections/run_YYYYMMDD_HHMMSS \
    --model claude-opus-4-7 \
    --attach-thumbnails 5
```

Salida:

- `report/inspection_summary.json` — agregado determinístico (clusters, posiciones, contadores). Se escribe siempre, también con `--dry-run`.
- `report/inspection_report.md` — informe redactado por Claude basado en el JSON anterior y hasta 5 miniaturas representativas. Estructura: ejecutivo · clases · tabla · hallazgos notables · recomendaciones.

`--dry-run` produce solo el JSON (útil para regenerar agregados sin pagar API).

## 9. Tópicos relevantes

| Tópico | Tipo | Publicado por | Consumido por |
|---|---|---|---|
| `/inspection/camera/image_raw` | `sensor_msgs/Image` | bridge Gazebo | detector, image_capture, synthetic_capture |
| `/inspection/camera/camera_info` | `sensor_msgs/CameraInfo` | bridge Gazebo | defect_mapper, synthetic_capture |
| `/inspection/detections/raw` | `vision_msgs/Detection2DArray` | detector | defect_mapper |
| `/inspection/detections/text` | `std_msgs/String` (JSON) | detector | image_capture |
| `/inspection/detections/image_annotated` | `sensor_msgs/Image` | detector | RViz |
| `/inspection/defects/cylindrical` | `std_msgs/String` (JSON) | defect_mapper | (consumidores futuros: mapa cilíndrico de defectos) |
| `/inspection/defects/cumulative` | `std_msgs/String` (JSON) | defect_mapper | (consumidores futuros: report node ROS-side) |
| `/inspection/cylindrical_pose` | `std_msgs/String` (JSON) | `cylindrical_map_node` | image_capture (ya consumido), defect_mapper (referencia para θ_surface_robot) |
| `/turner/angle` | `std_msgs/Float64` | `turner_node` | defect_mapper (acopla θ_world ↔ θ_surface) |

## 10. Decisiones que NO se cierran aquí

- **Calidad del dataset sintético**: las esferas son una proxy razonable para "círculo oscuro" pero no capturan textura real de óxido. El modelo sintético deberá fine-tunearse con fotos reales antes de ir a producción.
- **Mensajes custom**: hoy todo viaja como `std_msgs/String` JSON salvo `Detection2DArray`. Si se promueve a producción se sugiere crear `wind_tower_msgs/CylindricalDefect.msg` con `x_axial_m`, `theta_surface_deg`, `class_id`, `score`, `observations`.
- **Rotación del virador durante la captura sintética**: actualmente asumimos `θ_tube = 0`. Si se quiere variar la pose de los defectos, alternativas: regenerar el SDF con otra seed o capturar bajo distintos ángulos del virador, ajustando el ground truth.
- **Cluster radius**: `cluster.x_tol_m=0.30 m` y `cluster.theta_tol_deg=5°` son un punto de partida. Si los defectos reales son más densos, bajar; si la odometría drifta más, subir.
