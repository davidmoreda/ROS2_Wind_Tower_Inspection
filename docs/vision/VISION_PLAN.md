# Plan — Pipeline de Visión YOLO
**Branch**: `feature/vision-yolo` | **Paquete**: `wind_tower_perception`

---

## Estado

| Componente | Archivo | Estado |
|---|---|---|
| Mundo sintético con defectos | `worlds/wind_tower_world_synthetic.sdf` | ✅ |
| Ground truth de posiciones 3D | `worlds/defects_ground_truth.yaml` | ✅ |
| Teleport con pause/resume physics | `auto_dataset_node.py` | ✅ |
| Aiming del brazo con MoveIt IK | `auto_dataset_node.py` | ✅ |
| Autolabelado automático de frames | `synthetic_capture_node.py` | ✅ |
| Detector HoughCircles + YOLO | `detector_node.py` | ✅ |
| Proyección pixel → (x_axial, θ) | `defect_mapper_node.py` | ✅ |
| Script de entrenamiento YOLO11 | `scripts/train_yolo.py` | ✅ |
| Informe post-misión con Claude API | `scripts/generate_inspection_report.py` | ✅ |
| Overlay posición en imagen anotada | `defect_mapper_node.py` | ❌ pendiente |

---

## Fase 1 — Compilar y preparar

> Bloque de setup: ejecutar en **cada terminal nueva** antes de cualquier comando ROS.

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
ai-on                                   # GPU + ROS 2 Jazzy + venv
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
```

**Primera vez o tras cambios en el código:**
```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
colcon build --packages-select wind_tower_perception
source install/setup.bash
```

---

## Fase 2 — Captura automática del dataset

**Cómo funciona:** `auto_dataset_node` recorre cada defecto del ground truth. Por cada uno: pausa la física de Gazebo → teleporta el robot al Y del defecto → reanuda la física → usa MoveIt IK para apuntar el brazo UR5e directamente al defecto. `synthetic_capture_node` detecta que el defecto es visible y guarda frame + label YOLO. Sin mando, sin etiquetado manual.

**4 terminales** (bloque de setup al inicio de cada una):

**T1 — Simulación con mundo sintético:**
```bash
ros2 launch wind_tower_bringup simulation.launch.py \
    world_file:=$(ros2 pkg prefix wind_tower_simulation)/share/wind_tower_simulation/worlds/wind_tower_world_synthetic.sdf
```
Esperar: `platform_velocity_controller: Configured and activated`

**T2 — MoveIt (necesario para IK del brazo):**
```bash
ros2 launch wind_tower_arm_control move_group.launch.py
```
Esperar: `MoveGroup context initialization complete`

**T3 — Autolabelado:**
```bash
ros2 launch wind_tower_perception perception.launch.py \
    use_synthetic_capture:=true \
    ground_truth_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/defects_ground_truth.yaml \
    dataset_output_dir:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset \
    use_detector:=false \
    use_image_capture:=false \
    use_defect_mapper:=false
```

**T4 — Teleport + aiming automático:**
```bash
ros2 run wind_tower_perception auto_dataset \
    --ros-args \
    -p ground_truth_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/defects_ground_truth.yaml \
    -p robot_name:=robot/robot
```

> **Nota:** El modelo de Gazebo se llama `robot/robot`, no `robot`, porque Clearpath deriva
> el nombre de `namespace: robot` en `robot.yaml`. Si Gazebo muestra
> `Unable to update the pose for entity id:[0], name[robot]`, se está usando el nombre antiguo.

El nodo recorre los ~100 positions en ~3–4 min y termina solo. Progreso en el log:
```
[1/100] defect 0 (pitting)  offset=-1.5m  robot_y=-9.70
...
[100/100] defect N (...)  offset=+1.5m  robot_y=...
Auto-capture complete — 100 positions visited.
```

### Diagnóstico de `synthetic_capture_node`

Si el dataset queda vacío, relanzar T3 con `synthetic_save_empty_frames:=true`:
```bash
ros2 launch wind_tower_perception perception.launch.py \
    use_synthetic_capture:=true \
    ground_truth_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/defects_ground_truth.yaml \
    dataset_output_dir:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset \
    synthetic_save_empty_frames:=true \
    use_detector:=false \
    use_image_capture:=false \
    use_defect_mapper:=false
```

`synthetic_capture` imprimirá cada 5 s contadores `images`, `camera_info`, `tf_failures`, `no_labels` y `saved`. Interpretación rápida:

- `images=0`: no llega `/inspection/camera/image_raw`.
- `camera_info=0`: no llega `/inspection/camera/camera_info`.
- `camera_info=0` pero `intrinsics_fallback=1`: OK para captura sintética; se usan intrínsecos calculados desde el FOV del URDF.
- `tf_failures>0`: falta TF hasta el `camera_frame`. Forzar con `synthetic_tf_fallback_camera_frame:=<frame_tf>`.
- `no_labels>0` con `saved=0`: la cámara funciona pero ningún defecto entra limpio en el FOV. El nodo descarta cajas demasiado recortadas, muy pequeñas o sin contraste visual.

### Dataset resultante

```
~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset/
  images/train/*.jpg     ← fotos automáticas
  images/val/*.jpg
  labels/train/*.txt     ← labels YOLO auto-generados
  labels/val/*.txt
```

---

## Fase 3 — Entrenamiento

```bash
cp ~/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_perception/config/dataset.yaml \
   ~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset/

cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source /opt/ai-venv/bin/activate
source install/setup.bash
ros2 run wind_tower_perception train_yolo \
    --dataset ~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset/dataset.yaml \
    --weights yolo11s.pt \
    --epochs 80 \
    --imgsz 640 \
    --device cpu          # cambiar a "0" si hay GPU disponible
```
Pesos resultantes: `~/wind_tower_runs/wind_tower_defects/weights/best.pt`

---

## Fase 4 — Detección con posición en imagen ❌ pendiente

**Qué falta:** modificar `defect_mapper_node.py` para suscribirse a `/inspection/detections/image_annotated`, superponer `x=Xm θ=Y°` sobre cada bbox y publicar `/inspection/defects/image_position`.

**Lanzamiento misión real con YOLO (3 terminales):**

**T1 — Simulación:**
```bash
ros2 launch wind_tower_bringup simulation.launch.py
```

**T2 — Percepción con YOLO:**
```bash
ros2 launch wind_tower_perception perception.launch.py \
    backend:=yolo \
    yolo_model_path:=$HOME/wind_tower_runs/wind_tower_defects/weights/best.pt
```

**T3 — Misión:**
```bash
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

---

## Fase 5 — Informe post-misión

```bash
export ANTHROPIC_API_KEY="sk-ant-..."
python3 -m wind_tower_perception.scripts.generate_inspection_report \
    --run-dir ~/wind_tower_inspections/run_YYYYMMDD_HHMMSS \
    --model claude-opus-4-7 \
    --attach-thumbnails 5
```
`--dry-run` genera solo el JSON sin llamar a la API.
