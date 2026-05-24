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

**Cómo funciona:** `auto_dataset_node` recorre cada defecto del ground truth. Por cada uno: pausa la física de Gazebo → teleporta el robot al Y del defecto → reanuda la física → usa MoveIt IK para apuntar el brazo UR5e a la cámara directamente al defecto. `synthetic_capture_node` detecta que el defecto es visible y guarda frame + label YOLO. Sin mando, sin etiquetado manual.

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
    use_detector:=false \
    use_image_capture:=false \
    use_defect_mapper:=false
```

**T4 — Teleport + aiming automático:**
```bash
ros2 run wind_tower_perception auto_dataset \
    --ros-args \
    -p ground_truth_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/defects_ground_truth.yaml
```
El nodo recorre todas las posiciones y termina solo. Dataset en `~/wind_tower_dataset/`.

---

## Fase 3 — Entrenamiento

```bash
cp ~/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_perception/config/dataset.yaml \
   ~/wind_tower_dataset/

python3 -m wind_tower_perception.scripts.train_yolo \
    --dataset ~/wind_tower_dataset/dataset.yaml \
    --weights yolo11s.pt \
    --epochs 80 --imgsz 640 --device cpu
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
