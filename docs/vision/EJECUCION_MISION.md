# Ejecución de Misión — YOLO Entrenado

Guía de arranque rápido con el modelo `best.pt` ya entrenado.

**Modelo:** `~/ROS2_Wind_Tower_Inspection/ros2_ws/models/best.pt`

---

## Setup (cada terminal nueva)

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
ai-on
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
```

---

## Misión completa — 3 terminales

**T1 — Simulación:**
```bash
ros2 launch wind_tower_bringup simulation.launch.py
```
Esperar: `platform_velocity_controller: Configured and activated`

**T2 — Percepción con YOLO:**
```bash
ros2 launch wind_tower_perception perception.launch.py
```
Esperar: `Defect detector ready (backend=yolo ...)`

**T3 — Misión:**
```bash
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

---

## Topics útiles para monitorear

| Topic | Contenido |
|---|---|
| `/inspection/defects/image_position` | Imagen con bboxes y posición `x=Xm θ=Y°` |
| `/inspection/detections/image_annotated` | Imagen con bboxes del detector |
| `/inspection/defects/cylindrical` | JSON con coordenadas cilíndricas por frame |
| `/inspection/defects/cumulative` | JSON con clusters acumulados de la misión |

Ver imagen en tiempo real:
```bash
ros2 run rqt_image_view rqt_image_view /inspection/defects/image_position
```

Ver defectos detectados:
```bash
ros2 topic echo /inspection/defects/cumulative
```

---

## Informe post-misión

```bash
export ANTHROPIC_API_KEY="sk-ant-..."
python3 -m wind_tower_perception.scripts.generate_inspection_report \
    --run-dir ~/ROS2_Wind_Tower_Inspection/inspections/run_YYYYMMDD_HHMMSS \
    --model claude-opus-4-7 \
    --attach-thumbnails 5
```

`--dry-run` genera el JSON sin llamar a la API.

---

## Clases del modelo

| ID | Clase | Color en simulación |
|---|---|---|
| 0 | `pitting` | Gris oscuro |
| 1 | `rust` | Naranja óxido |
| 2 | `through_hole` | Negro |
