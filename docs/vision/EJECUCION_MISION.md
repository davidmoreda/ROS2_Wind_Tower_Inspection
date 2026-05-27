# Ejecución de Misión — YOLO Entrenado

Guía de arranque rápido con el modelo `best.pt` ya entrenado.

**Modelo:** `~/ROS2_Wind_Tower_Inspection/ros2_ws/models/best.pt`

---

## Setup (cada terminal nueva)

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
source install/setup.bash
```

`CYCLONEDDS_URI` y otras variables ya están en `~/.bashrc`. `ai-on` **solo** lo necesitas en la terminal del detector (T2). Para `colcon build` y herramientas GUI de ROS (`rqt_*`), **no actives el venv**.

---

## Misión completa — 3 terminales

**T1 — Simulación con mundo de defectos:**
```bash
ros2 launch wind_tower_bringup simulation.launch.py \
    world_file:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world_defects_actors.sdf
```
Esperar: `platform_velocity_controller: Configured and activated`

**T2 — Percepción con YOLO** (única terminal con `ai-on`):
```bash
ai-on
ros2 launch wind_tower_perception perception.launch.py
```
Esperar: `Defect detector ready (backend=yolo ...)` y `Image capture ready (run_id=run_..., dir=...)`. **Apunta el `run_id`** para luego generar el informe.

**T3 — Misión:**
```bash
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

---

## Troubleshooting — Gazebo se abre y se cierra

Si al lanzar T1 (simulación) Gazebo arranca y muere a los segundos, suele ser por procesos residuales y memoria compartida de DDS sin liberar. Limpia antes de relanzar:

```bash
# 1. Mata cualquier proceso residual
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gzserver\|gzclient" 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "parameter_bridge\|robot_state_publisher" 2>/dev/null

# 2. Limpia memoria compartida DDS y caches de Gazebo
rm -f /dev/shm/cdds_* /dev/shm/iceoryx* /dev/shm/gz_* 2>/dev/null
rm -rf ~/.gz/sim/cache /tmp/.gazebo 2>/dev/null

# 3. Verifica que no queda nada vivo
sleep 2
ps aux | grep -E "gz|ros2|gazebo" | grep -v grep
```

Si `ps aux` no devuelve nada, está limpio y puedes relanzar T1. Si devuelve algo, mata esos PIDs con `kill -9 <PID>` y reintenta.

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

Genera con Gemini un informe en español a partir de un `run_*/` de inspección.

La clave API se lee automáticamente de `~/ROS2_Wind_Tower_Inspection/.env` (variable `GEMINI_API_KEY`). También se acepta como env var del shell.

```bash
python3 -m wind_tower_perception.scripts.generate_inspection_report \
    --run-dir ~/ROS2_Wind_Tower_Inspection/inspections/run_YYYYMMDD_HHMMSS \
    --attach-thumbnails 5
```

Genera en `<run-dir>/report/`:

| Archivo | Contenido |
|---|---|
| `inspection_summary.md` | Resumen Markdown con tabla de defectos enviado al modelo |
| `defect_map.png` | Mapa visual con la distribución de defectos sobre el cilindro desplegado |
| `inspection_report.md` | Informe redactado por Gemini |

Opciones útiles:

- `--dry-run` → genera el resumen y el mapa **sin** llamar a la API (útil para iterar tolerancias de clustering sin gastar tokens).
- `--model gemini-2.5-flash` → más rápido y con más calidad (menos llamadas/día gratis).
- `--model gemini-2.5-pro` → modelo más potente, mucho más limitado en cuota gratis.
- `--cluster-x-tol-m 0.30 --cluster-theta-tol-deg 5.0` → tolerancias del clustering de defectos.

Dependencias (una sola vez):
```bash
pip install google-genai matplotlib --break-system-packages
```

---

## Clases del modelo

| ID | Clase | Color en simulación |
|---|---|---|
| 0 | `pitting` | Gris oscuro |
| 1 | `rust` | Naranja óxido |
| 2 | `through_hole` | Negro |
