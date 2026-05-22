# Plan de implementación — Pipeline de Visión YOLO
**Branch**: `feature/vision-yolo` | **Responsable**: Javier | **Paquete**: `wind_tower_perception`

---

## Cómo funciona el etiquetado

Se capturan **fotos individuales** (JPEG), no vídeo. El nodo `auto_dataset_node` teleporta el robot automáticamente a posiciones calculadas cerca de cada defecto. Para cada posición, `synthetic_capture_node` detecta que el defecto es visible en cámara (vía geometría + TF) y guarda la foto + el label `.txt` YOLO automáticamente. **No hay que etiquetar nada a mano.**

```
auto_dataset_node                   synthetic_capture_node
  ├─ lee ground_truth.yaml             ├─ proyecta defectos en imagen
  ├─ calcula posiciones robot           ├─ si visible → guarda foto
  └─ teleporta robot vía gz service     └─ escribe label YOLO automático
                                             "0 0.52 0.38 0.08 0.09"
                                              ↑              ↑
                                          clase (rust)   bbox normalizada
```

Con 20 defectos × 5 offsets axiales = **100 posiciones**. A ~3 frames/posición → ~300 imágenes etiquetadas sin tocar el mando.

---

## Estado actual

| Componente | Archivo | Estado |
|---|---|---|
| Mundo sintético con defectos (rust, pitting, through_hole) | `worlds/wind_tower_world_synthetic.sdf` | ✅ Hecho |
| Ground truth de posiciones 3D de defectos | `worlds/defects_ground_truth.yaml` | ✅ Hecho |
| Teleport automático del robot por cada defecto | `auto_dataset_node.py` | ✅ Código listo |
| Autolabelado automático de frames | `synthetic_capture_node.py` | ✅ Código listo |
| Detector con backends HoughCircles y YOLO | `detector_node.py` | ✅ Código listo |
| Proyección pixel → coordenadas cilíndricas | `defect_mapper_node.py` | ✅ Código listo |
| Guardado de frames durante inspección real | `image_capture_node.py` | ✅ Código listo |
| Script de entrenamiento YOLOv8 | `scripts/train_yolo.py` | ✅ Código listo |
| Generador de informe post-misión con Claude API | `scripts/generate_inspection_report.py` | ✅ Código listo |
| Posición cilíndrica del defecto visible en imagen | `defect_mapper_node.py` | ❌ Por implementar |

---

## Fase 1 — Setup y compilación

```bash
# En cada terminal nueva, siempre primero:
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
ai-on
source /opt/ai-venv/bin/activate
source /opt/ros/jazzy/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
```

> `ai-on` exporta `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, `GALLIUM_DRIVER=d3d12`, sourcea ROS 2 Jazzy y activa el entorno del curso. Para YOLO usamos explícitamente `/opt/ai-venv`, porque ahí está instalado `ultralytics`.

```bash
# Comprobar dependencias Python de visión
python3 -c "import ultralytics, torch, cv2, rclpy; print('vision deps ok')"
```

Si falla `ultralytics`, no reinstalar a ciegas sobre `/opt/ai-venv`: puede fallar
por permisos. Primero comprobar:

```bash
/opt/ai-venv/bin/python3 -c "import ultralytics; print(ultralytics.__version__)"
```

Si el paquete existe ahí, continuar con la compilación usando `python3 -m colcon`.

```bash
# Compilar el paquete de percepción con el Python del venv.
# Esto hace que los ejecutables instalados apunten a /opt/ai-venv/bin/python3.
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
python3 -m colcon build --packages-select \
    wind_tower_perception
source install/setup.bash
```

Para compilar también el resto del MVP:

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
python3 -m colcon build --packages-select \
    wind_tower_simulation \
    wind_tower_description \
    wind_tower_bringup \
    wind_tower_perception \
    wind_tower_inspection_behaviour

source install/setup.bash
```

Si es la primera vez o hay dependencias pendientes:
```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
python3 -m colcon build --packages-up-to wind_tower_perception
source install/setup.bash
```

Verificación final:
```bash
head -1 install/wind_tower_perception/lib/wind_tower_perception/detector
ros2 run wind_tower_perception train_yolo --help
```

La primera línea debe ser `#!/opt/ai-venv/bin/python3`.

---

## Fase 2 — Captura automática del dataset

Necesitas **3 terminales**. En cada una ejecutar primero:

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
ai-on
source /opt/ai-venv/bin/activate
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
```

No se necesita el DualSense. No se necesita `inspection.launch.py`.

### Terminal 1 — Simulación con mundo sintético

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 launch wind_tower_bringup simulation.launch.py \
    world_file:=$(ros2 pkg prefix wind_tower_simulation)/share/wind_tower_simulation/worlds/wind_tower_world_synthetic.sdf
```

Esperar a ver:
```
[INFO] platform_velocity_controller: Configured and activated
```

### Terminal 2 — Nodo de autolabelado

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 launch wind_tower_perception perception.launch.py \
    use_synthetic_capture:=true \
    ground_truth_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/defects_ground_truth.yaml \
    use_detector:=false \
    use_image_capture:=false \
    use_defect_mapper:=false
```

### Terminal 3 — Teleport automático por defectos

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 run wind_tower_perception auto_dataset \
    --ros-args \
    -p ground_truth_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_simulation/worlds/defects_ground_truth.yaml \
    -p robot_name:=robot/robot
```

El modelo de Gazebo se llama `robot/robot`, no `robot`, porque Clearpath deriva
el nombre de `namespace: robot` en `robot.yaml`. Si Gazebo muestra
`Unable to update the pose for entity id:[0], name[robot]`, se está usando el
nombre antiguo.

El nodo recorre los ~100 positions en ~3–4 min y termina solo. Puedes ver el progreso en el log:
```
[1/100] defect 0 (pitting)  y_defect=-8.20  offset=-2.0  → robot_y=-10.20
[2/100] defect 0 (pitting)  y_defect=-8.20  offset=-1.0  → robot_y=-9.20
...
[100/100] Auto-capture complete — 100 positions visited.
```

### Dataset resultante

```
~/wind_tower_dataset/
  images/train/*.jpg     ← fotos automáticas
  images/val/*.jpg
  labels/train/*.txt     ← labels YOLO auto-generados
  labels/val/*.txt
```

---

## Fase 3 — Entrenamiento YOLOv8

```bash
# Copiar el descriptor del dataset al directorio generado
cp ~/ROS2_Wind_Tower_Inspection/ros2_ws/src/wind_tower_perception/config/dataset.yaml \
   ~/wind_tower_dataset/

# Entrenar
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source /opt/ai-venv/bin/activate
source install/setup.bash
ros2 run wind_tower_perception train_yolo \
    --dataset ~/wind_tower_dataset/dataset.yaml \
    --weights yolov8n.pt \
    --epochs 80 \
    --imgsz 640 \
    --device cpu          # cambiar a "0" si hay GPU disponible
```

Pesos entrenados en:
```
~/wind_tower_runs/wind_tower_defects/weights/best.pt
```

### Criterio de calidad mínima
- `mAP50 > 0.6` en validación para continuar
- Si no converge: añadir más offsets en `auto_dataset_node` (`axial_offsets_m`) para más imágenes, o cambiar `--weights yolov8s.pt`

---

## Fase 4 — Detección con posición visible en imagen

### Qué falta por implementar

El `detector_node` dibuja cajas con clase + score pero no conoce las coordenadas 3D. El `defect_mapper_node` calcula `(x_axial_m, θ_surface_deg)` pero no retoca la imagen.

**Solución**: modificar `defect_mapper_node.py` para suscribirse también a `/inspection/detections/image_annotated`, sincronizar imagen + detecciones por timestamp, superponer el texto de posición y publicar `/inspection/defects/image_position`.

**Resultado visual en RViz** por cada detección:
```
┌──────────────────┐
│  rust   0.87     │
│  x=3.2m  θ=47°   │
└──────────────────┘
```

### Lanzamiento con YOLO (misión real — sí usa inspection.launch.py)

**Terminal 1 — Simulación** (mundo normal, sin defectos sintéticos):
```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 launch wind_tower_bringup simulation.launch.py
```

**Terminal 2 — Percepción con YOLO**:
```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source /opt/ai-venv/bin/activate
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 launch wind_tower_perception perception.launch.py \
    backend:=yolo \
    yolo_model_path:=$HOME/wind_tower_runs/wind_tower_defects/weights/best.pt
```

**Terminal 3 — Misión de inspección**:
```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
source install/setup.bash
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

En RViz añadir el topic `/inspection/defects/image_position` para ver la imagen con posición superpuesta.

---

## Fase 5 — Informe post-misión con LLM

Tras una misión real con percepción activa, `image_capture_node` habrá guardado los frames en `~/wind_tower_inspections/run_YYYYMMDD_HHMMSS/`.

```bash
export ANTHROPIC_API_KEY="sk-ant-..."

cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && ai-on
python3 -m wind_tower_perception.scripts.generate_inspection_report \
    --run-dir ~/wind_tower_inspections/run_YYYYMMDD_HHMMSS \
    --model claude-opus-4-7 \
    --attach-thumbnails 5
```

Usar `--dry-run` para generar solo el JSON sin gastar API:
```bash
python3 -m wind_tower_perception.scripts.generate_inspection_report \
    --run-dir ~/wind_tower_inspections/run_YYYYMMDD_HHMMSS \
    --dry-run
```

### Salidas
- `report/inspection_summary.json` — resumen determinístico (clusters, posiciones, contadores)
- `report/inspection_report.md` — informe redactado por Claude con tabla de defectos, hallazgos y recomendaciones

---

## Resumen de trabajo pendiente

| Tarea | Tipo | Estado |
|---|---|---|
| `defects_ground_truth.yaml` | Fichero | ✅ Hecho |
| `auto_dataset_node.py` | Código | ✅ Hecho |
| Compilación con `wind_tower_perception` | Ejecución | ⏳ Pendiente |
| Captura automática del dataset (~4 min) | Ejecución | ⏳ Pendiente |
| Entrenamiento YOLO | Ejecución (variable CPU/GPU) | ⏳ Pendiente |
| Overlay de posición en `defect_mapper_node.py` | Código (~60 líneas) | ⏳ Pendiente |
| Misión real + informe con LLM | Ejecución (20–30 min) | ⏳ Pendiente |
