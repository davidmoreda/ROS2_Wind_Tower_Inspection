# Inspeccion Autonoma de Torres Eolicas

> **Documento fuente para Manus IA.** Genera una presentacion de **~15 minutos** (17 slides, ratio 16:9) a partir del contenido de abajo. Cada slide trae: titulo, contenido para la diapositiva (bullets cortos), recurso visual sugerido y **notas de orador** (lo que se dice en voz). Idioma: español. Tono: tecnico pero claro, defensa de TFM de master.
>
> **Estilo visual sugerido:** identidad Universidad Loyola Andalucia, color principal granate `#9F1D35` con grises de soporte, tipografia sans-serif limpia, mucho espacio en blanco, una imagen grande por slide tecnico.

---

## Metadatos

- **Titulo:** Inspeccion Autonoma de Torres Eolicas
- **Subtitulo:** Husky A200 + UR5e + Nav2 + MoveIt 2 + vision (Hough/YOLO) + Behaviour Tree + voz
- **Institucion:** Universidad Loyola Andalucia — Master en Inteligencia Artificial — Robotica Inteligente
- **Equipo:** 3 integrantes (P1, P2, P3)
- **Stack:** ROS 2 Jazzy · Ubuntu 24.04 (WSL2) · Gazebo Sim 8
- **Repositorio:** github.com/davidmoreda/ROS2_Wind_Tower_Inspection

---

## Slide 1 — Portada

**Contenido**
- Titulo + subtitulo del proyecto.
- Nombres del equipo + universidad + master.
- Logo de Loyola.

**Visual:** logo Loyola centrado, fondo limpio con franja granate.

**Notas de orador:** "Buenas, presentamos nuestro proyecto de inspeccion autonoma de torres eolicas: un robot movil con brazo que entra en la torre, la inspecciona y genera un informe, todo orquestado en ROS 2."

---

## Slide 2 — Indice

**Contenido (dos columnas):**
1. Contexto y objetivo
2. Arquitectura del sistema
3. Plataforma robotica y simulacion
4. Mundo dinamico
5. Navegacion autonoma (Nav2)
6. Manipulacion (MoveIt 2)
7. Percepcion de defectos
8. Mision, voz e interaccion
9. Informe automatico
10. Demo + metricas + conclusiones

**Visual:** lista numerada a dos columnas.

**Notas de orador:** "Recorreremos la arquitectura, cada subsistema, y terminaremos con la demo completa y las metricas."

---

## Slide 3 — Contexto y objetivo

**Contenido**
- El **mastil** de una torre eolica es un **tubo de acero de unos 30 m** que necesita **inspeccion periodica** de su pared en busca de daños (grietas, corrosion, defectos de soldadura).
- Hoy esa inspeccion la hace **una persona a mano**: se mete en el tubo y revisa la pared visualmente. Es **lento, caro y arriesgado**.
- **Nuestra propuesta:** que esa persona **ya no haga falta** — un **robot autonomo con brazo manipulador** entra en el tubo e inspecciona la pared interior en su lugar.
- **Objetivo:** **automatizar por completo** esa inspeccion integrando navegacion, manipulacion, percepcion y comportamiento en un unico sistema autonomo (simulado en Gazebo).

**Visual:** `contexto_torre.png` — el tubo/mastil de la torre eolica con el robot entrando a inspeccionar la pared interior.

**Notas de orador:** "El contexto: el mastil de una torre eolica es basicamente un tubo de acero de unos 30 metros. Hoy, para buscar daños —grietas, corrosion, fallos de soldadura— tiene que entrar una persona y recorrer esos 30 metros revisando la pared a mano; es lento, caro y peligroso. Nuestro objetivo es que esa persona no haga falta: que un robot autonomo entre en el tubo y haga esa inspeccion por si solo."

---

## Slide 4 — Arquitectura general

**Contenido**
- Sistema modular en **6 paquetes ROS 2 propios**:
  - `wind_tower_bringup` — orquestacion: launch, config Nav2/SLAM/EKF/AMCL, nodos pegamento.
  - `wind_tower_description` — modelo URDF (tramo de torre + camara de inspeccion).
  - `wind_tower_simulation` — mundos SDF + modelos 3D.
  - `wind_tower_arm_control` — brazo UR5e con MoveIt 2.
  - `wind_tower_perception` — vision y deteccion de defectos.
  - `wind_tower_inspection_behaviour` — el "cerebro": mision, voz, comportamiento.
- Cuatro capas: **simulacion** (Gazebo) → **estimacion** (EKF + AMCL) → **actuacion** (Nav2 + MoveIt 2) → **logica** (BT + voz + percepcion).

**Visual:** `arquitectura.png` — diagrama de bloques con los 6 paquetes y las flechas de datos (topics/acciones).

**Notas de orador:** "Todo esta separado por subsistema: orquestacion, modelo, mundo, brazo, vision y cerebro. Cada uno es un paquete ROS 2, y se comunican por topics y acciones. Esto nos permitio repartir el trabajo entre los tres."

---

## Slide 5 — Plataforma robotica y simulacion

**Contenido**
- **Husky A200** de Clearpath — base diferencial.
- **UR5e** montado encima — brazo de 6 DOF.
- Sensores: **IMU**, **LIDAR 2D Hokuyo**, **LIDAR 3D Velodyne VLP-16**, **camara de inspeccion en el TCP** (con luz).
- Configuracion declarativa Clearpath en `robot.yaml`.
- Simulador: **Gazebo Sim 8**. Escena: nave industrial con un tramo de torre eolica y el tubo con virador.

**Visual:** `robot_gazebo.png` — el Husky+UR5e en Gazebo dentro de la nave.

**Notas de orador:** "La plataforma es un Husky con un UR5e encima. Como sensores tenemos IMU, un lidar 2D, un Velodyne de 16 planos y una camara en la punta del brazo para inspeccionar de cerca. Todo el robot se genera declarativamente con el stack de Clearpath."

---

## Slide 6 — Mundo dinamico: defectos + personas

**Contenido**
- Tres mundos SDF:
  - **principal** (la nave + el tubo de la torre),
  - **`defects_actors`** — con **defectos** sembrados y **personas que caminan** (actores animados),
  - **`synthetic`** — para generar el dataset de vision.
- Las personas son **obstaculos dinamicos**: capsulas de colision que se sincronizan con la animacion (`people_collision_sync`).
- `defects_ground_truth.yaml` guarda la "verdad" de donde estan los defectos para validar la deteccion.

**Visual:** `world_actors.png` — el mundo con personas caminando cerca del robot.

**Notas de orador:** "Para hacerlo realista metimos personas que caminan por la nave. No son decorado: tienen colision sincronizada, asi que el robot tiene que esquivarlas de verdad. Y sembramos defectos con su ground truth para poder medir si la vision acierta."

---

## Slide 7 — Navegacion autonoma (Nav2)

**Contenido**
- **SLAM Toolbox** (online_async) para mapear el entorno.
- **EKF** (`robot_localization`) fusiona IMU + odometria → TF `odom → base_link`.
- **AMCL** localiza globalmente sobre el mapa → TF `map → odom`.
- **Nav2**: planner NavFn, controller **RPP** (Regulated Pure Pursuit), BT navigator y recovery.
- `obstacle_cloud_filter`: **filtra el suelo** de la nube del Velodyne para el costmap → el robot **esquiva las personas** que caminan.

**Visual:** `nav2_rviz.png` — RViz con mapa, costmaps, plan global/local y la nube filtrada.

**Notas de orador:** "Mapeamos con SLAM Toolbox y luego localizamos con EKF + AMCL. La navegacion es Nav2 con el controlador Regulated Pure Pursuit. Lo importante: filtramos el suelo de la nube 3D del Velodyne para que el costmap solo vea obstaculos reales, y asi el robot esquiva a las personas en movimiento."

---

## Slide 8 — Tuning del controller (rampa de 13°)

**Contenido**
- **Problema:** el robot tiene que subir una rampa de 13° para acceder al tubo.
- **Hipotesis 1 (descartada):** mas velocidad y aceleracion para vencer la pendiente → el Husky **patinaba** en los giros y AMCL perdia coherencia.
- **Hipotesis 2 (actual):** aceleraciones **suaves** para no perder traccion.

| Parametro | Antes | Despues |
| --------- | ----- | ------- |
| `lookahead_dist` | 0.9 | **1.2** |
| `max_angular_accel` | 2.0 | **1.2** |
| `smoother.max_accel` | 2.5 | **0.75** |
| `rotate_to_heading_min_angle` | 45° | **60°** |

**Visual:** tabla embebida + (opcional) foto del robot en la rampa.

**Notas de orador:** "Nos costo la rampa. Al principio pensamos en darle mas potencia, pero patinaba y perdiamos la localizacion. La solucion fue al reves: suavizar aceleraciones y aumentar el lookahead para que la trayectoria fuera mas estable. Estos cuatro parametros son los que mas cambiaron."

---

## Slide 9 — Manipulacion con MoveIt 2 (barrido 360°)

**Contenido**
- Paquete propio `wind_tower_arm_control` con MoveIt 2 sobre el **UR5e**.
- URDF/Xacro de Clearpath; SRDF con grupo `manipulator`.
- Planificacion **articular** con OMPL (**RRTConnect**) y **cartesiana** con **Pilz LIN**.
- `arm_inspection_node` ejecuta un **barrido 360°**: home → **7 waypoints a 45°** → home, via `ros2_control` (`JointTrajectory`).
- Al terminar publica **`/arm/inspection_ready`** para que la mision avance.

**Visual:** `moveit_rviz.png` — el UR5e desplegado dentro del tubo haciendo el barrido.

**Notas de orador:** "Una vez dentro del tubo, el brazo hace un barrido de 360 grados: parte de home, pasa por siete posiciones cada 45 grados y vuelve. Usamos OMPL para los movimientos articulares y Pilz para los lineales. Cuando acaba, avisa con un topic para que la mision siga."

---

## Slide 10 — Percepcion de defectos (Hough / YOLO)

**Contenido**
- `detector_node` con **dos backends**:
  - **HoughCircles** de OpenCV (por defecto) — detecta los defectos circulares.
  - **YOLOv8** (ultralytics) opcional — si falta el modelo `.pt`, **cae automaticamente a Hough** (la tuberia nunca se queda a oscuras).
- Dataset **sintetico** generado desde Gazebo (capturas de la torre con defectos).
- `defect_mapper_node` proyecta cada defecto al **mapa cilindrico** de la torre (coordenadas axial + angular).
- `image_capture_node` guarda frames + detecciones para el informe.

**Visual:** `yolo_detection.png` — frame de la camara con los defectos detectados (cajas/circulos).

**Notas de orador:** "La vision detecta los defectos de la pared del tubo. Por defecto usamos HoughCircles, que es robusto para nuestros defectos circulares, y dejamos un backend YOLOv8 opcional entrenado con dataset sintetico. Si el modelo no esta, cae solo a Hough. Cada deteccion se proyecta al cilindro de la torre para saber donde esta exactamente."

---

## Slide 11 — Mision: Behaviour Tree + voz + UI web

**Contenido**
- `mission_controller`: orquesta **Nav2 + MoveIt + percepcion** en una unica mision (Behaviour Tree).
- **Voz**: `faster-whisper` (Whisper **local**) transcribe → NLU con **LangChain + Google Gemini** traduce *"inspecciona la torre"* a un intent/accion.
- **UI web Flask** (`http://localhost:5000`): control y monitorizacion de la mision en el navegador.
- **Simulacion de bateria**: se drena al moverse y se **recarga en `charging_station`**.
- **Gates obligatorios** para entrar en la zona del tubo; markers en RViz (waypoints, gates, ruta, bateria).

**Visual:** `mission_ui.png` — captura de la UI web de la mision (estado, bateria, botones).

**Notas de orador:** "El cerebro es un arbol de comportamiento que encadena navegar, cruzar los gates, entrar al tubo, desplegar el brazo, inspeccionar y generar el informe. Se dispara por voz: Whisper transcribe en local y un modelo Gemini via LangChain interpreta la orden. Ademas hay una interfaz web para seguir la mision y simulamos la bateria, que se recarga en su estacion."

---

## Slide 12 — Informe automatico (Gemini)

**Contenido**
- `generate_inspection_report.py` toma las detecciones de la mision (`manifest.json`, `detections.ndjson`, frames).
- **Agrupa** detecciones por proximidad en (axial, angular) → defectos unicos.
- Genera:
  1. **Resumen Markdown** (conteos por clase + tabla de defectos),
  2. **Mapa visual** de defectos sobre el cilindro desplegado (matplotlib),
  3. **Informe en español** redactado por **Gemini** (`gemini-2.5-flash`) con resumen, mapa e imagenes representativas.
- Modo `--dry-run`: genera resumen + mapa sin llamar a la API.

**Visual:** `report.png` — el mapa de defectos del cilindro desplegado + extracto del informe.

**Notas de orador:** "Al final de la mision, generamos un informe automatico. Agrupamos las detecciones en defectos unicos, dibujamos el cilindro de la torre desplegado con los defectos posicionados, y le pasamos todo a Gemini para que redacte un informe en español listo para un tecnico."

---

## Slide 13 — Demo: mision completa (video)

**Contenido**
- Video de la mision de inspeccion de principio a fin (~3:30).
- Si el formato no admite video: thumbnail + enlace a `video/demo.mp4`.

**Visual:** `demo_thumbnail.png` (o el video embebido).

**Notas de orador (locucion por bloques mientras corre el video):**
1. Orden por voz → la mision arranca.
2. Navegacion por la nave esquivando personas.
3. Cruce de gates y entrada al tubo subiendo la rampa.
4. Despliegue del brazo y barrido 360°.
5. Deteccion de defectos en directo.
6. Generacion del informe.

---

## Slide 14 — Demo: timeline

**Contenido (tabla)**

| t (mm:ss) | Hito |
| --------- | ---- |
| 00:00 | Orden por voz reconocida (Whisper + Gemini) |
| 00:20 | Nav2 navega a la zona del tubo (esquiva personas) |
| 01:00 | Cruce de gates + rampa de 13° |
| 01:40 | Brazo desplegado, inicio del barrido 360° |
| 02:30 | Defectos detectados y proyectados al cilindro |
| 03:10 | Informe generado con Gemini |

**Visual:** tabla embebida.

**Notas de orador:** "Esta es la cronologia de la demo: de la orden por voz al informe, en poco mas de tres minutos."

---

## Slide 15 — Metricas cuantitativas

**Contenido (rellenar con datos reales antes de la defensa):**

| Metrica | Valor |
| ------- | ----- |
| Tiempo medio de mision completa | _TODO_ |
| Tasa de exito de navegacion (cruce de gates) | _TODO_ |
| Precision / recall de deteccion de defectos vs ground truth | _TODO_ |
| Defectos detectados / sembrados | _TODO_ |
| Tasa de acierto del reconocimiento de voz | _TODO_ |

**Visual:** tabla (y opcional grafico de barras).

**Notas de orador:** "Estas son las metricas con las que evaluamos el sistema: tiempo de mision, exito de navegacion y, sobre todo, precision de la deteccion contra el ground truth de defectos que conocemos del mundo de simulacion."

---

## Slide 16 — Conclusiones y trabajo futuro

**Contenido**
- **Lo demostrado:**
  - Integracion end-to-end de **navegacion + manipulacion + percepcion + comportamiento** en ROS 2.
  - Mision autonoma disparada por **voz en lenguaje natural** (Whisper + Gemini).
  - Navegacion robusta en **rampa** y **esquivando personas dinamicas**.
  - **Informe automatico** generado por IA.
- **Trabajo futuro:**
  - Llevar el sistema al **robot real**.
  - Loop closure en SLAM para entornos mas grandes.
  - Reentrenar **YOLO con dataset real** de torres.
  - Cerrar el lazo: que el informe alimente una planificacion de mantenimiento.

**Visual:** dos bloques (logros / futuro).

**Notas de orador:** "Hemos demostrado un sistema completo, de la voz al informe, que navega entre personas, entra en el tubo de 30 m de la torre e inspecciona la pared sin necesidad de un operario. Como futuro, el salto al robot real y mejorar la vision con datos reales."

---

## Slide 17 — Preguntas

**Contenido**
- "¿Preguntas?"
- "Gracias por vuestra atencion."
- Repositorio: `github.com/davidmoreda/ROS2_Wind_Tower_Inspection`

**Visual:** slide limpia, granate, con el handle del repo.

**Notas de orador:** "Muchas gracias, quedamos a vuestras preguntas."

---

## Apendice — Lista de recursos visuales a aportar

| Archivo | Que debe mostrar | Slide |
| ------- | ---------------- | ----- |
| `logo_loyola.png` | Logo de la universidad | 1 |
| `contexto_torre.png` | Torre eolica / tramo de torre con el robot | 3 |
| `arquitectura.png` | Diagrama de bloques de los 6 paquetes | 4 |
| `robot_gazebo.png` | Husky + UR5e en Gazebo | 5 |
| `world_actors.png` | Mundo con personas caminando | 6 |
| `nav2_rviz.png` | RViz: mapa, costmaps, planes, nube filtrada | 7 |
| `moveit_rviz.png` | UR5e haciendo el barrido en el tubo | 9 |
| `yolo_detection.png` | Frame con defectos detectados | 10 |
| `mission_ui.png` | UI web del mission_controller (`:5000`) | 11 |
| `report.png` | Mapa de defectos del cilindro + informe | 12 |
| `demo_thumbnail.png` / `video/demo.mp4` | Demo de la mision completa | 13 |
