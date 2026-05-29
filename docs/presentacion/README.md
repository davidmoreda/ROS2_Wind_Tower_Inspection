# Presentacion del TFM

Beamer LaTeX de la presentacion del proyecto **Inspeccion Autonoma de Torres Eolicas** (Husky A200 + UR5e + Nav2 + MoveIt 2 + vision Hough/YOLO + Behaviour Tree + voz/Gemini).

Cubre el proyecto completo, no una practica en concreto. Las practicas tienen sus propias memorias en `docs/practica{1,2,3}/`.

> **Fuente de la verdad tecnica:** la descripcion real del sistema (paquetes, nodos, flujo de mision) esta en [`docs/estructura_y_lanzamiento.md`](../estructura_y_lanzamiento.md). El plan de slides de abajo ya esta alineado con ese documento.

## Indice

- [Estado del trabajo](#estado-del-trabajo)
- [Modelo real del sistema](#modelo-real-del-sistema)
- [Estructura](#estructura)
- [Compilar](#compilar)
- [Plan de slides](#plan-de-slides)
- [Reparto del equipo](#reparto-del-equipo)
- [Que falta para terminar](#que-falta-para-terminar)

## Estado del trabajo

- [x] Estructura Beamer creada con tema custom Loyola (rojo granate).
- [x] Slides esqueleto con TODOs por seccion y reparto P1/P2/P3.
- [x] Plantilla con tabla del tuning controller y tabla de metricas.
- [x] Plan de slides re-alineado con el modelo real (Hough/YOLO, voz+Gemini, UI web, bateria, informe Gemini, personas dinamicas).
- [x] Version Markdown para Manus IA en [`presentacion.md`](presentacion.md).
- [ ] Subir `figures/logo_loyola.png` (logo de la universidad).
- [ ] Subir capturas (ver lista en [Que falta para terminar](#que-falta-para-terminar)).
- [ ] Grabar el video de la demo y dejarlo en `video/demo.mp4`.
- [ ] Rellenar tabla de metricas.
- [ ] Rellenar narracion oral en cada slide (TODOs en `presentacion.tex`).
- [ ] Compilar y ensayar.

**Pre-requisito:** instalar LaTeX si no se tiene:

```bash
sudo apt install texlive-latex-recommended texlive-latex-extra \
                 texlive-fonts-recommended texlive-lang-spanish \
                 texlive-fonts-extra latexmk
```

## Modelo real del sistema

Resumen de lo que **de verdad** corre, para no prometer en las slides nada que no haya:

| Bloque | Que hay realmente |
| ------ | ----------------- |
| **Plataforma** | Husky A200 + UR5e (6 DOF) + camara de inspeccion en el TCP. Sensores: IMU, LIDAR 2D Hokuyo, LIDAR 3D Velodyne VLP-16. Config declarativa Clearpath (`robot.yaml`). |
| **Simulacion** | Gazebo Sim 8. Mundos SDF: principal, `defects_actors` (con **personas que caminan** + defectos), y `synthetic` (dataset). Tramo de torre (`TRAMO_TORRE.STL`) + tubo con virador. |
| **Navegacion** | Nav2 (planner NavFn, controller **RPP**, BT navigator, recovery) + **SLAM Toolbox** (mapeo) + **EKF** (`robot_localization`) + **AMCL**. `obstacle_cloud_filter` filtra el suelo de la nube 3D para el costmap → **esquiva personas dinamicas**. |
| **Manipulacion** | Paquete `wind_tower_arm_control` con MoveIt 2: OMPL (RRTConnect) articular + Pilz LIN cartesiano. `arm_inspection_node` hace **barrido 360°** (home → 7 puntos @45° → home) y publica `/arm/inspection_ready`. |
| **Percepcion** | `detector_node`: **HoughCircles (default)** + **YOLOv8/ultralytics opcional** con *fallback* automatico a Hough. `defect_mapper_node` proyecta defectos al cilindro de la torre. Dataset sintetico desde Gazebo. |
| **Mision / interaccion** | `mission_controller` (Flask, **UI web en `:5000`**): orquesta Nav2 + MoveIt + percepcion, **simula bateria** (se drena al moverse, recarga en `charging_station`), **gates obligatorios** para entrar al tubo, publica markers en RViz. Voz: **faster-whisper** (local) → NLU con **LangChain + Gemini** → intents. |
| **Informe** | `generate_inspection_report.py`: agrupa detecciones, genera resumen Markdown + mapa de defectos (cilindro desplegado, matplotlib) y un **informe en español con Gemini** (`gemini-2.5-flash`). |

## Estructura

```
docs/presentacion/
|-- presentacion.tex     # documento principal Beamer (~15 slides)
|-- presentacion.md      # version Markdown lista para Manus IA
|-- preamble.tex         # tema custom Loyola + paquetes
|-- Makefile             # latexmk wrapper (incluye target handout)
|-- README.md            # este archivo
|-- figures/             # logo, capturas de cada bloque, diagrama arquitectura
`-- video/               # mp4 de la demo (gitignored)
```

## Compilar

```bash
cd docs/presentacion
make             # genera presentacion.pdf
make watch       # recompila al guardar
make view        # abre el PDF
make handout     # version handout (4 slides por hoja para imprimir)
make clean       # limpia artefactos
```

## Plan de slides

Tema: rojo granate Loyola (`#9F1D35`) con grises de soporte. Ratio 16:9.

**Duracion total objetivo: 15 minutos** (presentacion en clase con video de demo embebido).

| # | Slide                                  | Duracion | Narra | Recursos visuales |
| - | -------------------------------------- | -------- | ----- | ----------------- |
| 1 | Portada                                | 0:15     | P1    | logo_loyola |
| 2 | Indice                                 | 0:15     | P1    | -- |
| 3 | Contexto y objetivo                    | 0:45     | P1    | contexto_torre.png |
| 4 | Arquitectura general (6 paquetes)      | 0:45     | P1    | arquitectura.png |
| 5 | Plataforma robotica + simulacion       | 0:45     | P2    | robot_gazebo.png |
| 6 | Mundo dinamico (defectos + personas)   | 0:30     | P2    | world_actors.png |
| 7 | Nav2: SLAM + EKF + AMCL + esquiva       | 1:00     | P2    | nav2_rviz.png |
| 8 | Tuning del controller (rampa 13°)      | 0:45     | P2    | tabla embebida |
| 9 | MoveIt 2 + barrido 360° del UR5e       | 0:45     | P3    | moveit_rviz.png |
| 10 | Percepcion: Hough/YOLO + cilindro      | 0:45     | P3    | yolo_detection.png |
| 11 | Mision: BT + voz/Gemini + UI web       | 1:00     | P3    | mission_ui.png |
| 12 | Informe automatico (Gemini)            | 0:30     | P3    | report.png |
| 13 | Demo (video)                           | 3:30     | rota  | video/demo.mp4 |
| 14 | Demo: timeline                         | 0:30     | P1    | tabla embebida |
| 15 | Metricas cuantitativas                 | 0:45     | P2    | tabla |
| 16 | Conclusiones y trabajo futuro          | 0:30     | P1    | -- |
| 17 | Preguntas                              | --       | --    | -- |

**Reparto temporal**:
- Oral (slides 1-12, 14-16): **~8:30**
- Video de demo (slide 13): **~3:30**
- Buffer / transiciones: **~3:00** para preguntas y margen

**Total con buffer**: **15:00**.

### Avisos de timing

- Cada bloque tematico va a 30-60 segundos: hay que practicar para no excederse.
- El video es el unico modulo "fijo". Si necesitais recuperar tiempo, recortar el video antes que las explicaciones orales.
- Ensayar con cronometro. Si en la primera pasada salen 18 min, hay que reducir la narracion de las slides tecnicas (7-12), no la introduccion ni la demo.

## Reparto del equipo

| Persona | Slides que narra                                                        | Capturas a aportar |
| ------- | ----------------------------------------------------------------------- | ------------------ |
| **P1**  | Portada, Indice, Contexto, Arquitectura, Timeline, Conclusiones         | `contexto_torre.png`, `arquitectura.png` |
| **P2**  | Plataforma, Mundo dinamico, Nav2, Tuning, Metricas                      | `robot_gazebo.png`, `world_actors.png`, `nav2_rviz.png` |
| **P3**  | MoveIt+barrido, Percepcion, Mision (voz/UI), Informe                    | `moveit_rviz.png`, `yolo_detection.png`, `mission_ui.png`, `report.png` |
| **Todos** | Narran la demo por bloques (~3:30 segun la timeline)                 | `demo_thumbnail.png` + `video/demo.mp4` |

## Que falta para terminar

1. **Subir el logo** a `figures/logo_loyola.png`. Si no esta, comentar las lineas `\titlegraphic` y `\logo` en `presentacion.tex`.
2. **Crear el diagrama de arquitectura** (`figures/arquitectura.png`) con las 6 cajas de paquetes propios y las flechas de datos.
3. **Recoger capturas** del resto de bloques:
   - `robot_gazebo.png`, `world_actors.png` (mundo con personas), `nav2_rviz.png`,
   - `moveit_rviz.png`, `yolo_detection.png`,
   - `mission_ui.png` (la UI web del `mission_controller` en `:5000`), `report.png` (el informe Gemini / mapa de defectos).
4. **Grabar la demo** (~3:30 min) que muestre la mision completa: orden por voz → navegacion al tubo cruzando los gates (esquivando personas) → despliegue y barrido 360° del brazo → deteccion de defectos → informe automatico.
5. **Rellenar TODOs** de narracion en `presentacion.tex` para tener notas de speaker.
6. **Compilar** y **ensayar tiempos**.

> Para una presentacion generada con **Manus IA** (o similar), usar [`presentacion.md`](presentacion.md): es el mismo contenido en Markdown autocontenido, con notas de orador por slide.
