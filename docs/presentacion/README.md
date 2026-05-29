# Presentacion del TFM

Beamer LaTeX de la presentacion del proyecto **Inspeccion Autonoma de Torres Eolicas** (Husky + UR5e + NAV2 + MoveIt 2 + YOLO + BT/voz).

Cubre el proyecto completo, no una practica en concreto. Las practicas tienen sus propias memorias en `docs/practica{1,2,3}/`.

## Indice

- [Estado del trabajo](#estado-del-trabajo)
- [Estructura](#estructura)
- [Compilar](#compilar)
- [Plan de slides](#plan-de-slides)
- [Reparto del equipo](#reparto-del-equipo)
- [Que falta para terminar](#que-falta-para-terminar)

## Estado del trabajo

- [x] Estructura Beamer creada con tema custom Loyola (rojo granate).
- [x] 14 slides esqueleto con TODOs por seccion y reparto P1/P2/P3.
- [x] Plantilla con tabla del tuning controller y tabla de metricas.
- [ ] Subir `figures/logo_loyola.png` (logo de la universidad).
- [ ] Subir capturas (`arquitectura.png`, `robot_gazebo.png`, `nav2_rviz.png`, `moveit_rviz.png`, `yolo_detection.png`, `bt_voice.png`, `contexto_torre.png`, `demo_thumbnail.png`).
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

## Estructura

```
docs/presentacion/
|-- presentacion.tex     # documento principal Beamer (~15 slides)
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
| 4 | Arquitectura general                   | 0:45     | P1    | arquitectura.png |
| 5 | Plataforma robotica + simulacion       | 0:45     | P2    | robot_gazebo.png |
| 6 | NAV2: SLAM + EKF + AMCL                | 1:00     | P2    | nav2_rviz.png |
| 7 | Tuning del controller (rampa)          | 1:00     | P2    | tabla embebida |
| 8 | MoveIt 2 sobre el UR5e                 | 0:45     | P3    | moveit_rviz.png |
| 9 | Percepcion YOLO + dataset sintetico    | 0:45     | P3    | yolo_detection.png |
| 10 | BT + voz + mission controller         | 0:45     | P3    | bt_voice.png |
| 11 | Demo (video)                           | 3:45     | rota  | video/demo.mp4 |
| 12 | Demo: timeline                         | 0:30     | P1    | tabla embebida |
| 13 | Metricas cuantitativas                 | 0:45     | P2    | tabla |
| 14 | Conclusiones y trabajo futuro          | 0:30     | P1    | -- |
| 15 | Preguntas                              | --       | --    | -- |

**Reparto temporal**:
- Oral (slides 1-10, 12-14): **~7:55**
- Video de demo (slide 11): **~3:45**
- Buffer / transiciones: **~3:20** para preguntas y margen

**Total con buffer**: **15:00**.

### Avisos de timing

- Cada bloque tematico va a 45-60 segundos: hay que practicar para no excederse.
- El video es el unico modulo "fijo". Si necesitais recuperar tiempo, recortar el video antes que las explicaciones orales.
- Ensayar con cronometro. Si en la primera pasada salen 18 min, hay que reducir la narracion de las slides 6-10 (las tecnicas), no la introduccion ni la demo.

## Reparto del equipo

| Persona | Slides que narra                                                        | Capturas a aportar |
| ------- | ----------------------------------------------------------------------- | ------------------ |
| **P1**  | Portada, Indice, Contexto, Arquitectura, Conclusiones                   | `contexto_torre.png`, `arquitectura.png` |
| **P2**  | Plataforma, NAV2 (2 slides), Metricas                                   | `robot_gazebo.png`, `nav2_rviz.png` |
| **P3**  | MoveIt, YOLO, BT+voz                                                    | `moveit_rviz.png`, `yolo_detection.png`, `bt_voice.png` |
| **Todos** | Narran la demo por bloques (~3:45 segun la timeline de la slide 12)  | `demo_thumbnail.png` + `video/demo.mp4` |

## Que falta para terminar

1. **Subir el logo** a `figures/logo_loyola.png`. Si no esta, comentar las lineas `\titlegraphic` y `\logo` en `presentacion.tex`.
2. **Crear el diagrama de arquitectura** (`figures/arquitectura.png`). Se puede hacer en draw.io o aprovechar diagramas existentes.
3. **Recoger capturas** del resto de bloques. Se pueden reutilizar capturas de las memorias de Practica 1/2/3 que ya teneis pensadas.
4. **Grabar la demo** (~3:45 min) que muestre la mision completa: activacion por voz, navegacion al tubo, despliegue del brazo, deteccion YOLO y generacion del informe.
5. **Rellenar TODOs** de narracion en `presentacion.tex` para tener notas de speaker.
6. **Compilar** y **ensayar tiempos**.
