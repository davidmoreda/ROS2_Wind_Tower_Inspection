# Video demo para la presentacion del TFM

Duracion objetivo: **4:30 - 5:00 minutos**. Va embebido o enlazado en la slide 11-12 de `presentacion.tex`. Equipo de **3 personas**.

A diferencia de los videos por practica, este video cubre la **mision completa de inspeccion end-to-end** y se narra **en vivo durante la presentacion** sin voz en el propio video (silenciado o con musica suave). Cada uno comenta su parte mientras el video corre.

## Indice
- [Antes de grabar (setup)](#antes-de-grabar-setup)
- [Guion escena por escena](#guion-escena-por-escena)
- [Comandos preparados](#comandos-preparados)
- [Como embeberlo en Beamer](#como-embeberlo-en-beamer)
- [Tips de grabacion y edicion](#tips-de-grabacion-y-edicion)

---

## Antes de grabar (setup)

1. **Compilar el workspace**:
   ```bash
   cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Preparar layout de pantalla**:
   - Pantalla izquierda: Gazebo a 60% del ancho.
   - Pantalla derecha: RViz con `navigation.rviz` o vista combinada.
   - Pequeno overlay opcional con la imagen de la camara de inspeccion.

3. **Asegurar que estos nodos arrancan limpios**:
   - `simulation.launch.py` (Gazebo + Husky).
   - `navigation_amcl.launch.py` (NAV2 + AMCL + mission_controller + voice_command_node).
   - El mapa ya esta en `ros2_ws/maps/wind_tower.*`.

4. **Grabador** (OBS) a 1080p, 30 fps. Audio del microfono **DESACTIVADO** (la narracion va en vivo).

5. **Ensayar la mision** antes de grabar la toma definitiva. Tener un rosbag de respaldo de la mision por si Gazebo falla en directo.

---

## Guion escena por escena

| Tiempo      | Escena                                  | Pantalla                          | Narracion en vivo |
| ----------- | --------------------------------------- | --------------------------------- | ----------------- |
| 0:00 - 0:30 | **Lanzamiento y spawn** `[P1]`          | Gazebo Sim                         | "Lanzamos primero la simulacion. Vemos el Husky con el UR5e aparecer dentro de la nave industrial, junto al tramo de torre eolica. La configuracion del robot la genera Clearpath desde un yaml versionado en el repo." |
| 0:30 - 1:00 | **Activacion por voz** `[P3]`           | Terminal + ondas de audio          | "El operador da el comando por voz \emph{\"inspecciona la torre\"}. Whisper transcribe el audio y LangChain interpreta la intencion. El mission_controller recibe la orden y arranca el behaviour tree de inspeccion." |
| 1:00 - 2:15 | **Navegacion al tubo** `[P2]`           | Gazebo + RViz lado a lado          | "NAV2 toma el control. AMCL ya esta localizado en el mapa estatico. El planner global genera la ruta y el RegulatedPurePursuitController la sigue, incluyendo la subida de la rampa de 13 grados donde antes el robot patinaba. Las particulas AMCL siguen concentradas durante todo el recorrido." |
| 2:15 - 3:15 | **Entrada al tubo + despliegue brazo** `[P3]` | Gazebo zoom + RViz MoveIt    | "El robot entra al tubo de la torre. El BT pasa al nodo de manipulacion. MoveIt 2 planifica con Pilz LIN una trayectoria cartesiana del UR5e para llevar la camara a la pose de inspeccion. El JointTrajectoryController ejecuta." |
| 3:15 - 4:15 | **Inspeccion + YOLO** `[P3]`            | Camara TCP + mapa cilindrico      | "La camara montada en el TCP recorre el interior de la torre. YOLO detecta defectos en tiempo real (oxido, grietas, soldaduras). El defect_mapper proyecta cada deteccion al mapa cilindrico de la torre que aparece a la derecha." |
| 4:15 - 4:45 | **Informe y fin de mision** `[P1]`      | Terminal + reporte generado        | "El mission_controller cierra el BT, genera el informe en formato JSON con la lista de defectos y posiciones. El robot recibe la orden de regresar a la pose home. Mision completada." |

**Total**: ~4:45.

---

## Comandos preparados

Para tener listos antes de grabar:

```bash
# T1 - Simulacion
ros2 launch wind_tower_bringup simulation.launch.py

# T2 - NAV2 + AMCL + behaviour layer (mission_controller + voice_command_node)
ros2 launch wind_tower_bringup navigation_amcl.launch.py

# T3 - Disparar mision manualmente si la voz falla en grabacion
ros2 run wind_tower_inspection_behaviour mission_controller
```

Si quieres grabar el comando de voz real en el video:

```bash
# T4 - Activar voice_command_node y hablar al microfono
ros2 topic echo /voice/command
# Luego: pulsar push-to-talk (segun configuracion del nodo) y decir
# "inspecciona la torre"
```

---

## Como embeberlo en Beamer

Hay tres opciones, de mas a menos robusta:

1. **Enlace al archivo** (siempre funciona, requiere abrir el video en el reproductor del SO):
   ```latex
   \href{run:video/demo.mp4}{Pulsa para reproducir}
   ```
   Ya esta puesto asi en `presentacion.tex` slide 11.

2. **Embebido con `\movie`** (depende del lector PDF):
   ```latex
   \movie[width=12cm,height=6.75cm,autostart,showcontrols]{}{video/demo.mp4}
   ```
   Funciona en Okular y algunos lectores. En Adobe Reader no.

3. **GIF embebido como imagen** (siempre funciona, sin sonido y bucle):
   ```bash
   # Convertir el video a GIF (calidad media):
   ffmpeg -i video/demo.mp4 -vf "fps=10,scale=960:-1:flags=lanczos" \
          -loop 0 figures/demo.gif
   ```
   ```latex
   \animategraphics[width=12cm,autoplay,loop]{10}{figures/demo-}{0}{N}
   ```

**Recomendado para una presentacion en clase**: opcion 1 (enlace) o, si quieres que se vea integrado, exportar la slide 11 como hueco y abrir el video en el reproductor del SO directamente con doble click cuando llegue ese momento.

---

## Tips de grabacion y edicion

- **Sin audio en el video**. La narracion es en vivo.
- **Velocidad x1.0** para los bloques de navegacion. **x2.0** opcional para SLAM si tarda mucho.
- **Cortes limpios** entre fases: dejar 0.5s de "respiracion" entre bloques.
- **Subtitulos en pantalla** opcionales: insertar texto "1. Activacion por voz", "2. Navegacion", "3. Manipulacion", "4. Percepcion" cuando cambia la fase.
- **Tamano del archivo**: mantener bajo 100 MB (H.264, CRF 23). Si Beamer va a llevarlo embebido, mejor 30-50 MB.
- **Backup**: guardar tambien un rosbag de la mision por si hay que regrabar.

## Postproduccion en Kdenlive (sugerido)

1. Importar el OBS .mkv.
2. Cortar las partes muertas (esperas de Gazebo, etc.) para llegar a 4:30-5:00.
3. Anadir subtitulos de fase si quieres.
4. Exportar a MP4 H.264, 1080p, 30 fps.
5. Copiar a `docs/presentacion/video/demo.mp4` (gitignored por defecto; usar Git LFS o enlace externo si quereis versionarlo).
