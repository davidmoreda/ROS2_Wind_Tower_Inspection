# TEAM_WORKFLOW — Cómo trabajamos en equipo

> Guía obligatoria para los tres miembros del equipo antes de tocar código.
> Objetivo: que tres branches puedan desarrollarse en paralelo sin pisarse.

---

## 1. Reparto de responsabilidades

### Quién toca qué

| Área | Responsable | Branch | Paquetes propios |
|---|---|---|---|
| Navegación autónoma + Nav2 + agente LangChain | David | `feature/nav2-control` | `wind_tower_inspection_behaviour`, `wind_tower_bringup` (launchers) |
| Visión + captura + YOLO | Compañero visión | `feature/vision-yolo` | `wind_tower_perception` |
| Brazo UR5e + MoveIt | Compañero brazo | `feature/moveit-arm` | `wind_tower_arm_control` (nuevo) |

### Regla de propiedad

Cada persona es la **única que hace commits** en sus paquetes propios.  
Si necesitas algo de un paquete ajeno, **abre una Issue** o habla con el responsable — no edites tú directamente.

---

## 2. Archivos que NADIE puede tocar sin consenso

Estos archivos afectan a todos. Cualquier cambio requiere que los tres estén de acuerdo antes de editarlos:

```
ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world.sdf
ros2_ws/src/wind_tower_description/         (URDFs, meshes)
ros2_ws/src/wind_tower_bringup/config/robot.yaml
```

Si necesitas modificar alguno, avisa en el grupo antes de hacerlo.

---

## 3. El contrato de topics (NO se cambia sin avisar)

Estos topics son la interfaz entre los tres. Si cambias el nombre, tipo o frecuencia de uno, **debes avisar a los otros dos** porque su código dejará de funcionar.

| Topic | Tipo | Produce | Consume |
|---|---|---|---|
| `/inspection/state_text` | `std_msgs/String` | David (state_machine) | Visión, Brazo |
| `/inspection/autonomous_active` | `std_msgs/Bool` | David (state_machine) | Visión, Brazo |
| `/inspection/cylindrical_pose` | `std_msgs/String` JSON | David (cylindrical_map) | Visión, Brazo |
| `/inspection/detections/raw` | `vision_msgs/Detection2DArray` | Visión (detector_node) | Brazo |
| `/inspection/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge | Visión |
| TF `camera_link` | TF tree | Brazo (MoveIt) | Visión (proyección) |
| `/arm/inspection_ready` | `std_msgs/Bool` | Brazo | David (state_machine) |

---

## 4. Flujo de trabajo con branches

### 4.1 Crear tu branch

Siempre parte desde `main` actualizado:

```bash
git checkout main
git pull origin main
git checkout -b feature/nav2-control   # sustituye por tu branch
```

### 4.2 Trabajar en tu branch

Haz commits pequeños y frecuentes. No acumules días de trabajo en un solo commit:

```bash
git add <archivos-que-son-tuyos>
git commit -m "descripción corta de qué cambia y por qué"
git push origin feature/nav2-control
```

**Nunca uses `git add .` o `git add -A`** — puedes añadir archivos que no son tuyos por error. Añade siempre por nombre de archivo o carpeta.

### 4.3 Mantenerte al día con main

Cuando el compañero merge algo a `main`, trae esos cambios a tu branch regularmente para evitar conflictos grandes al final:

```bash
git fetch origin
git merge origin/main
```

Si hay conflictos, resuélvelos tú — son en tus archivos, nadie más debería haberte tocado nada.

### 4.4 Cuándo NO hay que hacer merge con main

- En mitad de una feature incompleta que rompe el build
- Si tienes cambios sin testear en nodos que afectan a otros

---

## 5. Cómo hacer un Pull Request antes de mergear a main

**Nunca se mergea directamente a `main` con `git merge` o `git push` desde tu máquina.**  
Todo pasa por un Pull Request (PR) en GitHub para que al menos otro miembro lo revise.

### 5.1 Crear el PR

1. Sube tu branch a GitHub:
   ```bash
   git push origin feature/nav2-control
   ```

2. Ve a GitHub → repositorio → aparecerá un botón **"Compare & pull request"**, clícalo.

3. Rellena el PR:
   - **Título**: corto y descriptivo — qué feature añade este PR
   - **Descripción**: explica QUÉ hace, POR QUÉ y cómo probarlo. Usa esta plantilla:

```
## Qué hace este PR
- Añade X
- Modifica Y porque Z

## Cómo probarlo
1. Lanzar simulation.launch.py
2. Lanzar inspection.launch.py
3. Verificar que el topic /X publica correctamente con: ros2 topic echo /X

## Topics nuevos o modificados
- /nuevo_topic (std_msgs/String) — describe qué publica

## Lo que NO está en este PR (queda para después)
- Feature Y pendiente
```

4. Asigna como **reviewer** al menos a uno de los otros dos compañeros.

5. Clica **"Create pull request"**.

### 5.2 Reglas para que un PR sea aprobado

Un PR **no se mergea** hasta que:

- [ ] Al menos **1 reviewer** lo ha aprobado en GitHub
- [ ] No hay comentarios sin resolver
- [ ] El build compila sin errores (`colcon build`)
- [ ] No hay marcadores de conflicto (`<<<<<<<`) en ningún archivo
- [ ] Los topics del contrato (sección 3) no han cambiado sin avisar

---

## 6. Cómo revisar un PR

Cuando te llega una notificación de review en GitHub:

### 6.1 Qué mirar

1. **¿Toca archivos que no son suyos?**  
   Si el PR de visión modifica `inspection.launch.py` o `state_machine_node.py`, hay que rechazarlo.

2. **¿Cambia algún topic del contrato (sección 3)?**  
   Si sí, y no se ha avisado antes, pedir que se documente el cambio.

3. **¿El código nuevo rompe alguna interfaz existente?**  
   Revisa si elimina o renombra topics, funciones o parámetros que otros usan.

4. **¿Los launchers nuevos o modificados son correctos?**  
   Comprueba que los `setup.py` incluyen todos los archivos nuevos que se añaden.

### 6.2 Cómo dejar comentarios en GitHub

- Comenta **línea a línea** directamente en el diff — es más útil que un comentario general
- Si algo es bloqueante (impide el merge): usa **"Request changes"**
- Si es una sugerencia no bloqueante: déjalo como comentario normal y aprueba igualmente
- Cuando el autor lo corrija, marca el hilo como **"Resolved"**

### 6.3 Aprobar y mergear

1. Si todo está bien: clica **"Approve"** en GitHub
2. El autor del PR (no el reviewer) es quien clica **"Merge pull request"**
3. Usar siempre **"Squash and merge"** si el PR tiene muchos commits pequeños de trabajo en progreso, o **"Merge commit"** si los commits son limpios y descriptivos
4. Después del merge, borrar la branch en GitHub (GitHub lo propone automáticamente)

---

## 7. Qué hacer si hay un conflicto en el PR

Si GitHub muestra "This branch has conflicts that must be resolved":

1. El **autor del PR** es quien resuelve los conflictos, no el reviewer
2. Desde tu máquina:
   ```bash
   git fetch origin
   git merge origin/main
   # resolver conflictos en los archivos marcados
   git add <archivos-resueltos>
   git commit
   git push origin feature/mi-branch
   ```
3. El PR se actualiza automáticamente en GitHub

---

## 8. Resumen rápido — el checklist antes de pedir review

```
[ ] He hecho git pull origin main antes de empezar mi branch
[ ] Solo he tocado archivos que son míos (ver sección 1)
[ ] No he cambiado topics del contrato sin avisar (ver sección 3)
[ ] colcon build pasa sin errores
[ ] He descrito en el PR cómo probarlo
[ ] He asignado reviewer en GitHub
```
