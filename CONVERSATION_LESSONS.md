# Conversation Lessons

Resumen de lo que funcionó y lo que no funcionó en la conversación de ajuste de la inspección autónoma.

## Lo que sí funcionó

- Relanzar la simulación y la inspección con el workspace recompilado.
- Reducir la prueba de `90° / 30m` a `5° / 1m / 30Hz` para depurar más rápido.
- Separar de forma clara los dos lazos:
  - `/turner/cmd_vel` para el virador.
  - `/robot/platform/cmd_vel` para la base.
- Detectar con `ros2 topic info -v /turner/cmd_vel` que había dos publishers activos a la vez.
- Bloquear el teleop del virador mientras `autonomous_active=true`.
- Confirmar que `stability_monitor` publica `bottom_lane_locked`, `safe_to_scan` y `safe_to_index_tube`.
- Ver que la máquina de estados puede avanzar, indexar, girar tangencialmente y realinear a bottom lane.
- Ver que la IMU se calibra al arranque con 50 muestras si la postura inicial es buena.

## Lo que no funcionó bien

- Usar `lane_delta_theta_deg=90°` para depuración: era demasiado lento y dificultaba distinguir lógica de tiempo de espera.
- Depender de una sola muestra para decidir `safe_to_index_tube`: provocaba oscilaciones de estado.
- Tener varios publishers sobre `/turner/cmd_vel` al mismo tiempo: generaba movimiento aparente a trompicones y hacía difícil saber quién mandaba.
- Asumir que el tubo seguía girando por una orden nueva cuando muchas veces era inercia del joint o un comando previo.
- Mezclar validación funcional con ajuste fino de control en la misma prueba.

## Errores de criterio a evitar

- Confundir `θ_tube` con `α_robot`.
- Considerar cobertura nominal durante bypass como si fuera inspección normal.
- Usar vídeo continuo como evidencia principal en vez de capturas sincronizadas por distancia.
- Interpretar `WAIT_SAFE_TO_INDEX` como fallo del virador cuando puede ser solo una puerta de seguridad activa.

## Comandos útiles

```bash
ros2 topic info -v /turner/cmd_vel
ros2 topic echo /inspection/mission_status --once --full-length
ros2 topic echo /inspection/state_text
ros2 topic echo /turner/angle_deg
ros2 topic echo /inspection/stability --once --full-length
```

## Qué conviene hacer la próxima vez

- Arrancar con un perfil corto de prueba.
- Comprobar primero los publishers activos de cada actuador.
- Validar `mission_status` antes de tocar ganancias.
- Relanzar todo tras cambiar parámetros; no asumir que un nodo viejo recoge el YAML nuevo.
- Ajustar una sola cosa por prueba: primero seguridad, luego indexado, luego alineación, luego suavizado.

## Resumen práctico

La secuencia correcta no es “hacer que rote todo sin parar”, sino:

1. Mantener la base estable en la generatriz inferior.
2. Avanzar axialmente por una calle.
3. Parar.
4. Indexar el tubo un paso pequeño.
5. Realinear el robot.
6. Reanudar.

Ese fue el cambio conceptual importante de la sesión.
