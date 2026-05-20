# tools/debug

Scripts de depuracion standalone (sin instalar como paquete ROS).

## capture_inspection_debug.py

Grabador tipo "black box" para la mision autonoma. Guarda JSONL por topic y un resumen corto.

Ejecucion (desde la raiz del repo, con el entorno ROS ya cargado en la terminal):

    python3 tools/debug/capture_inspection_debug.py --duration 90

Salida:

- Crea `debug_runs/<timestamp>/`
- Genera `summary.md` listo para pegar a un agente (sin logs enormes)

Recomendado para compartir con un agente:

- Copiar y pegar solo `debug_runs/<timestamp>/agent_payload.md`.

## plot_imu_by_state.py

Genera grafica PNG de `yaw`, `roll` y `pitch` de IMU por fase de maquina de estado, coloreada por `state`.

Ejecucion (usa la ultima corrida por defecto):

    python3 tools/debug/plot_imu_by_state.py

Ejecucion sobre una corrida concreta:

    python3 tools/debug/plot_imu_by_state.py --run debug_runs/20260512_074310Z

Salida:

- `debug_runs/<timestamp>/imu_by_state_yaw_roll_pitch.png`
- `debug_runs/<timestamp>/imu_plot_summary.txt`
