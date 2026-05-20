# CONTROLLERS_IMPLEMENTED — Controladores de control actuales (IMPLEMENTADO)

> Documento vivo para entender los controladores usados hoy en `state_machine_node.py`. Solo describe lo que esta implementado en codigo. Todo lo de Nav2 queda fuera y se considera `PROPUESTO / NO IMPLEMENTADO`.

---

## 1. Objetivo

Este documento responde, de forma trazable, a tres preguntas:

1. que controlador actua en cada fase de la maquina de estado,
2. que pesos (`Kp`, `Ki`) usa,
3. que variable controla y como se satura.

La meta es mejorar observabilidad antes de tocar tuning.

---

## 2. Resumen rapido por fase

| Fase de maquina de estado | Controladores activos | Salida principal |
|---|---|---|
| `AXIAL_SCAN` | `axial_heading_pi`, `axial_lateral_pi`, `axial_sum` | `cmd_vel.angular.z` y `cmd_vel.linear.x` |
| `ROTATE_TO_TANGENTIAL` | `yaw_pi` (+ correccion lateral P durante rotacion) | `cmd_vel.angular.z` |
| `INDEX_TUBE` | `index_feedforward`, `index_lateral_pi`, `index_yaw_hold` (`yaw_pi`), `index_sum` | `cmd_vel.linear.x`, `cmd_vel.angular.z`, `/turner/cmd_vel` |
| `ROTATE_TO_AXIAL` | `yaw_pi` (+ correccion lateral P durante rotacion) | `cmd_vel.angular.z` |
| `ALIGN_TO_BOTTOM_LANE` | `align_heading_p`, `align_lateral_pi` | `cmd_vel.angular.z` y `cmd_vel.linear.x` |
| `RECOVER_BOTTOM_AFTER_INDEX` | `align_heading_p`, `align_lateral_pi` | `cmd_vel.angular.z` y `cmd_vel.linear.x` |
| `REALIGN_AXIAL_YAW` | `yaw_pi` (+ correccion lateral P durante rotacion) | `cmd_vel.angular.z` |
| `DESCEND_TO_BOTTOM_LANE` | `descend_heading_p`, `descend_lateral_pi` | `cmd_vel.angular.z` y `cmd_vel.linear.x` |

Estados sin lazos PI directos en esta implementacion: `IDLE`, `VERIFY_BOTTOM_LOCK`, `WAIT_SAFE_TO_INDEX`, `VERIFY_INDEXED_POSITION`, `FINISH`, `ERROR_RECOVERY`.

---

## 3. Pesos actuales (`Kp`, `Ki`) declarados

Valores por defecto declarados en `state_machine_node.py` (pueden sobreescribirse por parametros de launch):

| Parametro | Default | Uso |
|---|---:|---|
| `control.k_axial_heading_p` | `0.8` | P de heading en `AXIAL_SCAN` |
| `control.k_axial_heading_i` | `0.0` | I de heading en `AXIAL_SCAN` |
| `control.k_axial_lateral_p` | `1.0` | P lateral (error desde roll) en `AXIAL_SCAN` |
| `control.k_axial_lateral_i` | `0.15` | I lateral en `AXIAL_SCAN` |
| `control.k_align_heading_p` | `0.35` | P de heading en `ALIGN_TO_BOTTOM_LANE` / `RECOVER_BOTTOM_AFTER_INDEX` / `DESCEND_TO_BOTTOM_LANE` |
| `control.k_align_lateral_p` | `2.0` | P lateral en alineacion/descenso |
| `control.k_align_lateral_i` | `0.0` | I lateral en alineacion/descenso |
| `control.k_rotate_yaw_p` | `1.0` | P de yaw en rotaciones |
| `control.k_rotate_yaw_i` | `0.0` | I de yaw en rotaciones |
| `control.k_rotate_lateral_p` | `0.5` | P correctivo lateral durante rotacion |
| `control.k_index_lateral_p` | `0.0` | P lateral durante `INDEX_TUBE` |
| `control.k_index_lateral_i` | `0.0` | I lateral durante `INDEX_TUBE` |

Ganancias no PI pero relevantes en `INDEX_TUBE`:

| Parametro | Default | Uso |
|---|---:|---|
| `control.index_compensation_gain` | `1.0` | feedforward axial desde velocidad superficial del tubo |
| `control.index_compensation_sign` | `1.0` | signo base de compensacion |

---

## 4. Detalle por controlador

### 4.1 `axial_heading_pi` (IMPLEMENTADO)

- Fase: `AXIAL_SCAN`.
- Target: `target_yaw_rad` (normalmente `self._desired_yaw_rad`).
- Medida: `odom.yaw_rad`.
- Error: `wrap_pi(target - measured)`.
- Ley: `u_heading = Kp*error + Ki*integral`.
- Saturacion: no satura solo este termino; entra en suma final `axial_sum` y alli se clampa `cmd_vel.angular.z` por `control.max_angular_z_rad_s`.
- Integrador: si, unidad `rad*s`, clamp por `control.max_axial_heading_integral_deg_s` (convertido a rad/s acumulado).

### 4.2 `axial_lateral_pi` (IMPLEMENTADO)

- Fase: `AXIAL_SCAN`.
- Target: `lateral_error_rad = 0`.
- Medida: `signed_bottom_error_rad` (derivado de `roll_deg` desde `stability_monitor`).
- Error: lateral firmado en radianes.
- Ley: `u_lateral = sign*(Kp*error + Ki*integral)`.
- Saturacion: participa en `axial_sum`.
- Integrador: si, unidad `rad*s`, clamp por `control.max_axial_lateral_integral_deg_s`.

### 4.3 `axial_sum` (IMPLEMENTADO)

- Fase: `AXIAL_SCAN`.
- Suma de contribuciones heading+lateral.
- Modos:
  - `yaw_priority`: si `|heading_error_deg| > control.axial_yaw_priority_deg`, aplica solo heading.
  - `coupled`: combina heading+lateral.
- Salida final: `cmd_vel.angular.z` con clamp a `[-max_angular_z_rad_s, +max_angular_z_rad_s]`.

### 4.4 `yaw_pi` (IMPLEMENTADO)

- Fases: `ROTATE_TO_TANGENTIAL`, `ROTATE_TO_AXIAL`, `REALIGN_AXIAL_YAW`.
- Tambien se reutiliza en `INDEX_TUBE` como `index_yaw_hold`.
- Target: yaw objetivo de la fase (`tangential` o `axial`).
- Medida: `odom.yaw_rad`.
- Error: `wrap_pi(target - measured)`.
- Ley: `u = Kp*error + Ki*integral`.
- Saturacion: clamp por `control.rotate_angular_speed_rad_s`.
- Integrador: si, unidad `rad*s`, clamp por `control.max_rotate_yaw_integral_deg_s`.

### 4.5 Correccion lateral durante rotacion (IMPLEMENTADO)

- Funcion: `_rotate_lateral_correction()`.
- Tipo: P puro (no PI).
- Ley: `u_corr = sign * k_rotate_lateral_p * signed_bottom_error_rad`.
- Se suma a `yaw_pi` antes del clamp final de rotacion.

### 4.6 `align_heading_p` y `align_lateral_pi` (IMPLEMENTADO)

- Fases: `ALIGN_TO_BOTTOM_LANE` y `RECOVER_BOTTOM_AFTER_INDEX`.
- `align_heading_p`:
  - target: yaw de realineacion,
  - medida: `odom.yaw_rad`,
  - ley: P puro (`k_align_heading_p`).
- `align_lateral_pi`:
  - target: lateral cero,
  - medida: error lateral firmado,
  - ley: `sign*(Kp*error + Ki*integral)` con `k_align_lateral_*`.
- Seleccion de modo interno:
  - `yaw_reacquire`, `yaw_capture(_forced)`: prioriza heading,
  - `lateral_capture(_forced)`: prioriza recuperar generatriz inferior,
  - `hold_ready`: mantiene cero.
- Saturacion: clamp por `control.align_max_angular_z_rad_s`.

### 4.7 `descend_heading_p` y `descend_lateral_pi` (IMPLEMENTADO)

- Fase: `DESCEND_TO_BOTTOM_LANE`.
- Estructura equivalente a alineacion, con logica de modos de descenso:
  - `descend_bottom_lane`,
  - `descend_yaw_trim`,
  - `descend_hold_ready`.
- Pesos: comparten `k_align_heading_p`, `k_align_lateral_p`, `k_align_lateral_i`.
- Saturacion: clamp por `control.align_max_angular_z_rad_s`.

### 4.8 `index_feedforward`, `index_lateral_pi`, `index_sum` (IMPLEMENTADO)

- Fase: `INDEX_TUBE`.
- `index_feedforward`:
  - `surface_speed_mps = tube_radius_m * turner_velocity_rad_s`,
  - `u_ff = compensation_sign * index_compensation_gain * surface_speed_mps`.
- `index_lateral_pi`:
  - target lateral cero,
  - medida lateral en grados (`roll_deg`),
  - ley: `sign*(Kp*error + Ki*integral)` con `k_index_lateral_*`.
  - nota: integrador explicito en `deg*s`.
- `index_sum`:
  - suma feedforward + lateral,
  - salida `cmd_vel.linear.x` con clamp por `control.max_index_linear_speed_mps`.
- Yaw durante indexado: `yaw_pi` se ejecuta como `index_yaw_hold` para mantener orientacion tangencial.

---

## 5. Resets de integrador y estado interno

Resets principales al cambiar de fase:

- `_reset_yaw_control()` al entrar en rotaciones (`ROTATE_TO_TANGENTIAL`, `ROTATE_TO_AXIAL`) y en `reset_mission_runtime`.
- `_reset_axial_control()` al entrar en `AXIAL_SCAN` y en `reset_mission_runtime`.
- `_reset_align_control()` al entrar en `ALIGN_TO_BOTTOM_LANE`, `RECOVER_BOTTOM_AFTER_INDEX`, `DESCEND_TO_BOTTOM_LANE` y en `reset_mission_runtime`.

Esto evita arrastrar integrales entre fases con dinamicas distintas.

---

## 6. Saturaciones y limites relevantes

| Variable de salida | Limite |
|---|---|
| `cmd_vel.angular.z` en axial | `control.max_angular_z_rad_s` |
| `cmd_vel.angular.z` en align/descend | `control.align_max_angular_z_rad_s` |
| `cmd_vel.angular.z` en rotate/yaw_pi | `control.rotate_angular_speed_rad_s` |
| `cmd_vel.linear.x` en `INDEX_TUBE` | `control.max_index_linear_speed_mps` |

Integrales con clamp:

- axial heading (`rad*s`),
- axial lateral (`rad*s`),
- rotate yaw (`rad*s`),
- align lateral (`rad*s`),
- index lateral (`deg*s`, sin clamp dedicado explicito en codigo actual).

---

## 7. Observabilidad para validar cada lazo

Topic principal:

- `/inspection/control_debug` (`std_msgs/String` JSON)

Campos ya publicados por tick (cuando aplica):

- `state`,
- `controllers[]` con `controller`, `target`, `measured`, `error`, `p`, `i`, `u_raw`, `u_sat`, `integrator`, `dt`, `saturated`,
- `cmd_linear_x`, `cmd_angular_z`,
- flags de safety (`bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube`),
- `angles_deg` y `targets_deg`.

Comandos manuales sugeridos:

    ros2 topic echo /inspection/state_text
    ros2 topic echo /inspection/control_debug
    ros2 topic echo /inspection/stability --once --full-length

---

## 8. Limites de este documento

- No cubre planificadores/controladores Nav2 (`RPP`, `MPPI`): `PROPUESTO / NO IMPLEMENTADO`.
- No redefine tuning recomendado; solo describe implementacion actual.
- Si cambian parametros en launch/CLI, los defaults de este documento dejan de coincidir con runtime.

## 9. Fuente de heading configurable (nuevo en launcher)

El codigo de control de heading sigue leyendo `yaw_rad` desde el topic de odometria configurado en `state_machine.odom_topic`.

- Default actual: `/robot/platform/odom/filtered`.
- Modo recomendado IMU+Kalman: lanzar `inspection.launch.py` con EKF y usar `state_machine_odom_topic:=/inspection/odom_kf`.

Importante: esto cambia la **fuente de yaw** para PI de heading, pero no cambia la logica lateral/safety basada en IMU de `stability_monitor`.

---

## 10. Referencias de codigo

- `ros2_ws/src/wind_tower_inspection_behaviour/wind_tower_inspection_behaviour/state_machine_node.py`
- `ros2_ws/src/wind_tower_inspection_behaviour/launch/inspection.launch.py`
- `docs/architecture/CURRENT_ARCHITECTURE.md`
- `docs/agent/CONTROL_DEBUG_BRIEF.md`
