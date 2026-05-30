#!/usr/bin/env python3
"""Reduce las personas del mundo defects_actors de 12 a 4, REPARTIDAS por los
cuatro cuadrantes (SO, NO, SE, NE).

  - Conserva actor_1 (oeste-sur) y actor_2 (oeste-norte) tal cual.
  - Reorienta actor_3 (sur) y actor_4 (norte) — y sus capsulas person_col_3/4 —
    al pasillo ESTE (x: -6.9 -> 6.9), para que las 4 queden repartidas.
  - Elimina actor_5..actor_12 y person_col_5..person_col_12.

Edicion guardada: si las lineas ancla no coinciden, ABORTA sin escribir.
"""
from pathlib import Path
import xml.dom.minidom as minidom

P = Path("/home/dani/ROS2_Wind_Tower_Inspection/ros2_ws/src/"
         "wind_tower_simulation/worlds/wind_tower_world_defects_actors.sdf")
lines = P.read_text(encoding="utf-8").splitlines(keepends=True)
n0 = len(lines)


def L(n):  # linea 1-indexada
    return lines[n - 1]


# --- Anclas de seguridad (deben coincidir con el fichero actual) ---
assert '<actor name="actor_3">'        in L(3362), L(3362)
assert '<actor name="actor_4">'        in L(3384), L(3384)
assert '<actor name="actor_5">'        in L(3406), L(3406)
assert '<actor name="actor_12">'       in L(3560), L(3560)
assert '</actor>'                      in L(3581), L(3581)
assert '<pose>-6.9 -18.0'              in L(3643), L(3643)   # person_col_3
assert '<pose>-6.9 2.0'               in L(3672), L(3672)   # person_col_4
assert '<model name="person_col_5">'   in L(3700), L(3700)
assert '<model name="person_col_12">'  in L(3903), L(3903)
assert '</model>'                      in L(3930), L(3930)

# 1) Reorientar actor_3 (sur) y actor_4 (norte) al ESTE: x -6.9 -> 6.9
for i in range(3362 - 1, 3405):        # lineas 3362..3405 (bloques actor_3 y actor_4)
    if "-6.9" in lines[i]:
        lines[i] = lines[i].replace("-6.9", "6.9")

# 2) Reorientar capsulas person_col_3 (3643) y person_col_4 (3672) al ESTE
lines[3643 - 1] = lines[3643 - 1].replace("-6.9", "6.9")
lines[3672 - 1] = lines[3672 - 1].replace("-6.9", "6.9")

# 3) Borrar bloques sobrantes (rango superior primero para no invalidar indices)
del lines[3699 - 1:3930]   # comentario + person_col_5 .. person_col_12
del lines[3406 - 1:3581]   # actor_5 .. actor_12

text = "".join(lines)

# 4) Validar XML bien formado antes de escribir
minidom.parseString(text)

P.write_text(text, encoding="utf-8")
print(f"[ok] {P.name}: {n0} -> {len(lines)} lineas (XML valido)")
