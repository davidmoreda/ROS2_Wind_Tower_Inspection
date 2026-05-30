#!/usr/bin/env python3
"""Valida nav2_params.yaml y muestra los topes de velocidad tras el cambio."""
import yaml

P = ("/home/dani/ROS2_Wind_Tower_Inspection/ros2_ws/src/"
     "wind_tower_bringup/config/nav2_params.yaml")
d = yaml.safe_load(open(P, encoding="utf-8"))

fp = d["controller_server"]["ros__parameters"]["FollowPath"]
vs = d["velocity_smoother"]["ros__parameters"]

print("== Controlador DWB (FollowPath) ==")
print(f"  max_vel_x     = {fp['max_vel_x']}   (antes 0.50)")
print(f"  max_speed_xy  = {fp['max_speed_xy']}   (antes 0.50)")
print(f"  max_vel_theta = {fp['max_vel_theta']}   (sin cambios)")
print(f"  acc_lim_x     = {fp['acc_lim_x']}   (sin cambios - tuning rampa)")
print("== velocity_smoother ==")
print(f"  max_velocity  = {vs['max_velocity']}   (antes [0.60, 0.0, 0.9])")
print(f"  min_velocity  = {vs['min_velocity']}")
print(f"  max_accel     = {vs['max_accel']}   (sin cambios)")
print("[ok] nav2_params.yaml es YAML valido")
