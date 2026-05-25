#!/usr/bin/env python3
import requests
import os
from pathlib import Path

API_KEY    = "sfoh8lecp4kiqvEU7De6"
WORKSPACE  = "javiers-workspace-q8mnr"
PROJECT    = "ros2_wind_tower_inspection-bjmhc"
IMAGES_DIR = Path("/home/javie/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/roboflow_v2/images")

images = sorted(IMAGES_DIR.rglob("*.jpg")) + sorted(IMAGES_DIR.rglob("*.png"))
print(f"Subiendo {len(images)} imágenes a {WORKSPACE}/{PROJECT} ...\n")

ok = 0
fail = 0
for i, img_path in enumerate(images, 1):
    with open(img_path, "rb") as f:
        r = requests.post(
            f"https://api.roboflow.com/dataset/{PROJECT}/upload",
            params={"api_key": API_KEY},
            files={"file": (img_path.name, f, "image/jpeg")},
        )
    data = r.json()
    if data.get("success"):
        ok += 1
        print(f"[{i}/{len(images)}] OK  {img_path.name}")
    else:
        fail += 1
        print(f"[{i}/{len(images)}] FAIL {img_path.name} → {data}")

print(f"\nListo: {ok} subidas, {fail} errores")
