"""Validate SDF XML parses correctly."""
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1]
try:
    tree = ET.parse(path)
    root = tree.getroot()
    n_models = len(root.findall('.//model'))
    n_actors = len(root.findall('.//actor'))
    n_lights = len(root.findall('.//light'))
    print(f'SDF OK: {n_models} models, {n_actors} actors, {n_lights} lights')
except ET.ParseError as e:
    print(f'SDF PARSE ERROR: {e}')
    sys.exit(1)
