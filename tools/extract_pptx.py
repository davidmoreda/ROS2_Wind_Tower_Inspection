#!/usr/bin/env python3
"""Extrae el texto de cada diapositiva de la presentacion (en orden real)."""
import glob
import zipfile
from xml.etree import ElementTree as ET

A = '{http://schemas.openxmlformats.org/drawingml/2006/main}'
R = '{http://schemas.openxmlformats.org/officeDocument/2006/relationships}'
P = '{http://schemas.openxmlformats.org/presentationml/2006/main}'

DL = '/mnt/c/Users/danie/Downloads'
cands = glob.glob(f'{DL}/*Torres*.pptx') or glob.glob(f'{DL}/*.pptx')
path = cands[0]
print('FILE:', path)

z = zipfile.ZipFile(path)

pres = ET.fromstring(z.read('ppt/presentation.xml'))
rels = ET.fromstring(z.read('ppt/_rels/presentation.xml.rels'))
relmap = {rel.get('Id'): rel.get('Target') for rel in rels}

order = []
for sld in pres.find(P + 'sldIdLst').findall(P + 'sldId'):
    tgt = relmap.get(sld.get(R + 'id'), '')
    if not tgt:
        continue
    tgt = tgt.lstrip('/').replace('../', '')
    if not tgt.startswith('ppt/'):
        tgt = 'ppt/' + tgt
    order.append(tgt)


def paragraphs(xml_bytes):
    root = ET.fromstring(xml_bytes)
    out = []
    for p in root.iter(A + 'p'):
        line = ''.join(t.text for t in p.iter(A + 't') if t.text).strip()
        out.append(line)
    return out


def notes_for(slide_path):
    base = slide_path.split('/')[-1]
    rels_name = f'ppt/slides/_rels/{base}.rels'
    if rels_name not in z.namelist():
        return []
    rr = ET.fromstring(z.read(rels_name))
    for rel in rr:
        if 'notesSlide' in (rel.get('Target') or ''):
            t = rel.get('Target').lstrip('/').replace('../', '')
            if not t.startswith('ppt/'):
                t = 'ppt/' + t
            if t in z.namelist():
                return [ln for ln in paragraphs(z.read(t)) if ln]
    return []


print(f'TOTAL SLIDES: {len(order)}')
for i, sp in enumerate(order, 1):
    data = z.read(sp) if sp in z.namelist() else z.read('ppt/' + sp.split('ppt/')[-1])
    print(f'\n===== Slide {i}  ({sp.split("/")[-1]}) =====')
    for ln in paragraphs(data):
        if ln:
            print('  •', ln)
    notes = notes_for(sp)
    if notes:
        print('  [NOTAS]:')
        for ln in notes:
            print('    >', ln)
