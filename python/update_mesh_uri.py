#! /usr/bin/python2.7
import sys
import os
import xml.etree.ElementTree as ET
import pdb

def str_to_bool(s):
    s = s.lower()
    if s == 'true' or s == '1':
         return True
    elif s == 'false'  or s == '0':
         return False
    else:
         raise ValueError("Cannot covert {} to a bool".format(s))

argv = sys.argv
xml = None
use_collada_meshes = False
if len(argv) < 2:
    print("USAGE: python update_mesh_uri.py /path/to/xml (optional: use_collada_meshes[0/1])")
    sys.exit()
else:
    xml = argv[1]

try:
    use_collada_meshes = str_to_bool(argv[2])
except:
    pass

replacements = {"collision": "collisions", "visual": "visuals"}

tree = ET.parse(xml)
root = tree.getroot()

if xml.endswith('sdf'):
    for link in root.find('model').findall('link'):
        for key in replacements.iterkeys():
            mesh_uri = link.find(key).find('geometry').find('mesh').find('uri')
            mesh_uri.text = mesh_uri.text.replace('CAD', replacements[key])
            if use_collada_meshes:
                mesh_uri.text = mesh_uri.text.replace('.stl', '.dae')
            print("New mesh URI: " + mesh_uri.text)

elif xml.endswith('urdf'):
    for link in root.findall('link'):
        for key in replacements.iterkeys():
            mesh = link.find(key).find('geometry').find('mesh')
            mesh.set("filename", mesh.get("filename").replace('CAD', replacements[key]))
            if use_collada_meshes:
                mesh.set("filename", mesh.get("filename").replace('.stl', '.dae'))
            print("New mesh URI: " + mesh.get("filename"))

else:
    print("Unknown XML format. I can only parse SDF and URDF")

tree.write(xml)
