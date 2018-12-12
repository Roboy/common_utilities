#! /usr/bin/python2.7
'''
Reduces all stl meshes in the folder and converts all of them to dae.
Usage:
blender --background --python reduce-stl-dae-bpy.py -- path/to/stl outputpath/to/dae ratio[0-1]

using blender>=2.8
'''
import os
import sys
import pdb

import bpy
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
argv = argv[argv.index("--") + 1:] # get all args after "--"

if len(argv) < 3 & os.path.isdir(argv[0]) == False:
    print('too few arguments, USAGE: blender --background --python reduce-stl-dae-bpy.py -- path/to/stl outputpath/to/dae ratio[0-1] (optional: export_collada[0/1])')
else:
    print('input folder: ' + argv[0])
    print('ouput folder: ' + argv[1])
    print('faces ratio: ' + argv[2])
    outputpath = argv[1]
    inputpath = argv[0]
    ratio = argv[2]

export_collada = False
try:
    export_collada = str_to_bool(argv[3])
    if export_collada:
        print('Exporting Collada files')
    else:
        print('Exporting STL files')
except:
    print('Defaulting to STL export')

collisions_path = outputpath + "collisions"
visuals_path = outputpath + "visuals"

if not os.path.exists(collisions_path):
    os.makedirs(collisions_path)

if not os.path.exists(visuals_path):
    os.makedirs(visuals_path)

for root, dirs, files in os.walk(inputpath):
    if "collisions" in root: continue
    if "visuals" in root: continue
    for f in files:
        if f.endswith('.stl') or f.endswith('.STL') :
            # reduce the ammount of vertices for the mesh
            mesh_file = os.path.join(root, f)
            name = os.path.splitext(f)[0].replace("_", " ").title()
            visual_file = os.path.join(visuals_path, os.path.splitext(f)[0])
            collision_file = os.path.join(collisions_path, os.path.splitext(f)[0])
            if export_collada:
                visual_file += ".dae"
                collision_file += ".dae"
            else:
                visual_file += ".stl"
                collision_file += ".stl"

            candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

            for object_name in candidate_list:
                bpy.data.objects.remove(bpy.data.objects[object_name], True)

            bpy.ops.import_mesh.stl(filepath=mesh_file)

            bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.data.objects[name].select = True

            bpy.ops.mesh.select_all(action='SELECT')
            bpy.context.scene.objects.active = bpy.data.objects[name]
            obj = bpy.context.active_object

            # bpy.context.scene.objects.link(bpy.data.objects[name])

            bpy.ops.object.modifier_add(type='DECIMATE')
            bpy.data.objects[name].modifiers["Decimate"].ratio=float(ratio)
            bpy.ops.object.modifier_apply(modifier='DECIMATE')

            if export_collada:
                bpy.ops.wm.collada_export(filepath=visual_file, apply_modifiers=True)
            else:
                bpy.ops.export_mesh.stl(filepath=visual_file, check_existing=True, use_mesh_modifiers=True, use_selection=True)
                print("Exported simplified visual to {}".format(visual_file))

            # generate collision meshes using bounding box
            bpy.ops.object.mode_set(mode='OBJECT')
            faces = [(0, 1, 2, 3),
               (4, 7, 6, 5),
               (0, 4, 5, 1),
               (1, 5, 6, 2),
               (2, 6, 7, 3),
               (4, 0, 3, 7)]
            verts = bpy.data.objects[name].bound_box

            collision_mesh = bpy.data.meshes.new("Mesh")
            collision_mesh.from_pydata(verts, [], faces)
            collision_mesh.update()

            bpy.data.objects[name].select = True
            bpy.ops.object.delete()

            scene = bpy.context.scene
            obj = bpy.data.objects.new("Object", collision_mesh)
            scene.objects.link(obj)

            scene.objects.active = obj
            obj.select = True

            if export_collada:
                bpy.ops.wm.collada_export(filepath=collision_file)
            else:
                bpy.ops.export_mesh.stl(filepath=collision_file, use_selection=True)
                print("Exported collision to {}".format(collision_file))
            print()
