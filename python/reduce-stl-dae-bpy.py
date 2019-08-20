#! /usr/bin/python2.7
'''
Reduces all stl meshes in the folder and converts all of them to dae.
Usage:
blender --background --python reduce-stl-dae-bpy.py -- path/to/stl outputpath/to/dae ratio[0-1] (optional export_collada[0/1]) (optional collsion ratio[0..1])

If you specify a collision_ratio the reduce mesh function will be used to create a collision meshes, else axis-aligned bounding boxes will be used.

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


def reduce_mesh(inmesh, outmesh, ratio):
    name = os.path.splitext(os.path.basename(inmesh))[0].replace("_", " ").title()
    bpy.ops.import_mesh.stl(filepath=inmesh)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.data.objects[name].select = True
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.context.scene.objects.active = bpy.data.objects[name]
    bpy.ops.object.modifier_add(type='DECIMATE')
    bpy.data.objects[name].modifiers["Decimate"].ratio = float(ratio)
    bpy.ops.object.modifier_apply(modifier='DECIMATE')
    if outmesh.endswith(".dae"):
        bpy.ops.wm.collada_export(filepath=outmesh, apply_modifiers=True)
    else:
        bpy.ops.export_mesh.stl(filepath=outmesh, check_existing=True, use_mesh_modifiers=True, use_selection=True)
        print("Exported simplified visual to {}".format(outmesh))


def axis_aligned_bounding_box(inmesh, outmesh):
    name = os.path.splitext(os.path.basename(inmesh))[0].replace("_", " ").title()
    bpy.ops.import_mesh.stl(filepath=inmesh)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.data.objects[name].select = True
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.context.scene.objects.active = bpy.data.objects[name]
    bpy.ops.object.mode_set(mode='OBJECT')
    faces = [(0, 1, 2, 3),
             (4, 7, 6, 5),
             (0, 4, 5, 1),
             (1, 5, 6, 2),
             (2, 6, 7, 3),
             (4, 0, 3, 7)]
    verts = bpy.data.objects[name].bound_box

    box = bpy.data.meshes.new("Mesh")
    box.from_pydata(verts, [], faces)
    box.update()

    bpy.data.objects[name].select = True
    bpy.ops.object.delete()

    scene = bpy.context.scene
    obj = bpy.data.objects.new("Object", box)
    scene.objects.link(obj)

    scene.objects.active = obj
    obj.select = True

    if outmesh.endswith(".dae"):
        bpy.ops.wm.collada_export(filepath=outmesh)
    else:
        bpy.ops.export_mesh.stl(filepath=outmesh, use_selection=True)
        print("Exported collision to {}".format(collision_file))
    print()


argv = sys.argv
argv = argv[argv.index("--") + 1:] # get all args after "--"

if len(argv) < 3 & os.path.isdir(argv[0]) == False:
    print("""too few arguments.
     USAGE: blender --background --python reduce-stl-dae-bpy.py -- path/to/stl outputpath/to/dae ratio[0-1] (optional: export_collada[0/1]) (optional: collision_ratio[0..1])
     
     If you provide the collision_ratio, reduced meshes will be used for collision files, else bounding boxes will be used.""")
    raise ValueError
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

if len(argv) >= 5:
    collision_ratio = float(argv[4])
else:
    collision_ratio = None

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

            # Write reduced visual mesh
            reduce_mesh(mesh_file, visual_file, ratio)

            if collision_ratio is None:
                axis_aligned_bounding_box(mesh_file, collision_file)
            else:
                reduce_mesh(mesh_file, collision_file, collision_ratio)
