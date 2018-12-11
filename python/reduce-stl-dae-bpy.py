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
import bmesh
import pdb

argv = sys.argv
argv = argv[argv.index("--") + 1:] # get all args after "--"

if len(argv) < 3 & os.path.isdir(argv[0]) == False:
  print('too few arguments, USAGE: blender --background --python reduce-stl-dae-bpy.py -- path/to/stl outputpath/to/dae ratio[0-1]')
else:
	print('input folder: ' + argv[0])
	print('ouput folder: ' + argv[1])
	print('faces ratio: ' + argv[2])
	outputpath = argv[1]
	inputpath = argv[0]
	ratio = argv[2]

for root, dirs, files in os.walk(inputpath):
    for f in files:
        if f.endswith('.stl') or f.endswith('.STL') :
            mesh_file = os.path.join(inputpath, f)
            name = os.path.splitext(f)[0].replace("_", " ").title()
            dae_file = os.path.join(outputpath, os.path.splitext(f)[0]) + ".dae"
            collision_file = os.path.join(outputpath, "collision_" + os.path.splitext(f)[0]) + ".dae"

            candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

            for object_name in candidate_list:
            	bpy.data.objects.remove(bpy.data.objects[object_name], True)

            bpy.ops.import_mesh.stl(filepath=mesh_file)

            # bpy.ops.object.mode_set(mode='OBJECT')
            # bpy.ops.object.mode_set(mode='EDIT')
            bpy.data.objects[name].select = True

            # bpy.ops.object.modifier_add(type='DECIMATE')
            # bpy.data.objects[name].modifiers["Decimate"].ratio=float(ratio)
            # bpy.ops.object.modifier_apply(modifier='DECIMATE')
            # bound_box = bpy.context.active_object

            # print(bpy.data.objects[name].bound_box)
            # pdb.set_trace()




            collision_mesh = bpy.context.object.data
            bm = bmesh.new()

            faces = [(0, 1, 2, 3),
                     (4, 7, 6, 5),
                     (0, 4, 5, 1),
                     (1, 5, 6, 2),
                     (2, 6, 7, 3),
                     (4, 0, 3, 7)]
            verts = bpy.data.objects[name].bound_box

            location = bpy.data.objects[name].location
            rotation_euler = bpy.data.objects[name].rotation_euler
            scale = bpy.data.objects[name].scale

            # pdb.set_trace()
            print(location)
            print(rotation_euler)
            print(scale)

            collision_mesh = bpy.data.meshes.new("Mesh")
            collision_mesh.from_pydata(verts, [], faces)
            collision_mesh.update()
            # bm.to_mesh(collision_mesh)
            # bm.free()

            bpy.data.objects[name].select = True
            bpy.ops.object.delete()

            scene = bpy.context.scene
            obj = bpy.data.objects.new("Object", collision_mesh)
            scene.objects.link(obj)

            scene.objects.active = obj
            obj.select = True
            obj.location = location
            obj.scale = scale
            obj.rotation_euler = rotation_euler


            bpy.ops.wm.collada_export(filepath=collision_file)
            # bound_box.location = bpy.ops.objects[name].location
            # bound_box.rotation_euler = bpy.ops.objects[name].rotation_euler


            # bpy.ops.wm.collada_export(filepath=dae_file, apply_modifiers=True)


# for root, dirs, files in os.walk(inputpath):
# 	for f in files:
# 		if f.endswith('.stl') or f.endswith('.STL') :
#
# 			mesh_file = os.path.join(inputpath, f)
# 			name = os.path.splitext(f)[0].replace("_", " ").title()
# 			dae_file = os.path.join(outputpath, os.path.splitext(f)[0]) + ".dae"
#
# 			candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]
#
# 			for object_name in candidate_list:
# 				bpy.data.objects.remove(bpy.data.objects[object_name], True)
#
# 			bpy.ops.import_mesh.stl(filepath=mesh_file)
#
# 			bpy.ops.object.mode_set(mode='OBJECT')
#             bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
#             bpy.ops.mesh.primitive_cube_add()
