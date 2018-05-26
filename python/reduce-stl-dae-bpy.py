
import os
import sys
import pdb

import bpy
import pdb

argv = sys.argv
argv = argv[argv.index("--") + 1:] # get all args after "--"

if len(argv) < 3 & os.path.isdir(argv[0]) == False:
  print('too few arguments, USAGE: path/to/stl outputpath/to/dae ratio[0-1]')
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
			dae_file = os.path.join(outputpath, os.path.splitext(f)[0]) + ".dae"

			candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

			for object_name in candidate_list:
				bpy.data.objects.remove(bpy.data.objects[object_name], True)



			bpy.ops.import_mesh.stl(filepath=mesh_file) 
			bpy.ops.object.mode_set(mode='OBJECT')
			bpy.ops.object.mode_set(mode='EDIT')

			for obj in bpy.context.selected_objects:
				obj.name = "node"

			obj = bpy.context.active_object

			bpy.ops.object.modifier_add(type='DECIMATE')
			bpy.data.objects['node'].modifiers["Decimate"].ratio=float(ratio)
			bpy.ops.object.modifier_apply(modifier='DECIMATE')

			bpy.ops.wm.collada_export(filepath=dae_file)