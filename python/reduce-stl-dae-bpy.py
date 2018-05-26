
import os
import sys
import pdb

import bpy
import pdb

argv = sys.argv
# argv = argv[argv.index("--") + 1:] # get all args after "--"

if len(argv) < 3 & os.path.isdir(argv[0]) == False:
  print('too few arguments, USAGE: path/to/stl outputpath/to/dae ratio[0-1]')
else:
	print('input folder: ' + argv[4])
	print('ouput folder: ' + argv[5])
	print('faces ratio: ' + argv[6])
	outputpath = argv[5]
	inputpath = argv[4]
	ratio = argv[6]

for root, dirs, files in os.walk(inputpath):
	for f in files:
		if f.endswith('.stl') or f.endswith('.STL') :

			mesh_file = os.path.join(inputpath, f)
			dae_file = os.path.join(outputpath, os.path.splitext(f)[0]) + ".dae"

			bpy.ops.object.select_all(action='SELECT')
			bpy.ops.object.delete()

			bpy.ops.import_mesh.stl(filepath=mesh_file) # change this line

			bpy.ops.object.mode_set(mode='OBJECT')

			bpy.ops.object.mode_set(mode='EDIT')

			bpy.ops.mesh.select_all(action='SELECT')

			for obj in bpy.context.selected_objects:
				obj.name = "node"

			candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

			obj = bpy.data.objects[0]
			pdb.set_trace()
			obj.hide = False
			obj.select = True

			bpy.ops.object.modifier_add(type='DECIMATE')
			bpy.data.objects['node'].modifiers["Decimate"].ratio=float(ratio)
			bpy.ops.object.modifier_apply(modifier='DECIMATE')

			# bpy.ops.object.select_all(action='SELECT')
			bpy.ops.wm.collada_export(filepath=dae_file)