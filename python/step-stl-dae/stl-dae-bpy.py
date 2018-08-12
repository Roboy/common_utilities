
import os
import sys
import pdb

import bpy
import pdb

argv = sys.argv
argv = argv[argv.index("--") + 1:]

if len(argv) < 2 & os.path.isfile(argv[0]) == False:
  print('too few arguments, USAGE: blender --background --python stl-dae-bpy.py -- path/to/stl outputpath/to/dae')
else:
	inputpath = argv[0]
	outputpath = argv[1]


if inputpath.endswith('.stl') :
	mesh_file = inputpath
	dae_file = os.path.join(outputpath, os.path.splitext(os.path.basename(mesh_file))[0]) + ".dae"

	bpy.ops.object.select_all(action='SELECT')
	bpy.ops.object.delete()

	bpy.ops.import_mesh.stl(filepath=mesh_file) 

	bpy.ops.object.select_all(action='SELECT')
	bpy.ops.wm.collada_export(filepath=dae_file)