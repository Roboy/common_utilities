import sys, bpy
from os.path import isfile, join

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

if len(argv) < 3 & isfile(argv[0]) == False:
  print('too few arguments, USAGE: path/to/stl outputpath/to/stl ratio[0-1] scale[0.01-1000]')
else:
  print('input file: ' + argv[0])
  print('ouput file: ' + argv[1])
  print('faces ratio: ' + argv[2])

  bpy.ops.object.mode_set(mode='OBJECT')

  bpy.ops.object.select_all()

  # gather list of items of interest.
  candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

  # select them only.
  for object_name in candidate_list:
    bpy.data.objects[object_name].select = True

  # remove all selected.
  bpy.ops.object.delete()

  # remove the meshes, they have no users anymore.
  for item in bpy.data.meshes:
    bpy.data.meshes.remove(item)

  bpy.ops.import_mesh.stl(filepath=argv[0])

  bpy.ops.object.mode_set(mode='OBJECT')
  bpy.ops.object.mode_set(mode='EDIT')

  bpy.ops.mesh.select_all(action='SELECT')
  for obj in bpy.context.selected_objects:
    obj.name = "node"

  candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

  obj = bpy.context.active_object

  bpy.ops.object.modifier_add(type='DECIMATE')
  bpy.data.objects['node'].modifiers["Decimate"].ratio=float(argv[2])
  bpy.ops.object.modifier_apply(modifier='DECIMATE')

  bpy.ops.export_mesh.stl(filepath=argv[1], check_existing=True, use_mesh_modifiers=True, global_scale=float(argv[3]))
