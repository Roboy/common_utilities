import sys, bpy
import os
from os.path import isfile, join


def ensure_dirs(file_path):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

argv = sys.argv
argv = argv[1:]  # get all args after "--"

if len(argv) < 3:
    print('too few arguments, USAGE: path/to/stl outputpath/to/stl ratio[0-1] scale[0.001-1000]')
else:
    print('input file: ' + argv[0])
    print('ouput file: ' + argv[1])
    print('faces ratio: ' + str(argv[2]))
    print('scale: ' + str(argv[3]))

    try:
        # in case no more models are there, this is not possible
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all()
        #delete previous stuff
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
    except RuntimeError:
        pass



    # import all meshes in folder / mesh directly
    if os.path.isdir(argv[0]):
        fileList = [f for f in os.listdir(argv[0]) if f.endswith('.stl')]
        for entry in fileList:
            print('adding: ' + entry)
            bpy.ops.import_mesh.stl(filepath=argv[0] + "/" + entry)
    else:
        bpy.ops.import_mesh.stl(filepath=argv[0])


    # apply decimate modifier for every object in the scene
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        print("decimating " + obj.name + "!!!")
        # select obj
        bpy.context.scene.objects.active = obj
        bpy.ops.object.mode_set(mode='OBJECT')

        #apply modifier
        obj.modifiers.new("Decimate" + obj.name, "DECIMATE")
        obj.modifiers["Decimate" + obj.name].ratio=float(argv[2])
        bpy.ops.object.modifier_apply(modifier="Decimate" + obj.name)

    for obj in bpy.data.objects:
        obj.select = False

    scale = sys.argv[4]
    for obj in bpy.data.objects:
        name = obj.name.replace(' ', '_').lower()
        if obj.type != "MESH":
            continue
        obj.select = True
        print("exporting: " + obj.name)
        obj.scale = (scale, scale, scale)
        filepath = argv[1] + name + ".obj"
        ensure_dirs(filepath)
        bpy.ops.export_scene.obj(filepath=filepath, check_existing=True, use_mesh_modifiers=True, use_selection=True,
                                global_scale=float(argv[3]))
        obj.select = False

        # remove .mtl file#
        os.remove(argv[1] + name + ".mtl")