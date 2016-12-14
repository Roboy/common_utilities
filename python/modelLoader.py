import bpy
from bpy.props import (StringProperty,
                       PointerProperty,
                       BoolProperty
                       )
from bpy.types import (Panel,
                       Operator,
                       AddonPreferences,
                       PropertyGroup,
                       )
from bpy_extras.io_utils import ImportHelper
import xml.etree.ElementTree
import os

bl_info = {
    "name": "modelLoader for model.sdf files",
    "category": "User Interface",
}

# ------------------------------------------------------------------------
#    ui
# ------------------------------------------------------------------------

sdf_file = "/home/letrend/workspace/roboy-ros-control/src/roboy_models/legs_with_muscles_simplified/model.sdf"
mesh_dir = "/home/letrend/workspace/roboy-ros-control/src/roboy_models/legs_with_muscles_simplified/cad"
collisionPose = False


class FilePath(PropertyGroup):
    sdf_file = StringProperty(
        name="",
        description="Path to sdf file",
        default="",
        maxlen=1024,
        subtype='FILE_PATH')


class DirectoryPath(PropertyGroup):
    mesh_dir = StringProperty(
        name="",
        description="Path to stl directory",
        default="",
        maxlen=1024,
        subtype='DIR_PATH')


class CollisionPose(PropertyGroup):
    collisionPose = BoolProperty(
        name="",
        description="use collision pose",
        default=False)


class OBJECT_PT_my_panel(Panel):
    bl_idname = "OBJECT_PT_modelLoader"
    bl_label = "modelLoader"
    bl_space_type = "VIEW_3D"
    bl_region_type = "TOOLS"
    bl_category = "Tools"
    bl_context = "objectmode"

    def draw(self, context):
        layout = self.layout
        scn = context.scene
        col = layout.column(align=True)
        col.prop(scn.sdf_file, "sdf_file", text="model.sdf")
        col.prop(scn.mesh_dir, "mesh_dir", text="stl directory")
        col.prop(scn.collisionPose, "collisionPose", text="use collision pose")
        col.operator("my.load_meshes_button", text="load meshes")

        global sdf_file
        global mesh_dir
        global collisionPose
        sdf_file = scn.sdf_file.sdf_file
        mesh_dir = scn.mesh_dir.mesh_dir
        collisionPose = scn.collisionPose.collisionPose


# load meshes Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.load_meshes_button"
    bl_label = "Button"

    def execute(self, context):
        global collisionPose
        root = xml.etree.ElementTree.parse(sdf_file).getroot()
        #read in poses of links
        poses = dict()
        getnamefrom = 'link'
        if collisionPose:
            getnamefrom = 'collision'
        #for link in root.iter(getnamefrom):
        for link in root.findall('./model/link'):
            pose = list()
            #skipPose = True
            for posestring in link.iter('pose'):
                pose.clear()
                i = 0
                for num in posestring.text.split(' '):
                    pose.append(float(num))
                    i += 1
                if i != 6:
                    print("ERROR reading pose of " + link.attrib['name'] + " , expected 6 values xyzrpy")
                    continue
                # if collisionPose and #skipPose:
                #     #skipPose = False
                #     continue
                # else:
                break
            name = link.attrib['name']
            if collisionPose:
                name = name[:-10]
            print(name + " with pose: " + str(pose))
            poses[name] = pose
        i = 0
        #poses2 = dict()
        for mesh in poses.keys():
            print("loading " + mesh)
            #meshname = mesh
            #if mesh.endswith('.dae'):
                #bpy.ops.wm.collada_import(filepath=mesh_dir+mesh)
                # bpy.ops.import_mesh.stl(files=[{"name": mesh }], directory=mesh_dir)
                #meshname = os.path.splitext(meshname)[0]
                #bpy.data.objects['node'].name = meshname
                #poses2[meshname] = poses[mesh]
            #else:
            bpy.ops.import_mesh.stl(files=[{"name": mesh + ".STL"}], directory=mesh_dir)

        #poses = poses2
        for obj in bpy.data.objects:
            name = obj.name.replace(" ","_")
            if name in poses:
                print(str(poses[name]))
                print("setting pose of " + name + " : " + str((poses[name][0], poses[name][1], poses[name][2])) + str(
                    (poses[name][3], poses[name][4], poses[name][5])))
                obj.location = (poses[name][0], poses[name][1], poses[name][2])
                obj.rotation_euler = (poses[name][3], poses[name][4], poses[name][5])
                i += 1
        return {'FINISHED'}


# ------------------------------------------------------------------------
#    register and unregister functions
# ------------------------------------------------------------------------
def register():
    bpy.utils.register_module(__name__)
    bpy.types.Scene.sdf_file = PointerProperty(type=FilePath)
    bpy.types.Scene.mesh_dir = PointerProperty(type=DirectoryPath)
    bpy.types.Scene.collisionPose = PointerProperty(type=CollisionPose)


def unregister():
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.sdf_file
    del bpy.types.Scene.mesh_dir


if __name__ == "__main__":
    register()
