import bpy
from bpy.props import (StringProperty,
                       PointerProperty,
                       )
from bpy.types import (Panel,
                       Operator,
                       AddonPreferences,
                       PropertyGroup,
                       )
import xml.etree.ElementTree

bl_info = {
    "name": "viaPoints for myoMuscles",
    "category": "User Interface",
}

# ------------------------------------------------------------------------
#    Dialog Operator
# ------------------------------------------------------------------------

current_viaPoint = 0
total_number_of_viaPoints = 0
viaPoints = list
myoMuscles = list
links = list
tree = xml.etree.ElementTree

class ViaPointInfoOperator(bpy.types.Operator):
    bl_idname = "wm.viapoint_info"
    bl_label = "show viaPoint info"

    x = bpy.props.FloatProperty()
    y = bpy.props.FloatProperty()
    z = bpy.props.FloatProperty()

    def execute(self, context):
        # this way the message appears in the header,
        global links
        global current_viaPoint
        global total_number_of_viaPoints
        global myoMuscles
        self.report({'INFO'}, "viaPoint (%d/%d) of link %s at %f %f %f (myoMuscle: %s " % ( current_viaPoint, total_number_of_viaPoints-1, links[current_viaPoint].attrib , self.x, self.y, self.z, myoMuscles[current_viaPoint].attrib ))
        return {'FINISHED'}

    def invoke(self, context, event):
        global current_viaPoint
        global viaPoints
        cursor = bpy.context.scene.cursor_location
        floats = [float(x) for x in viaPoints[current_viaPoint].text.split()]
        self.x = floats[0]
        self.y = floats[1]
        self.z = floats[2]
        return self.execute(context)


# ------------------------------------------------------------------------
#    ui
# ------------------------------------------------------------------------

sdf_file = "/home/roboy/PaBiRoboy/model.sdf"
stl_dir = "/home/roboy/PaBiRoboy/meshes"

class FilePath(PropertyGroup):
    sdf_file = StringProperty(
        name="",
        description="Path to sdf file",
        default="",
        maxlen=1024,
        subtype='FILE_PATH')

class DirectoryPath(PropertyGroup):
    stl_dir = StringProperty(
        name="",
        description="Path to stl directory",
        default="",
        maxlen=1024,
        subtype='DIR_PATH')

class OBJECT_PT_my_panel(Panel):
    bl_idname = "OBJECT_PT_my_panel"
    bl_label = "viaPoints for myoMuscles"
    bl_space_type = "VIEW_3D"
    bl_region_type = "TOOLS"
    bl_category = "Tools"
    bl_context = "objectmode"

    def draw(self, context):
        layout = self.layout
        scn = context.scene
        col = layout.column(align=True)
        col.prop(scn.sdf_file, "sdf_file", text="model.sdf")
        col.prop(scn.stl_dir, "stl_dir", text="stl directory")
        col.operator("my.load_meshes_button", text="load meshes")
        col.operator("my.acquire_viapoints", text="acquire viaPoints")
        col.operator("my.next_viapoint", text="next viaPoint")
        col.operator("my.prev_viapoint", text="previous viaPoint")
        col.operator("my.accept_viapoint", text="accept viaPoint")
        col.operator("wm.viapoint_info")

        global sdf_file
        global stl_dir
        sdf_file = scn.sdf_file.sdf_file
        stl_dir = scn.stl_dir.stl_dir

#   load meshes Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.load_meshes_button"
    bl_label = "Button"

    def execute(self, context):
        root = xml.etree.ElementTree.parse(sdf_file).getroot()
        meshes = set()
        for myoMuscle in root.iter('myoMuscle'):
            for l in myoMuscle.iter('link'):
                meshes.add(l.attrib['name'])

        for mesh in meshes:
            bpy.ops.import_mesh.stl(files=[{"name": mesh + ".STL"}], directory=stl_dir, global_scale=0.001)
        return {'FINISHED'}

#   acquire_viaPoints Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.acquire_viapoints"
    bl_label = "Button"

    def execute(self, context):
        global tree
        global links
        global current_viaPoint
        global total_number_of_viaPoints
        global myoMuscles
        global viaPoints
        current_viaPoint = 0
        total_number_of_viaPoints = 0
        tree = xml.etree.ElementTree.parse(sdf_file)
        root = tree.getroot()
        myoMuscles = list()
        links = list()
        viaPoints = list()
        for myoMuscle in root.iter('myoMuscle'):
            for l in myoMuscle.iter('link'):
                for viaPoint in l.iter('viaPoint'):
                    myoMuscles.append(myoMuscle)
                    links.append(l)
                    viaPoints.append(viaPoint)
                    total_number_of_viaPoints += 1
        try:
            bpy.ops.wm.viapoint_info.execute(context=context)
        except AttributeError:
            print("COOL ADDON NOT INSTALLED!")
        return {'FINISHED'}

#   next viaPoint Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.next_viapoint"
    bl_label = "Button"
    def execute(self, context):
        global current_viaPoint
        global total_number_of_viaPoints
        if(current_viaPoint < total_number_of_viaPoints - 1):
            current_viaPoint += 1
        return {'FINISHED'}

#   previous viaPoint Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.prev_viapoint"
    bl_label = "Button"

    def execute(self, context):
        global current_viaPoint
        if (current_viaPoint > 0):
            current_viaPoint -= 1
        return {'FINISHED'}

#   accept viaPoint Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.accept_viapoint"
    bl_label = "Button"

    def execute(self, context):
        cursor = bpy.context.scene.cursor_location
        global tree
        global viaPoints
        global current_viaPoint
        viaPoints[current_viaPoint].text = "%f %f %f" % (cursor.x, cursor.y, cursor.z)
        tree.write(sdf_file)
        return {'FINISHED'}
# ------------------------------------------------------------------------
#    register and unregister functions
# ------------------------------------------------------------------------
def register():
    bpy.utils.register_module(__name__)
    bpy.types.Scene.sdf_file = PointerProperty(type=FilePath)
    bpy.types.Scene.stl_dir = PointerProperty(type=DirectoryPath)

def unregister():
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.sdf_file
    del bpy.types.Scene.stl_dir

if __name__ == "__main__":
    register()
