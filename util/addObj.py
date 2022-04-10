import bpy
import os.path
import math
import sys

C = bpy.context
D = bpy.data
scene = D.scenes['Scene']

class testPanel(bpy.types.Panel):
    bl_label = "test panel"
    bl_idname = "PT_TestPanel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "NewTap"
    
    def draw(self, context):
        layout = self.layout
        
        row = layout.row()
        row.label(text = "sample text",icon = "CUBE")
        row = layout.row()
        row.operator("mesh.primitive_cube_add")
        row.operator("import_mesh.stl(filepath='C:\\Program Files\\BlenderPhong\\Avionfixed'")
        
    
def init_camera():
    cam = D.objects['Camera']
    # select the camera object
    bpy.context.view_layer.objects.active = cam
    # scene.objects.active = cam
    # cam.select = True
    bpy.context.active_object.select_set(state=True)

    # set the rendering mode to orthogonal and scale
    C.object.data.type = 'ORTHO'
    C.object.data.ortho_scale = 10
    C.object.location = (6,-6,5.6)
    bpy.context.active_object.select_set(state=False)
            
        
def register():
    bpy.utils.register_class(testPanel)
    
def unregister():
    bpy.utils.unregister_class(testPanel)
    
    
if __name__ == "__main__":
    # bpy.ops.object.camera_add(enter_editmode=False, align='VIEW', location=(-13.8654, 0.00558472, 20.2967), rotation=(1.10932, -4.6508e-08, 0.814928), scale=(1, 1, 1))
    # bpy.ops.object.light_add(type='SUN', align='WORLD', location=(-13.8654, 0.00558472, 20.2967), scale=(1, 1, 1))

    # bpy.ops.import_mesh.stl(filepath='C:\ProgramFiles\BlenderPhong\Avionfixed.stl')
    path = "./Torus.stl"
    
    d = os.path.dirname(path)
    ext = path.split('.')[-1]

    name = os.path.basename(path).split('.')[0]
    # handle weird object naming by Blender for stl files
    if ext == 'stl':
        name = name.title().replace('_', ' ')
    if name not in D.objects:
        print('loading :' + name)
        if ext == 'stl':
            bpy.ops.import_mesh.stl(filepath=path, directory=d,
                                    filter_glob='*.stl')
        elif ext == 'off':
            bpy.ops.import_mesh.off(filepath=path, filter_glob='*.off')
        elif ext == 'obj':
            bpy.ops.import_scene.obj(filepath=path, filter_glob='*.obj')
        else:
            print('Currently .{} file type is not supported.'.format(ext))
            exit(-1)
            
    init_camera()
    target = D.objects['Sun']
    bpy.context.view_layer.objects.active = target
    bpy.context.active_object.select_set(state=True)
    target.location = (0,4,6)
    bpy.context.active_object.select_set(state=False)
    
    target = D.objects[name]
    bpy.context.view_layer.objects.active = target
    bpy.context.active_object.select_set(state=True)
    
    bpy.ops.object.material_slot_add()
    # Get material
    mat = bpy.data.materials.get("F water")
    if mat is None:
    # create material
        mat = bpy.data.materials.new(name="F water")
        
    if target.data.materials:
    # assign to 1st material slot
        target.data.materials[0] = mat
    else:
    # no slots
        target.data.materials.append(mat)
            
    bpy.ops.render.render()
    
    path = os.path.join("result/", name + '.png')
    D.images['Render Result'].save_render(filepath=path)
    print('save to ' + path)
    
    target = D.objects[2]
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.delete()

    register()
