import bpy

class testPanel(pby.types.Panel):
    bl_label = "test panel"
    bl_idname = "PT_TestPanel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "NewTap"
    
    def draw(self, context):
        layout = self.layout
        
        row = layout.row()
        row.label(text = "sample text", icon = "cube")
        
        
        
        
def register():
    bpy.utils.register_class(testPanel)
    
def unregister():
    bpy.utils.unregister_class(testPanel)
    
    
if __name__ == "__main__":
    register()