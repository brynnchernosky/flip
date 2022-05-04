from genericpath import exists
import bpy 
import os
from pathlib import Path
import configparser
import numpy as np

parent_folder = "/Users/adrianchang/desktop/CS/CS2240/flip/results/workingFolder"

C = bpy.context
D = bpy.data
scene = D.scenes['Scene']

def get_enclosure_bounds(config_filepath):
    config = configparser.ConfigParser()
    config.read(config_filepath)

    gridHeight = config.getint('Conversion', 'gridHeight') #x
    gridWidth = config.getint('Conversion', 'gridWidth') #y
    gridLength = config.getint('Conversion', 'gridLength') #z

def render_obj(input_filepath, output_filepath):
    bpy.ops.import_scene.obj(filepath=input_filepath, axis_forward='Y', axis_up='Z')
    obj_object = bpy.context.selected_objects[0] ####<--Fix
    name = obj_object.name
    
    target = D.objects[name]
    bpy.context.view_layer.objects.active = target
    bpy.context.active_object.select_set(state=True)

    bpy.ops.object.material_slot_add()
    # Get material
    mat = bpy.data.materials.get("F My_Water")
    target.data.materials[0] = mat
    
    bpy.ops.render.render()

    D.images['Render Result'].save_render(filepath=output_filepath)
    print('save to ' + output_filepath)

    target = D.objects[2]
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.delete()

def clear_directory(folder):
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        os.unlink(file_path)

def main():
    parent_folder = "/Users/adrianchang/desktop/CS/CS2240/flip/results/workingFolder"
    mesh_folder = os.path.join(parent_folder, "meshes")
    render_folder = os.path.join(parent_folder, "renders")
    if not os.path.exists(render_folder):
        os.mkdir(render_folder)
    else:
        print("Clearing", render_folder)
        clear_directory(render_folder)

    print("Collecting filenames")
    filenames = []
    for file in os.listdir(mesh_folder):
        filename = os.fsdecode(file)
        if filename.endswith(".obj"):
            filenames.append(filename)

    filenames = sorted(filenames, key=lambda x: float(Path(x).stem))

    print("Iterating through filenames")
    for filename in filenames:
        #Import relevant objs
        input_filepath = os.path.join(mesh_folder, filename)
        output_filepath = os.path.join(render_folder, Path(filename).stem + ".png")
        print(input_filepath)
        print(output_filepath)
        render_obj(input_filepath, output_filepath)

if __name__ == "__main__":
    main()

