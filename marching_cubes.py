import argparse
import configparser
import numpy as np
import os
from pathlib import Path
import mcubes
import open3d as o3d

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=str, default="test", 
        help="Parent folder to look in")
    parser.add_argument("--visualization", action="store_true", 
        help="If enabled creates intermediate visualization for each object")
    args = parser.parse_args()
    return args

def generate_mesh(input_sdf):
    fin = open(input_sdf, "r")
    # First line is grid dimensions, rest of lines [grid_pos as x, y, z] [signed_distance]
    dims = fin.readline()
    dims = dims.split(',')

    x_dim = int(dims[0])
    y_dim = int(dims[1])
    z_dim = int(dims[2])

    u = np.zeros([x_dim, y_dim, z_dim])
    grid_size = x_dim * y_dim * z_dim
    
    for _ in range(grid_size):
        line = fin.readline()
        line = line.split(",")
        index = [int(line[0]), int(line[1]), int(line[2])]
        sd = float(line[3])
        u[index[0], index[1], index[2]] = sd
    
    fin.close()

    vertices, triangles = mcubes.marching_cubes(u, 0)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(vertices))
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(triangles))
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()

    return mesh

def clear_directory(folder):
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        os.unlink(file_path)

def vis_folder(mesh_list, obb):
    for mesh in mesh_list:
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(obb)
        vis.add_geometry(mesh)

        ctr = vis.get_view_control()
        ctr.set_front([0, -1, 0])
        ctr.set_lookat([-1, 0, 0])
        ctr.set_up([0, 0, 1])
        ctr.set_zoom(0.3)

        vis.run()
        vis.destroy_window()

def main(args):
    output_folder = os.path.join(args.folder, "meshes")
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)
    else:
        # Clear directory 
        clear_directory(output_folder)

    filenames = []
    input_folder = os.path.join(args.folder, "sdfs")
    directory = os.fsencode(input_folder)
    for file in os.listdir(directory):
        filename = os.fsdecode(file)
        if filename.endswith(".csv"):
            input_sdf = os.path.join(input_folder, filename)
            filenames.append(input_sdf)
    
    filenames = sorted(filenames, key=lambda x: float(Path(x).stem))
    meshes = []
    print("Reading SDFs and writing Meshes")
    for filename in filenames:
        mesh = generate_mesh(filename)
        meshes.append(mesh)
        name = Path(filename).stem
        output_filepath = os.path.join(output_folder, name) + ".obj"
        o3d.io.write_triangle_mesh(output_filepath, mesh)

    print("Finished Conversion")

if __name__ == '__main__':
    args = parseArguments()
    main(args)