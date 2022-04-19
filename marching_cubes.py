import argparse
import numpy as np
import os
import mcubes
import open3d as o3d

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_filepath", type=str, default="test_sdf", 
        help="Folder containing all point cloud files to convert to meshes")
    parser.add_argument("--output_filepath", type=str, default="test_meshes",
        help="Folder to write all converted triangle meshes to, names will be transfered over")
    parser.add_argument("--visualization", action="store_true", 
        help="If enabled creates intermediate visualization for each object")
    args = parser.parse_args()
    return args

def generate_mesh(fin):
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
    
    vertices, triangles = mcubes.marching_cubes(u, 0)
    center = [x_dim/2, y_dim/2, z_dim/2]
    vertices = vertices - center

    return vertices, triangles

def main(args):
    # Open folder

    output_folder = args.output_filepath
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    directory = os.fsencode(args.input_filepath)
    for file in os.listdir(directory):
        filename = os.fsdecode(file)
        if filename.endswith(".csv"):
            input_sdf = os.path.join(args.input_filepath, filename)
            print("Reading", input_sdf)
            fin = open(input_sdf, "r")
            vertices, triangles = generate_mesh(fin)

            name = filename.split('.')[0]
            output_filepath = os.path.join(output_folder, name) + ".obj"

            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(np.asarray(vertices))
            mesh.triangles = o3d.utility.Vector3iVector(np.asarray(triangles))
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(output_filepath, mesh)
            o3d.visualization.draw_geometries([mesh])

if __name__ == '__main__':
    args = parseArguments()
    main(args)