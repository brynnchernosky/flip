import argparse
import numpy as np
import os
import mcubes

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_filepath", type=str, default="test_sdf", 
        help="Folder containing all point cloud files to convert to meshes")
    parser.add_argument("--output_filepath", type=str, default="test_meshes",
        help="Folder to write all converted triangle meshes to, names will be transfered over")
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

def main():
    # Open folder

    output_folder = args.output_filepath
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    directory = os.fsencode(args.input_filepath)
    for file in os.listdir(directory):
        filepath = os.fsdecode(file)
        fin = open(filepath, "r")
        vertices, triangles = generate_mesh(fin)
        filename = filepath.split('.')[0]
        output_filepath = os.path.join(output_folder, filename)
        mcubes.export_obj(vertices, triangles, output_filepath)

if __name__ == '__main__':
    args = parseArguments()
    main(args)