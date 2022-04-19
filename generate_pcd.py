import argparse 
import numpy as np
import os
import open3d as o3d

def parseArguments():
    parser = argparse.ArgumentParser()
    # General arguments 
    parser.add_argument("--output_filepath", type=str, default="test_points",
        help="Folder to write all point clouds to")
    parser.add_argument("--visualization", action="store_true", 
        help="If enabled creates intermediate visualization for each object")
    # Arguments per each object type 
    parser.add_argument("--sphere", action="store_true")
    parser.add_argument("--sphere_points", type=int, default=5000)
    parser.add_argument("--sphere_radius", type=int, default=8)

    args = parser.parse_args()
    return args

def sphere(num_points, radius):
    current_points = 0
    accum_points = np.zeros((1, 3))
    while current_points < num_points:
        sampled_point = np.random.uniform(-radius, radius, size=(1, 3))
        distance = np.linalg.norm(sampled_point)
        if distance < 8:
            accum_points = np.append(accum_points, sampled_point, axis=0)
            current_points += 1
    return current_points

def write_points(fout, points):
    for i in range(points.shape[0]):
        particle = points[i]
        toWrite = str(particle[0]) + ", " + str(particle[1]) + ", " + str(particle[2]) + "\n"
        fout.write(toWrite)

def visualize_points(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0,0,1])
    o3d.visualization.draw_geometries([pcd])

def main():
    # Open folder and write each of the arguments to files in the 
    #Create directory if does not exist
    dataset_location = args.output_filepath
    if not os.path.exists(dataset_location):
        os.mkdir(dataset_location)
    
    if args.sphere:
        print("Generating sphere with ", args.sphere_points, " points")
        filepath = os.path.join(dataset_location, "sphere.csv")
        fout = open(filepath, "w")
        points = sphere(args.sphere_points, args.sphere_radius)
        write_points(fout, points)
        if args.visualization:
            visualize_points(points)
        fout.close()
        print("Finished writing sphere file")

if __name__ == '__main__':
    args = parseArguments()
    main(args)