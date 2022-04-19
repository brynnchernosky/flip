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
    parser.add_argument("--sphere_points", type=int, default=50000)
    parser.add_argument("--sphere_radius", type=int, default=8)

    parser.add_argument("--cylinder", action="store_true")
    parser.add_argument("--cylinder_points", type=int, default=50000)
    parser.add_argument("--cylinder_radius", type=int, default=8)
    parser.add_argument("--cylinder_height", type=int, default=16)

    parser.add_argument("--cone", action="store_true")
    parser.add_argument("--cone_points", type=int, default=50000)
    parser.add_argument("--cone_radius", type=int, default=8)
    parser.add_argument("--cone_height", type=int, default=16)
    args = parser.parse_args()
    return args

def sphere(num_points, radius):
    current_points = 0
    points = np.zeros((1, 3))
    while current_points < num_points:
        sampled_point = np.random.uniform(-radius, radius, size=(1, 3))
        distance = np.linalg.norm(sampled_point)
        if distance < 8:
            points = np.append(points, sampled_point, axis=0)
            current_points += 1
    return points

def cylinder(num_points, radius, height):
    current_points = 0
    points = np.zeros((1, 3))
    while current_points < num_points:
        r = radius * np.sqrt(np.random.uniform(0, 1))
        theta = np.random.uniform(0, 1) * 2 * np.pi
        x = r * np.cos(theta)
        z = r * np.sin(theta)
        y = np.random.uniform(-height / 2, height / 2)
        sampled_point = np.array([[x, y, z]])
        points = np.append(points, sampled_point, axis=0)
        current_points += 1
    return points

def cone(num_points, radius, height):
    current_points = 0
    points = np.zeros((1, 3))
    ratio  = height / radius
    while current_points < num_points:
        sampled_radius = radius * np.sqrt(np.random.uniform(0, 1))
        sampled_height = ratio * sampled_radius
        r = sampled_radius * np.sqrt(np.random.uniform(0, 1))
        theta = np.random.uniform(0, 1) * 2 * np.pi
        x = r * np.cos(theta)
        z = r * np.sin(theta)
        y = (height / 2) - sampled_height
        sampled_point = np.array([[x, y, z]])
        points = np.append(points, sampled_point, axis=0)
        current_points += 1
    return points

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

def main(args):
    # Open folder and write each of the arguments to files in the 
    #Create directory if does not exist
    dataset_location = args.output_filepath
    if not os.path.exists(dataset_location):
        os.mkdir(dataset_location)
    
    if args.sphere:
        print("Generating sphere with", args.sphere_points, "points")
        filepath = os.path.join(dataset_location, "sphere.csv")
        fout = open(filepath, "w")
        points = sphere(args.sphere_points, args.sphere_radius)
        write_points(fout, points)
        if args.visualization:
            visualize_points(points)
        fout.close()
        print("Finished writing sphere.csv")
    
    if args.cylinder:
        print("Generating cylinder with", args.cylinder_points, "points")
        filepath = os.path.join(dataset_location, "cylinder.csv")
        fout = open(filepath, "w")
        points = cylinder(args.cylinder_points, args.cylinder_radius, args.cylinder_height)
        write_points(fout, points)
        if args.visualization:
            visualize_points(points)
        fout.close()
        print("Finished writing cylinder.csv")  

    if args.cone:
        print("Generating cone with", args.cone_points, "points")
        filepath = os.path.join(dataset_location, "cone.csv")
        fout = open(filepath, "w")
        points = cone(args.cone_points, args.cone_radius, args.cone_height)
        write_points(fout, points)
        if args.visualization:
            visualize_points(points)
        fout.close()
        print("Finished writing cone.csv")

if __name__ == '__main__':
    args = parseArguments()
    main(args)