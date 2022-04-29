import numpy as np
import open3d as o3d
import argparse
import os 

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", type=str, default="", 
        help="filepath to csv")
    parser.add_argument("--folder", type=str, default="", 
        help="folder to csvs to iterate through")
    args = parser.parse_args()
    return args

def vis_file(filepath):
    with open(filepath) as file:
        lines = file.readlines()
    
    points = []
    for line in lines:
        pos = line.split(',')
        points.append(float(pos[0]))
        points.append(float(pos[1]))
        points.append(float(pos[2]))
    
    points = np.reshape(np.array(points), (-1, 3))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0, 0, 1])
    print(points.shape[0])
    o3d.visualization.draw_geometries([pcd])

def main(args):
    if args.filepath:
        vis_file(args.filepath)
    
    if args.folder:
        directory = os.fsencode(args.folder)
        for file in os.listdir(directory):
            filename = os.fsdecode(file)
            if filename.endswith(".csv"):
                filepath = os.path.join(args.folder, filename)
                vis_file(filepath)

if __name__ == '__main__':
    args = parseArguments()
    main(args)