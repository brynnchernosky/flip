import numpy as np
import open3d as o3d
import argparse

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", type=str, default="test.csv", 
        help="filepath to csv")
    args = parser.parse_args()
    return args

def main(args):
    with open(args.filepath) as file:
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
    print(pcd.get_center())
    print(pcd.get_max_bound())
    print(pcd.get_min_bound())
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    args = parseArguments()
    main(args)