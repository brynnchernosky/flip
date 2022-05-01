import numpy as np
import open3d as o3d
import argparse
import configparser
import os 
import time 

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=str, default="", 
        help="folder to csvs to iterate through")
    args = parser.parse_args()
    return args

def get_points_from_file(filepath):
    with open(filepath) as file:
        lines = file.readlines()

    points = []
    for line in lines:
        pos = line.split(',')
        points.append(float(pos[0]))
        points.append(float(pos[1]))
        points.append(float(pos[2]))
    
    points = np.reshape(np.array(points), (-1, 3))

    return points

def get_bounding_box(config_filepath):
    config = configparser.ConfigParser()
    config.read(config_filepath)

    cellWidth = config.getfloat('Simulation', 'cellWidth')

    cellCountX = config.getint('Simulation', 'cellCountX')
    cellCountY = config.getint('Simulation', 'cellCountY')
    cellCountZ = config.getint('Simulation', 'cellCountZ')

    cornerPositionX = config.getfloat('Simulation', 'cornerPositionX')
    cornerPositionY = config.getfloat('Simulation', 'cornerPositionY')
    cornerPositionZ = config.getfloat('Simulation', 'cornerPositionZ')

    minBound = np.array([cornerPositionX, cornerPositionY, cornerPositionZ])
    maxBound = minBound + (np.array([cellCountX, cellCountY, cellCountZ]) * cellWidth)
    center = (minBound + maxBound) / 2
    extent = maxBound - minBound

    obb = o3d.geometry.OrientedBoundingBox(center, np.eye(3), extent)
    
    return obb

def vis_folder(pcd_list, obb):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(obb)
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    ctr = vis.get_view_control()
    ctr.set_front([0, 1, 0])
    ctr.set_lookat([-1, 0, 0])
    ctr.set_up([0, 0, 1])
    ctr.set_zoom(1.3)

    current_frame = 0
    num_frames = len(pcd_list)

    prev = time.time()
    frame_time = 0.5
    
    # pcd.points = o3d.utility.Vector3dVector(pcd_list[current_frame % num_frames])
    # pcd.paint_uniform_color([0, 0, 1])
    # vis.update_geometry(pcd)
    # vis.run()
    
    while True:
        current = time.time()
        if current - prev > frame_time: 
            pcd.points = o3d.utility.Vector3dVector(pcd_list[current_frame % num_frames])
            pcd.paint_uniform_color([0, 0, 1])
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            prev = current
            current_frame += 1
        
        if current_frame == 50:
            vis.destroy_window()

def main(args):    
    # Accumulate file names in list 
    filenames = []
    if args.folder:
        particle_folder = os.path.join(args.folder, "particles")
        directory = os.fsencode(particle_folder)
        for file in os.listdir(directory):
            filename = os.fsdecode(file)
            if filename.endswith(".csv"):
                filenames.append(filename)
    else:
        exit()
    
    obb = get_bounding_box(os.path.join(args.folder, "config.ini")) 

    def convert_filename(filename):
        return float(filename.split('.')[0])
    filenames = sorted(filenames, key=lambda x: convert_filename(x))

    pcd_list = []
    for filename in filenames:
        filepath = os.path.join(particle_folder, filename)
        pcd_list.append(get_points_from_file(filepath))
    
    vis_folder(pcd_list, obb)

if __name__ == '__main__':
    args = parseArguments()
    main(args)