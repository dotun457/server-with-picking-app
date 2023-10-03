
from collections import namedtuple
from pylibpcap.pcap import rpcap
from velodyne_decoder_pylib import *

import argparse
import asyncio
import dpkt
import json
import numpy as np
import open3d as o3d
import pynmea2
import time
import socket
import velodyne_decoder as vd  # Requires version 2.3.0
import websockets
import os
import warnings


# Directory Management
settings = {
    '__header__': b'MATLAB 5.0 MAT-file, Platform: PCWIN64, Created on: Sun Aug  6 14:57:54 2023',
    '__version__': '1.0',
    '__globals__': [],
    'R': np.array([[0]], dtype=np.uint8),
    'alpha_1': np.array([[0]], dtype=np.uint8),
    'alpha_2': np.array([[0]], dtype=np.uint8),
    'angle': np.array([[360]], dtype=np.uint16),
    'first': np.array([[10]], dtype=np.uint8),
    'gridStep': np.array([[0.005]]),
    'input_file_name': np.array(['./pcap_files/5rpm 1min.pcap'], dtype='<U30'),
    'pos': np.array([[2]], dtype=np.uint8),
    'pos2': np.array([[0]], dtype=np.uint8),
    'puck': np.array([[2]], dtype=np.uint8),
    'times': np.array([[3600]], dtype=np.uint16),
    'totalScanFrames': np.array([[3652]], dtype=np.uint16),
    'usableFrames': np.array([[3642]], dtype=np.uint16)
}

input_file_name = settings['input_file_name'][0]
puck = settings['puck'][0][0]
config = vd.Config(model="VLP-32C", rpm=243, gps_time = True)  # LiDAR settings 

#create point cloud 
def create_point_cloud(input_file_name, config):
    frame = 0
    pcd = []   
    for points in vd.read_pcap(input_file_name, config):
        # print("Frame " + str(frame) + " with " + str(len(points.points)) + " points received at timestamp: " + str(points.stamp))
        data = points.points 
        data = [np.array([x[0], x[1], x[2], x[3]]) for x in data] # array of xyz points => cloud
        pcd.append(np.array(data)) # append cloud to list
        frame = frame + 1
    # Convert the NumPy array to an Open3D PointCloud
    return pcd

if puck == 1:
    pcd_array = create_point_cloud(input_file_name, config)
else:
    # Assuming the 'VLP32C' format corresponds to another pcd format. Adjust as needed.
    pcd_array = create_point_cloud(input_file_name, config)

# Define transformation matrices
def generate_transform_matrix(angle, alpha_1, alpha_2, R, theta3):
    VM = np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

    A1 = np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.radians(alpha_1)), np.sin(np.radians(alpha_1)), 0],
        [0, -np.sin(np.radians(alpha_1)), np.cos(np.radians(alpha_1)), 0],
        [0, 0, 0, 1]
    ])

    A2 = np.array([
        [np.cos(np.radians(alpha_2)), np.sin(np.radians(alpha_2)), 0, 0],
        [-np.sin(np.radians(alpha_2)), np.cos(np.radians(alpha_2)), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, R],
        [0, 0, 0, 1]
    ])

    T4 = np.array([
        [np.cos(np.radians(theta3)), np.sin(np.radians(theta3)), 0, 0],
        [-np.sin(np.radians(theta3)), np.cos(np.radians(theta3)), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    return VM, A1, A2, T, T4

def filter_points_by_pos(ptCloudIn, pos=2): #filters the point cloud data to only include points that are located in the negative X half-space (X < 0)
    if pos == 1:
        return ptCloudIn[ptCloudIn[:, 0] > 0]
    elif pos == 2:
        # ptCloudIn3 = ptCloudIn[:, 0].copy()
        # # Replace NaN values with zeros
        # ptCloudIn3[np.isnan(ptCloudIn3)] = 0

        # # Replace positive values with 0, and negative values with 1
        # ptCloudIn3[ptCloudIn3 > 0] = 0
        # ptCloudIn3[ptCloudIn3 < 0] = 1

        # # Process intensities
        # ptCloudIntensity = ptCloudIn[:, 3].copy()  # Extracting the fourth column for intensity
        # ptCloudIntensity[np.isnan(ptCloudIntensity)] = 0

        # # Multiply intensity with the processed X values
        # ptCloudIntensity2 = ptCloudIntensity * ptCloudIn3

        # # Update point cloud locations and intensities
        # ptCloudIn[:, :3] = ptCloudIn[:, :3] * ptCloudIn3[:, np.newaxis]  # Broadcasting to multiply with XYZ coordinates
        # ptCloudIn[:, 3] = ptCloudIntensity2  # Update the intensity values
        # return ptCloudIn[ptCloudIn[:, 0] > 0]

        indices = ptCloudIn[:, 0] <= 0
    
        # Use the indices to filter the array
        filtered_points = ptCloudIn[indices]
        return filtered_points
    else:
        return ptCloudIn

def filter_points_by_band(Cloud, times, pos2=0, puck=2):
    bandNumber = 16 if (pos2 == 1 and puck == 1) else (32 if (pos2 == 1 and puck == 2) else (2 if pos2 == 0 else 1))


    point_clouds = []

    for bande_sep in range(bandNumber):

        if pos2 == 1:
            iiii = bande_sep
        elif pos2 == 0:
            iiii = [1, 16] if puck == 1 else [1, 32][bande_sep]
        else:
            iiii = 8 if puck == 1 else 16

        # Variable initialization
        x, y, z, inten = [], [], [], []
        X_ref, Y_ref, Z_ref, int_ref = [], [], [], []
        X_ref_final, Y_ref_final, Z_ref_final, int_ref_final = [], [], [], []

        for iii in range(10):
            for ii in range(round((times / 10 * iii) - ((times / 10) - 1)), round(iii * times / 10)):
                # Extract data from Cloud (assuming Cloud is a list of numpy arrays)
                x1 = Cloud[ii][:, 0]
                y1 = Cloud[ii][:, 1]
                z1 = Cloud[ii][:, 2]
                int1 = Cloud[ii][:, 3]

                x.append(x1)
                y.append(y1)
                z.append(z1)
                inten.append(int1)

            X_ref.extend(x)
            Y_ref.extend(y)
            Z_ref.extend(z)
            int_ref.extend(inten)

            x, y, z, inten = [], [], [], []

        X_ref_final.extend(X_ref)
        Y_ref_final.extend(Y_ref)
        Z_ref_final.extend(Z_ref)
        int_ref_final.extend(int_ref)

        # Assuming the reconstruction is directly from the final X, Y, Z, and intensity values
        point_cloud = np.column_stack((X_ref_final, Y_ref_final, Z_ref_final, int_ref_final))
        point_clouds.append(point_cloud)
        return point_cloud

async def pre_processing(points):
    #Loop over each angle and apply transformations
    times = settings['times'][0][0]
    angle = settings['angle'][0][0]
    if angle < 45 or angle > 360:
        print("Input for angle is invalid or outside the valid range (45-360). Setting angle to 360.")
        angle = 360

    first = settings['first'][0][0]
    puck = settings['puck'][0][0]
    filesToMerge = []

    # Export parameters
    pos = settings['pos'][0][0]
    pos2 = settings['pos2'][0][0]
    gridStep = settings['gridStep'][0][0]

    # Calibration parameters
    alpha_1 = settings['alpha_1'][0][0]
    alpha_2 = settings['alpha_2'][0][0]
    R = settings['R'][0][0]
    theta3 = 0

    # totalScanFrames = len(pcd_array) #hard coded because our library can't read frames
    # usableFrames = totalScanFrames - first
    # if times > usableFrames:
    #     warnings.warn(f'Entered time value is too large. It has been replaced with the maximum allowed Time of {usableFrames}.')
    #     times = usableFrames

    # last = first + times
    # angle_deg = np.linspace(0, angle, times+1)
    # angles = np.radians(angle_deg)
    # s = 0
    
    transformed_points = filter_points_by_pos(points, pos)
    VM, A1, A2, T, T4 = generate_transform_matrix(angle, alpha_1, alpha_2, R, theta3)
    for tform in [T, A1, A2, T4, VM]:
        transformed_points = points @ tform.T
        #filter by band
    #make everything a list 
    new_points = [list(point) for point in transformed_points]
    return new_points
print("number of points ", len(pcd_array[0]))
# updated_pcd_array = []
# for points in pcd_array:
#     updated_points = pre_processing(points)
#     updated_pcd_array.append(updated_points)

# test = filter_points_by_pos(pcd_array[0], 2)
# print(len(test))