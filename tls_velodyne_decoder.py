
from collections import namedtuple
from pylibpcap.pcap import rpcap
from velodyne_decoder_pylib import *
# from scipy.ndimage import affine_transform
import concurrent.futures
from copy import deepcopy
import numpy as np

# Define transformation matrices
def generate_transform_matrix(alpha_1, alpha_2, R, theta3):

    A1 = np.array([ #degrees in matlab
        [1, 0, 0, 0],
        [0, np.cos(np.radians(alpha_1)), np.sin(np.radians(alpha_1)), 0],
        [0, -np.sin(np.radians(alpha_1)), np.cos(np.radians(alpha_1)), 0],
        [0, 0, 0, 1]
    ])

    A2 = np.array([ #degrees in matlab
        [np.cos(np.radians(alpha_2)), np.sin(np.radians(alpha_2)), 0, 0],
        [-np.sin(np.radians(alpha_2)), np.cos(np.radians(alpha_2)), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T = np.array([ # R = 0
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, R],
        [0, 0, 0, 1]
    ])

    T4 = np.array([ # degrees in matlab 
        [np.cos(theta3), np.sin(theta3), 0, 0],
        [-np.sin(theta3), np.cos(theta3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    #have to return the transpose
    return A1, A2, T, T4


def filter_points_by_pos(ptCloudIn, pos=2): #filters the point cloud data to only include points that are located in the negative X half-space (X < 0)
    if pos == 1:
        return ptCloudIn[ptCloudIn[:, 0] > 0]
    elif pos == 2:
        x = ptCloudIn[:, 0]
        x_without_nan = np.nan_to_num(x, nan=0)
        indices = x_without_nan <= 0
        filtered_points = ptCloudIn[indices]
        return filtered_points
    else:
        return ptCloudIn

def apply_affine_transformation(points, transform):
    transformed_points = points @ transform
    return transformed_points

def matlab_point_clouds_dome(times, pointCloud, band_number: int=32): #first version of the matlab merging I converted 
    rotated_point_cloud = []
    for bande_sep in range(band_number):#32
        X_ref_final, Y_ref_final, Z_ref_final , int_ref_final = []
        for iii in range(10):
            lower_range = round( times/10) * iii  
            upper_range = (iii + 1) * round(times/10) - 1

            X_ref, Y_ref, Z_ref, int_ref = []
            for ii in range(lower_range, upper_range): 
            # this loop runs num_frames / 10 i.e if we have a pcap of 600 frames it will run 60 times
            # on each iteration we take a frame fromn the point cloud i.e frame = pointCloud[ii]
            # each frame has 32 bands (lidar has 32 lasers) and we take the points for a band
            # if a frame has 640 points that means each band has 20 points 
            # band_sep let's us know what current band we want the points to be from
                frame = pointCloud[ii]
                num_points = len(frame)
                lower_bound = bande_sep * round(num_points / 32)
                upper_bound = ( bande_sep + 1) * round( num_points / 32 ) - 1

                X_ref.append(frame[:, 0][lower_bound:upper_bound]) #take x points of a band range
                Y_ref.append(frame[:, 1][lower_bound:upper_bound])
                Z_ref.append(frame[:, 2][lower_bound:upper_bound])
                int_ref.append(frame[:,3][lower_bound:upper_bound])
        
            X_ref_final.append(X_ref)
            Y_ref_final.append(Y_ref)
            Z_ref_final.append(Z_ref)
            int_ref_final.append(int_ref)

        PC_corr1 = np.column_stack((X_ref_final, Y_ref_final, Z_ref_final))
        rot = np.array([
            [1, 0, 0, 0],
            [0, np.cosd(np.radians(-90)), -np.sin(np.radians(-90)), 0],
            [0, np.sind(np.radians(-90)), np.cosd(np.radians(-90)), 0],
            [0, 0, 0, 1]
        ])
        PC_Final1 = apply_affine_transformation(PC_corr1, rot)
        rotated_point_cloud.append(PC_Final1)
    return rot


def point_cloud_dome(point_cloud, band_number: int=32): #final version, current function being used 
    combined_pcd = None
    def map_fun_rotate(frame, bande_sep):#return a rotated point cloud frame 
        cur_frame = deepcopy(frame)
        num_points = len(frame)
        lower_bound = bande_sep * round(num_points / band_number)
        upper_bound = ( bande_sep + 1) * round( num_points / band_number ) - 1

        mask = np.zeros(num_points, dtype=bool) #creates a frame numpy array where all points not in the current band range will be False / 0
        mask[lower_bound:upper_bound] = True

        # Set rows not in the second chunk to zero
        cur_frame[~mask] = 0
        rot = np.array([
                [1, 0, 0, 0],
                [0, np.cos(-90), -np.sin(-90), 0],
                [0, np.sin(-90), np.cos(-90), 0],
                [0, 0, 0, 1]
            ])
        rotated_frame = apply_affine_transformation(cur_frame, rot)
        return rotated_frame
    
    for bande_sep in range(band_number):
        band_point_cloud =  list(map( lambda frame: map_fun_rotate(frame, bande_sep), point_cloud)) #switch to process pool later
        if bande_sep == 0: 
            combined_pcd = band_point_cloud
        else:
            combined_pcd = [final_arr + new_arr for final_arr, new_arr in zip(combined_pcd, band_point_cloud)] #MIGHT BE WRONG 
    return combined_pcd
