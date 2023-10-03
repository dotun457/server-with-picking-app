import numpy as np
import open3d as o3d
import os
import scipy.io
import warnings

# Directory Management
settings = scipy.io.loadmat('C:/TLS_Velodyne/application/settings.mat')
input_file_name = settings['input_file_name'][0]
filepath = os.path.dirname(input_file_name)
resultsDir = os.path.join(filepath, 'results')

try:
    os.makedirs(resultsDir, exist_ok=True)
except Exception as e:
    print(f"Error creating directory: {e}")

output = resultsDir
output_file_name, ext = os.path.splitext(os.path.basename(input_file_name))

# Initialization of parameters
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

# ... Rest of the code ...

# Note: The Python version only covers the provided MATLAB code up to this point. The complete transformation of the MATLAB code, including point cloud operations, would require a much lengthier conversion. The key functions and libraries needed for such a conversion would be Open3D for point cloud operations and numpy for mathematical operations.

# Continue with point cloud processing...

# Read point cloud file
if puck == 1:
    pcd = o3d.io.read_point_cloud(input_file_name, format='pcd')
else:
    # Assuming the 'VLP32C' format corresponds to another pcd format. Adjust as needed.
    pcd = o3d.io.read_point_cloud(input_file_name, format='pcd')

totalScanFrames = len(pcd.points)
usableFrames = totalScanFrames - first
if times > usableFrames:
    warnings.warn(f'Entered time value is too large. It has been replaced with the maximum allowed Time of {usableFrames}.')
    times = usableFrames

last = first + times
angle_deg = np.linspace(0, angle, times+1)
angles = np.radians(angle_deg)
s = 0

# Save settings (using a simple dictionary saved as a npz file)
settings_dict = {'times': times, 'angle': angle, 'first': first, 'gridStep': gridStep, 'alpha_1': alpha_1, 
                 'alpha_2': alpha_2, 'R': R, 'pos': pos, 'pos2': pos2, 'input_file_name': input_file_name, 
                 'puck': puck, 'usableFrames': usableFrames, 'totalScanFrames': totalScanFrames}
np.savez('C:/TLS_Velodyne/application/settings.npz', **settings_dict)

# ... Rest of the code ...

# You'd then continue with the logic that processes each frame of the point cloud data.
# Using Open3D, you can extract frames, apply transformations, and export data in various formats.

# Given the complexity and the specific details that would need to be known for a precise conversion 
# (e.g., the structure and format of your point cloud data, other specifics about your use case), 
# this provides a general outline to get you started.

# If you have specific functionality or operations from the MATLAB code that you'd like translated into Python,
# breaking down the code into smaller, more specific chunks would allow for a more accurate conversion.

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

def transform_point_cloud(pcd, tform):
    # Convert open3d point cloud to numpy array
    points = np.asarray(pcd.points)
    # Homogeneous coordinates
    points_h = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_points_h = points_h @ tform.T
    transformed_points = transformed_points_h[:, :3]
    # Update point cloud
    pcd.points = o3d.utility.Vector3dVector(transformed_points)
    return pcd

# Create a list to store the transformed point clouds
clouds = []

# Loop over each angle and apply transformations
for angle in angles:
    VM, A1, A2, T, T4 = generate_transform_matrix(angle, alpha_1, alpha_2, R, theta3)
    transformed_pcd = pcd
    for tform in [T, A1, A2, T4, VM]:
        transformed_pcd = transform_point_cloud(transformed_pcd, tform)
    clouds.append(transformed_pcd)

# Note: This code covers transformations based on the angles.
# The operations like frame-based processing, file merging, and point cloud operations 
# like downsampling or filtering based on position are still to be translated.

# More functionality would be added by diving deep into the specific operations 
# required for your use case and leveraging Open3D's rich API.

# Define the transformation matrices as before:

# ... [Previously defined functions] ...

# Next, we handle the part of the code responsible for selecting specific segments of the point cloud:

# If we're only considering X (as in your MATLAB code for Velodyne):
def filter_points_by_pos(points, pos):
    if pos == 1:
        return points[points[:, 0] > 0]
    elif pos == 2:
        return points[points[:, 0] < 0]
    else:
        return points

points = filter_points_by_pos(points, pos)

# The below logic simulates the band extraction from your MATLAB code. 
# Given we are not aware of the exact structure of your point cloud, 
# we'll assume that bands are a consistent split of your point cloud in the Z-direction:

def filter_points_by_band(points, pos2, puck):
    z_values = points[:, 2]
    min_z, max_z = np.min(z_values), np.max(z_values)
    total_bands = 16 if puck == 1 else 32
    band_height = (max_z - min_z) / total_bands

    if pos2 == 1:
        return points
    elif pos2 == 0:
        # Taking first and last bands
        first_band = points[(z_values >= min_z) & (z_values < min_z + band_height)]
        last_band = points[(z_values <= max_z) & (z_values > max_z - band_height)]
        return np.vstack((first_band, last_band))
    elif pos2 == 2:
        # Taking middle band
        mid_z = (min_z + max_z) / 2
        return points[(z_values >= mid_z - band_height/2) & (z_values < mid_z + band_height/2)]
    else:
        return points

points = filter_points_by_band(points, pos2, puck)

# Downsampling can be achieved using a simple grid-based approach. For now, we'll skip that, 
# but libraries like `scipy.spatial` can be used to facilitate the downsampling process.

# Transformation of each band and Cloud reconstruction:

def transform_band(points, angle, alpha_1, alpha_2, R, theta3):
    VM, A1, A2, T, T4 = generate_transform_matrix(angle, alpha_1, alpha_2, R, theta3)
    for tform in [T, A1, A2, T4, VM]:
        points = apply_transform(points, tform)
    return points

# Assuming bands represent divisions in Z-axis:
band_indices = np.linspace(np.min(points[:, 2]), np.max(points[:, 2]), num=32 if puck == 2 else 16)

transformed_clouds = []

for i in range(len(band_indices) - 1):
    band_points = points[(points[:, 2] >= band_indices[i]) & (points[:, 2] < band_indices[i + 1])]
    for angle in angles:
        transformed_band = transform_band(band_points, angle, alpha_1, alpha_2, R, theta3)
        transformed_clouds.append(transformed_band)

merged_cloud = np.vstack(transformed_clouds)

# For saving the numpy array:
output_file = os.path.join(output, output_file_name + '.npy')
np.save(output_file, merged_cloud)

# The final cloud (merged_cloud) is a numpy array containing your processed points.
