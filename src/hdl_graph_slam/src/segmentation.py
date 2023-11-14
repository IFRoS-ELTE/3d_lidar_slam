# import pandas as pd

# # Read the CSV file (assuming the delimiter is a tab)
# df = pd.read_csv('lidar_data_from_lidar_test.csv', delimiter=',')
# print(df.columns)
# # # Extracting relevant columns for XYZ coordinates and intensity
# # xyz_intensity = df[['field.fields0.name', 'field.fields1.name', 'field.fields2.name', 'field.fields3.name']]

# # # Renaming columns for convenience
# # xyz_intensity.columns = ['x', 'y', 'z', 'intensity']

# # # Display the structured data (for visualization purposes)
# # print(xyz_intensity.head())






# # Extracting offsets for x, y, z, and intensity
# x_offset = df['field.fields0.offset'].values
# y_offset = df['field.fields1.offset'].values
# z_offset = df['field.fields2.offset'].values
# intensity_offset = df['field.fields3.offset'].values

# # Retrieve the data using the offsets
# x_values = df.iloc[:, x_offset].values
# y_values = df.iloc[:, y_offset].values
# z_values = df.iloc[:, z_offset].values
# intensity_values = df.iloc[:, intensity_offset].values

# # Display a few values from each column for verification
# print("X values:")
# print(x_values[:5])  # Display the first 5 values of x

# print("\nY values:")
# print(y_values[:5])  # Display the first 5 values of y

# print("\nZ values:")
# print(z_values[:5])  # Display the first 5 values of z

# print("\nIntensity values:")
# print(intensity_values[:5])  # Display the first 5 values of intensity

import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np

# Open the ROS bag
bag_path = 'lidar_test.bag'  # Replace with the actual path to your ROS bag
bag = rosbag.Bag(bag_path)

# Specify the desired topic
topic = '/velodyne_points'

# Initialize an empty list to store XYZ coordinates
xyz_data = []

# Iterate through messages in the bag
for _, msg, _ in bag.read_messages(topics=[topic]):
    # Extract XYZ coordinates from PointCloud2 message
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        xyz_data.append([point[0], point[1], point[2]])

# Convert the list to a NumPy array
xyz_array = np.array(xyz_data)

# Close the ROS bag
bag.close()

# Save XYZ coordinates to a text file for PointNet++ input
output_file = 'pointnet_input.txt'  # Specify the desired output file name

# Write the number of points as the first line
with open(output_file, 'w') as file:
    file.write(f"{xyz_array.shape[0]}\n")

# Append XYZ coordinates to the text file
with open(output_file, 'a') as file:
    np.savetxt(file, xyz_array, fmt='%.6f')

# Now, the XYZ coordinates are saved in the 'pointnet_input.txt' file
print(f'XYZ coordinates saved to {output_file}')

