# import rosbag
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2

# bag_file = '/home/preeti/elte_ws/lidar_test.bag'  # Replace with your ROS bag file
# output_file = 'lidar_test.xyz'  # File to store x, y, z coordinates in XYZ format

# # try:
# #     with open(output_file, 'w') as file:
# #         bag = rosbag.Bag(bag_file, 'r')
# #         for topic, msg, t in bag.read_messages(topics=['/velodyne_points']):
# #             print(f"Topic: {topic}")  # Check the topic being read
# #             if topic == '/velodyne_points':
# #                 print("Message Type: ", type(msg))  # Check the message type
# #                 if isinstance(msg, PointCloud2):
# #                     for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
# #                         x, y, z = point
# #                         print(f"X: {x}, Y: {y}, Z: {z}")  # Verify the extracted points
# #                         file.write(f"{x} {y} {z}\n")
# #                 else:
# #                     print("Not a PointCloud2 message")
# #             else:
# #                 print("Unexpected topic")
# #         bag.close()
# # except Exception as e:
# #     print(f"Error: {e}")


# try:
#     with open(output_file, 'w') as file:
#         bag = rosbag.Bag(bag_file, 'r')
#         for topic, msg, t in bag.read_messages():
#             print(f"Topic: {topic}")
#             print(f"Raw Message: {msg}")
#             if topic == '/velodyne_points' and isinstance(msg, PointCloud2):
#                 for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                     x, y, z = point
#                     file.write(f"{x} {y} {z}\n")
#         bag.close()
# except Exception as e:
#     print(f"Error: {e}")


import rosbag
import struct
import os
# Input and output file paths
input_bag_file = '/home/preeti/elte_ws/lidar_test.bag'  # Replace with your ROS bag file
output_xyz_file = 'lidar_test.xyz'  # File to store x, y, z coordinates in XYZ format
output_xyz_file = os.path.join(os.getcwd(), output_xyz_file)
# Open the bag file
bag = rosbag.Bag(input_bag_file, 'r')

# Define the topic name
topic = '/velodyne_points'  # Replace with your specific topic name

# Open the output file in write mode
with open(output_xyz_file, 'w') as xyz_file:
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Check if the message is of type sensor_msgs/PointCloud2
        if msg._type == 'sensor_msgs/PointCloud2':
            # Extract point cloud data
            point_data = list(struct.unpack('fff', msg.data))  # Assuming the data follows x, y, z format

            # Write to the output file in XYZ format
            x, y, z = point_data
            xyz_file.write(f"{x} {y} {z}\n")

# Close the bag
bag.close()
