# 3d_lidar_slam
The ***Pomona 3D Graph Slam offline environment mapping*** project aims to generate the map indoor and outdoor environment based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. In this project, we utilizes the GPS, IMU to correct the pose graph and compare the performance of mapping between these sensors. 


[![Build](https://github.com/koide3/hdl_graph_slam/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/hdl_graph_slam/actions/workflows/build.yml) on melodic & noetic

## Implementation

1. Set ROS parameter for simulated time:
- Open a terminal and set the *use_sim_time* parameter to true. This is required for working with bag files, as it allows ROS to use simulated time:
```bash
rosparam set use_sim_time true
```
2. Launch hdl_graph_slam nodes:
- Open a new terminal and launch the HDL Graph SLAM nodes using the provided launch file:
 - For indoor environment:
  ```bash
  roslaunch hdl_graph_slam hdl_graph_slam_501.launch
  ```
  - For outdoor environment:
  ```bash
  roslaunch hdl_graph_slam_400.launch
  ```
3. Open RViz for visualization:
- In a new terminal, navigate to the rviz configuration file for HDL Graph SLAM:
```bash
roscd hdl_graph_slam/rviz
```
- Now, start RViz with the hdl_graph_slam configuration file:
```bash
rviz -d hdl_graph_slam.rviz
```
4. Play the bag file:
- In a new terminal, start playing the filtered bag file with the *rosbag play* command. The *--clock* option is used to sync the playback with the simulated time set earlier:
```bash
rosbag play --clock rosbag_file
```
5. Define the transformation between your sensors (LIDAR, IMU, GPS) and base_link of your system using static_transform_publisher (see line #11, hdl_graph_slam.launch). All the sensor data will be transformed into the common base_link frame, and then fed to the SLAM algorithm.

2. Remap the point cloud topic of ***prefiltering_nodelet***. Like:

```bash
  <node pkg="nodelet" type="nodelet" name="prefiltering_nodelet" ...
    <remap from="/velodyne_points" to="/rslidar_points"/>
  ...
```

## Constraints (Edges)

In this project, we utilized these following contraints to generate the map.

- ***Loop closure***

- ***GPS***
  - */gnss* (gps from pomona toolkit)


- ***IMU acceleration (gravity vector)***
  - */imu/imu_data* (sensor_msgs/Imu from pomona toolkit)

- ***Floor plane***
  - */floor_detection/floor_coeffs* (hdl_graph_slam/FloorCoeffs)

This constraint optimizes the graph so that the floor planes (detected by RANSAC) of the pose nodes becomes the same. This is designed to compensate the accumulated rotation error of the scan matching in large flat indoor environments.


## Parameters
All the configurable parameters are listed in *launch/hdl_graph_slam.launch* as ros params.


## Requirements
This project is running based on the [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) package that requires the following libraries:

- OpenMP
- PCL
- g2o
- suitesparse

The following ROS packages are required:

- geodesy
- nmea_msgs
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)

```bash
# for melodic
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git -b melodic
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam

cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release

# for noetic
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam

cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
```

**[optional]** *bag_player.py* script requires ProgressBar2.
```bash
sudo pip install ProgressBar2
```

## Example1 (Indoor)

Bag file (recorded in CLC building at ELTE):

- [hdl_501.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501.bag.tar.gz) (raw data, 344MB)
- [hdl_501_filtered.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501_filtered.bag.tar.gz) (downsampled data, 57MB, **Recommended!**)

```bash
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_501.launch
```

```bash
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_501_filtered.bag
```

We also provide bag_player.py which automatically adjusts the playback speed and processes data as fast as possible.

```bash
rosrun hdl_graph_slam bag_player.py hdl_501_filtered.bag
```

You'll see a point cloud like:

<img src="imgs/lidarindoor.png" height="256pix" /> <img src="imgs/lidarindoor2.png" height="256pix" />

You can save the generated map by:
```bash
rosservice call /hdl_graph_slam/save_map "resolution: 0.05
destination: '/full_path_directory/map.pcd'"
```

## Example2 (Outdoor) without IMU, GPS
In this example, we solely captured lidar data in an outdoor setting. The outcome shows distortion and overlapping between the lidar scans. A superior outcome was achieved when comparing it to example 3, where GPS and IMU data were integrated to elevate the pose graph's performance.

Bag file (recorded in an outdoor environment):
- [hdl_400.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz) (raw data, about 900MB)

```bash
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```

```bash
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_400.bag
```

<img src="imgs/lidaroutdoor.png" height="256pix" /> <img src="imgs/lidaroutdoor2.png" height="256pix" />

## Example2 (Outdoor) with IMU, GPS

In this example, we utilizes the GPS data and IMU data to correct the pose graph. As depicted in the images, the building is accurately represented without distortion, providing a clear view of the environment.


<img src="imgs/imugnsslidaroutdoor.png" height="256pix" /> <img src="imgs/lidarimugnss2outdoor.png" height="256pix" />

## Common Problems

### Parameter tuning guide

The mapping quality largely depends on the parameter setting. In particular, scan matching parameters have a big impact on the result. Tune the parameters accoding to the following instructions:

- ***registration_method***
  **[updated] In short, use FAST_GICP for most cases and FAST_VGICP or NDT_OMP if the processing speed matters** This parameter allows to change the registration method to be used for odometry estimation and loop detection. Note that GICP in PCL1.7 (ROS kinetic) or earlier has a bug in the initial guess handling. **If you are on ROS kinectic or earlier, do not use GICP**.

- ***ndt_resolution***
  This parameter decides the voxel size of NDT. Typically larger values are good for outdoor environements (0.5 - 2.0 [m] for indoor, 2.0 - 10.0 [m] for outdoor). If you chose NDT or NDT_OMP, tweak this parameter so you can obtain a good odometry estimation result.

- ***other parameters***
  All the configurable parameters are available in the launch file. Copy a template launch file (hdl_graph_slam_501.launch for indoor, hdl_graph_slam_400.launch for outdoor) and tweak parameters in the launch file to adapt it to your application.

## License

This package is released under the BSD-2-Clause License.


Note that the cholmod solver in g2o is licensed under GPL. You may need to build g2o without cholmod dependency to avoid the GPL.

## Related packages

- [interactive_slam](https://github.com/koide3/interactive_slam)
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)
- [hdl_localization](https://github.com/koide3/hdl_localization)
- [hdl_people_tracking](https://github.com/koide3/hdl_people_tracking)

<!-- <img src="imgs/packages.png"/> -->

## Papers
Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, Advanced Robotic Systems, 2019 [[link]](https://www.researchgate.net/publication/331283709_A_Portable_3D_LIDAR-based_System_for_Long-term_and_Wide-area_People_Behavior_Measurement).

## Contact
Kenji Koide, k.koide@aist.go.jp, https://staff.aist.go.jp/k.koide

Active Intelligent Systems Laboratory, Toyohashi University of Technology, Japan [\[URL\]](http://www.aisl.cs.tut.ac.jp)  
Mobile Robotics Research Team, National Institute of Advanced Industrial Science and Technology (AIST), Japan  [\[URL\]](https://unit.aist.go.jp/hcmrc/mr-rt/contact.html)
