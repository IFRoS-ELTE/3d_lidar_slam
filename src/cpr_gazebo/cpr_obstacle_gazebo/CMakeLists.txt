cmake_minimum_required(VERSION 2.8.3)
project(cpr_obstacle_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch models rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
