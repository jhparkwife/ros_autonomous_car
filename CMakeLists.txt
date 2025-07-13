cmake_minimum_required(VERSION 3.0.2)
project(lane_follower_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  cv_bridge
  image_transport
)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

catkin_install_python(PROGRAMS
  scripts/enhanced_lane_follower.py
  scripts/navigation_node.py
  scripts/diagnostics_monitor.py
  nodes/system_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)