cmake_minimum_required(VERSION 3.0.2)
project(lidar_nodes)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs visualization_msgs std_msgs tf)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs geometry_msgs visualization_msgs std_msgs tf)
catkin_install_python(PROGRAMS src/input_and_prefilter.py src/clustering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
configure_file(src/clustering_realtime.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/clustering_realtime.py COPYONLY)
install(FILES src/clustering_realtime.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
