cmake_minimum_required(VERSION 3.0.2)
project(msim_sensors)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs livox_ros_driver)
catkin_package()
catkin_install_python(PROGRAMS scripts/imu_model_node.py scripts/lidar_format_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
