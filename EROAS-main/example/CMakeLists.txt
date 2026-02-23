cmake_minimum_required(VERSION 3.0.2)
project(eroas_example)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/pose_gt_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
