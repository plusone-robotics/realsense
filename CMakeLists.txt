cmake_minimum_required(VERSION 2.8.3)
project(realsense_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

## Build ##
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Install ##
foreach(dir launch config)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY launch/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

catkin_install_python(PROGRAMS nodes/realsense_reboot_test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
