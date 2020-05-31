cmake_minimum_required(VERSION 3.5)
project(ira_laser_tools)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

# generate_dynamic_reconfigure_options(cfg/laserscan_multi_merger.cfg
#   cfg/laserscan_virtualizer.cfg)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

#add_definitions(${EIGEN_DEFINITIONS})

## Declare a cpp library
# add_library(laser_merger
#   src/${PROJECT_NAME}/laser_merger.cpp
# )

## Declare a cpp executable
#add_executable(laser_merger_node src/laser_merger_node.cpp)
#target_link_libraries(laser_merger_node ${catkin_LIBRARIES})

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(laserscan_multi_merger ${PROJECT_NAME}_gencfg)

# add_executable(laserscan_virtualizer src/laserscan_virtualizer.cpp)
find_package(ament_cmake REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${REQUIRED_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${laser_geometry_LIBRARIES} ${std_msgs_LIBRARIES} ${tf2_LIBRARIES}
  ${pcl_ros_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES} ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES})
target_link_libraries(laserscan_multi_merger ${LIBRARY_DIRS})
# target_link_libraries(laserscan_virtualizer ${LIBRARY_DIRS})
set(LIBS ${ament_cmake_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${laser_geometry_LIBRARIES} ${std_msgs_LIBRARIES} ${tf2_LIBRARIES}
  ${Eigen3_LIBRARIES} ${PCL_LIBRARIES} ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(laserscan_virtualizer ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION bin
# )

## Mark executables and/or libraries for installation
# install(TARGETS laserscan_multi_merger laserscan_virtualizer
#   ARCHIVE
#   DESTINATION lib
#   LIBRARY
#   DESTINATION lib
#   RUNTIME
#   DESTINATION bin)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_laser_merger.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(laser_geometry)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(Eigen3)
ament_export_dependencies(PCL)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
