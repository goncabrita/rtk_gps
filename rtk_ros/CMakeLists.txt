cmake_minimum_required(VERSION 2.8.3)
project(rtk_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp sensor_msgs std_msgs rtk_msgs rtklib cereal_port angles)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES UTMConverter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${rtklib_INCLUDE_DIRS} ${cereal_port_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(rtk_ros src/rtk_ros.cpp )

add_library(UTMConverter src/UTMConverter.cpp)

## Declare a cpp executable
# add_executable(rtk_ros_node src/rtk_ros.cpp)

add_executable(rtk_robot_node src/rtk_robot_node.cpp)

add_executable(rtk_base_station_node src/rtk_base_station_node.cpp)

add_executable(rtk_ppp_node src/rtk_ppp_node.cpp)

add_executable(save_ecef_file src/save_ecef_file.cpp)
add_executable(save_utm_file src/save_utm_file.cpp)

add_executable(save_fixed_position src/save_fixed_position.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rtklib_node rtklib_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rtklib_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(rtk_robot_node ${catkin_LIBRARIES} rtklib UTMConverter)

target_link_libraries(rtk_base_station_node ${catkin_LIBRARIES} cereal_port)

target_link_libraries(rtk_ppp_node ${catkin_LIBRARIES} rtklib)

target_link_libraries(save_ecef_file ${catkin_LIBRARIES})
target_link_libraries(save_utm_file ${catkin_LIBRARIES})

target_link_libraries(save_fixed_position ${catkin_LIBRARIES} UTMConverter)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rtklib rtklib_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtklib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



