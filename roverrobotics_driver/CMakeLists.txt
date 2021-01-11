cmake_minimum_required(VERSION 2.8.3)
project(roverrobotics_driver)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  roverrobotics_driver_msgs
  pluginlib
  serial
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   roverrobotics_driver_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime pluginlib roscpp std_msgs roverrobotics_driver_msgs sensor_msgs serial ds4_driver
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(support_lib src/protocol_base.cpp
                         src/protocol_pro.cpp
                         src/protocol_zero.cpp
                         src/comm_manager.cpp
                         src/utils.cpp)

add_executable(roverrobotics_driver_node src/ros_top_node.cpp)
add_dependencies(roverrobotics_driver_node 
                                
                                roverrobotics_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
##target_link_libraries(roverrobotics_driver_node ${catkin_LIBRARIES} support_lib)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS roverrobotics_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##Install Linked Library
install(TARGETS support_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Copy any python scripts
##catkin_install_python(PROGRAMS scripts/diagnostics.py
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
##)

#############
## Testing ##
#############

# TODO
