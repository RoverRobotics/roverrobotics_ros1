cmake_minimum_required(VERSION 3.8.0)
project(roverrobotics_driver)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  pluginlib
  nav_msgs
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  message_runtime 
  pluginlib 
  roscpp 
  std_msgs 
  sensor_msgs 
  ds4_driver
  nav_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(librover SHARED IMPORTED)
set_target_properties(librover PROPERTIES
  IMPORTED_LOCATION "/usr/lib/liblibrover.so"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/librover")
add_executable(roverrobotics_driver_node src/roverrobotics_ros_driver.cpp)
##add_dependencies(roverrobotics_driver_node roverrobotics_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


target_link_libraries(roverrobotics_driver_node ${catkin_LIBRARIES} librover)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS roverrobotics_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ##Install Linked Library
# install(TARGETS librover
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

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
