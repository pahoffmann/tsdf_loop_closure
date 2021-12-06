cmake_minimum_required(VERSION 3.0.2)
project(loop_closure)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  dynamic_reconfigure
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/LoopClosure.cfg
)

###########
## Build ##
###########

# External interface for HDF5 (libhdf5) HighFive (https://github.com/BlueBrain/HighFive).
set(HIGHFIVE_EXAMPLES FALSE CACHE INTERNAL "")
set(HIGHFIVE_UNIT_TESTS FALSE CACHE INTERNAL "")
add_subdirectory(ext/HighFive)
include_directories("ext/HighFive/include")
find_package(HDF5 REQUIRED COMPONENTS CXX C HL)
include_directories(${HDF5_INCLUDE_DIRS})

#eigen3
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(map_node src/map_node.cpp src/map/global_map.cpp src/map/local_map.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(map_node
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
)

# dynamic reconfigure
add_dependencies(map_node loop_closure_gencfg ${catkin_EXPORTED_TARGETS})
