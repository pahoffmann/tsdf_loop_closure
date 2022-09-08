cmake_minimum_required(VERSION 3.0.2)
project(loop_closure LANGUAGES)# CXX CUDA)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17) # use -std=c++11 for cuda, check how to fix the warnings

# add a few cmake additions to run things more smoothly
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
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
  cfg/RayTracer.cfg
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

# json
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

#boost
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# openmp
find_package(OpenMP)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(loop_closure src/nodes/loop_closure_node.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp
                        src/data_association/association.cpp
                        src/data_association/association_manager.cpp
                        src/options/options_reader.cpp
                        src/path/path_exploration.cpp
                        src/map/attribute_data_model.cpp)

add_executable(path_listener src/nodes/path_listener.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp
                        src/map/attribute_data_model.cpp)


add_executable(test_lc_detection src/nodes/test_lc_detection.cpp 
                        src/path/path.cpp
                        src/ray_tracer/ray_tracer.cpp
                        src/options/options_reader.cpp
                        src/map/global_map.cpp 
                        src/map/local_map.cpp
                        src/map/attribute_data_model.cpp)

add_executable(test_map_update src/nodes/test_map_update.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp
                        src/data_association/association.cpp
                        src/data_association/association_manager.cpp
                        src/options/options_reader.cpp
                        src/path/path_exploration.cpp
                        src/map/attribute_data_model.cpp)

add_executable(test_localmap src/nodes/test_localmap.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp
                        src/data_association/association.cpp
                        src/data_association/association_manager.cpp
                        src/options/options_reader.cpp
                        src/path/path_exploration.cpp
                        src/map/attribute_data_model.cpp)

add_executable(test_map_cleanup src/nodes/test_map_cleanup.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp
                        src/options/options_reader.cpp
                        src/map/attribute_data_model.cpp)
                  
# currently only for the test node.
#set_target_properties(lc_test_node PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

include_directories(include)

# dynamic reconfigure
add_dependencies(loop_closure loop_closure_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(test_map_update loop_closure_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(test_localmap loop_closure_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(test_map_cleanup loop_closure_gencfg ${catkin_EXPORTED_TARGETS})


# enable / disable debug by commenting either line for the specific target
target_compile_definitions(loop_closure PRIVATE DEBUG)
target_compile_definitions(test_map_update PRIVATE DEBUG)
target_compile_definitions(test_localmap PRIVATE DEBUG)
target_compile_definitions(test_map_cleanup PRIVATE DEBUG)

## Specify libraries to link a library or executable target against

target_link_libraries(loop_closure
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(path_listener
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_lc_detection
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)


target_link_libraries(test_map_update
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_localmap
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_map_cleanup
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

