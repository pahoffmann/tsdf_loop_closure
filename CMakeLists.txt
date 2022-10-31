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
  message_filters
  eigen_conversions
  tf2_ros
  tf2
  bondcpp
  fast_gicp
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
set(Boost_USE_STATIC_LIBS ON)

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

# find pcl
find_package(PCL REQUIRED)

#boost
FIND_PACKAGE(Boost REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# openmp
find_package(OpenMP)

###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes) # Load build type flags and default to Debug mode
#include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
#include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###### GTSAM STUFF ENDS HER

## teaser ++ ##
find_package(teaserpp REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

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
                        src/path/path_exploration.cpp
                        src/map/attribute_data_model.cpp)

add_executable(path_listener src/nodes/path_listener.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp
                        src/map/attribute_data_model.cpp)


add_executable(test_lc_detection src/nodes/test_lc_detection.cpp 
                        src/path/path.cpp
                        src/ray_tracer/ray_tracer.cpp
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
                        src/path/path_exploration.cpp
                        src/map/attribute_data_model.cpp)

add_executable(test_localmap src/nodes/test_localmap.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp
                        src/data_association/association.cpp
                        src/data_association/association_manager.cpp
                        src/path/path_exploration.cpp
                        src/map/attribute_data_model.cpp)

add_executable(test_map_cleanup src/nodes/test_map_cleanup.cpp 
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp
                        src/map/attribute_data_model.cpp)

#pcd tsdf update test
add_executable(pcd2tsdf src/nodes/pcd2tsdf.cpp 
                        src/util/update_tsdf.cpp
                        src/util/runtime_evaluator.cpp
                        src/map/local_map.cpp
                        src/map/global_map.cpp
                        src/map/attribute_data_model.cpp)

add_executable(transform_coord_sys src/nodes/transform_coord_sys.cpp 
                        src/util/runtime_evaluator.cpp
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/map/attribute_data_model.cpp
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp)

add_executable(pcd_hdf5_publisher src/nodes/data_publishers/pcd_hdf5_publisher.cpp 
                        src/util/runtime_evaluator.cpp
                        src/map/global_map.cpp 
                        src/map/local_map.cpp 
                        src/map/attribute_data_model.cpp
                        src/ray_tracer/ray_tracer.cpp
                        src/path/path.cpp)

add_executable(slam6d_listener src/nodes/slam6d_listener.cpp 
                        src/util/runtime_evaluator.cpp
                        src/map/global_map.cpp 
                        src/map/local_map.cpp
                        src/util/update_tsdf.cpp
                        src/map/attribute_data_model.cpp
                        src/ray_tracer/ray_tracer.cpp
                        src/data_association/association.cpp
                        src/data_association/association_manager.cpp
                        src/path/path.cpp
                        src/gtsam/gtsam_wrapper.cpp
                        src/util/lc_evaluator.cpp
                        src/map/map_updater.cpp
                        src/util/csv_wrapper.cpp)

add_executable(test_partial_update src/nodes/test_partial_update.cpp 
                        src/util/runtime_evaluator.cpp
                        src/map/global_map.cpp 
                        src/map/local_map.cpp
                        src/util/update_tsdf.cpp
                        src/map/attribute_data_model.cpp
                        src/ray_tracer/ray_tracer.cpp
                        src/data_association/association.cpp
                        src/data_association/association_manager.cpp
                        src/path/path.cpp
                        src/gtsam/gtsam_wrapper.cpp
                        src/util/lc_evaluator.cpp
                        src/map/map_updater.cpp)

                  
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
target_compile_definitions(slam6d_listener PRIVATE DEBUG)
target_compile_definitions(test_partial_update PRIVATE DEBUG)

# compile definition for debug in testing environment
add_compile_definitions(TEST_DEBUG=1)


# globally available data path
add_compile_definitions(DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")

## Specify libraries to link a library or executable target against

target_link_libraries(loop_closure
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)

target_link_libraries(path_listener
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_lc_detection
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)


target_link_libraries(test_map_update
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_localmap
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_map_cleanup
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(pcd2tsdf PUBLIC
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX 
)

target_link_libraries(transform_coord_sys PUBLIC
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX 
)

target_link_libraries(pcd_hdf5_publisher PUBLIC
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX 
)

target_link_libraries(slam6d_listener
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  ${JSONCPP_LIBRARIES}
  teaserpp::teaser_registration
  teaserpp::teaser_features
  teaserpp::teaser_io
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)

target_link_libraries(test_partial_update
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
  teaserpp::teaser_registration
  teaserpp::teaser_features
  teaserpp::teaser_io
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)

## Tests ##

catkin_add_gtest(${PROJECT_NAME}-test test/test_lc_rejectors.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${catkin_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
    Eigen3::Eigen
    ${JSONCPP_LIBRARIES}
    teaserpp::teaser_registration
    teaserpp::teaser_features
    teaserpp::teaser_io
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    OpenMP::OpenMP_CXX
    gtsam
  )
endif()


