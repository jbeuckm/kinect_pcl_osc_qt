cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_pcl_osc)

find_package(PCL 1.7 REQUIRED)
find_package(Qt4 4.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinect_pcl_osc main.cpp)
target_link_libraries (kinect_pcl_osc ${PCL_LIBRARIES})

