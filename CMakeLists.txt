cmake_minimum_required(VERSION 3.0.0)

project(pointclouds)
add_compile_options(-std=c++11)
# Lib RealSense

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(${OpenCV_DIR} /home/tapati/OpenCV/opencv/build)
find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(OpenMP REQUIRED)
add_executable(${PROJECT_NAME} "main.cpp")
#include_directories(${realsense_INCLUDE_DIR})
include_directories(${realsense2_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${ADDITIONAL_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS})


target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${realsense2_LIBRARY} OpenMP::OpenMP_CXX ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} ${REALSENSE2_LIBRARIES})

