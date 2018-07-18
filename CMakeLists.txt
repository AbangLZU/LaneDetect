cmake_minimum_required(VERSION 3.5.1)
project(Lane_Detect)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR})
find_package(OpenCV REQUIRED)
add_executable(Lane_Detect main.cpp laneDetection.cpp laneDetection.h calibration.cpp calibration.h)
target_link_libraries(Lane_Detect ${OpenCV_LIBS})
