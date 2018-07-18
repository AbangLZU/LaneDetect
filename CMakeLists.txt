cmake_minimum_required(VERSION 3.5.1)
project(Lane_Detect)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR})
find_package(OpenCV REQUIRED)
add_executable(Lane_Detect main.cpp laneDetection.cpp laneDetection.h calibration.cpp calibration.h)
target_link_libraries(Lane_Detect ${OpenCV_LIBS})


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#
#set(CMAKE_INSTALL_PREFIX ..)
#
##Files installation path
#install(DIRECTORY camera_cal/
#        DESTINATION LaneDetection/camera_cal)
#install(DIRECTORY build/bin/
#        DESTINATION LaneDetection/
#        FILES_MATCHING PATTERN laneDetection
#        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
#        GROUP_EXECUTE GROUP_READ)
#install(DIRECTORY video/
#        DESTINATION LaneDetection/)
