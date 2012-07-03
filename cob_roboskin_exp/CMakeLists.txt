cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(fts_listener src/fts_listener.cpp)
rosbuild_add_executable(roboskin_listener src/roboskin_listener.cpp)
rosbuild_add_executable(roboskin_trajectory_cancel src/roboskin_trajectory_cancel.cpp)
rosbuild_add_executable(calibration src/calibration.cpp)

rosbuild_add_executable(cancel_topic_export_data src/cancel_topic_export_data.cpp)
rosbuild_add_executable(fts_export_data src/fts_export_data.cpp)
rosbuild_add_executable(rs_export_data src/rs_export_data.cpp)
rosbuild_add_executable(min_export_data src/min_export_data.cpp)
rosbuild_add_executable(trajectory_export_data src/trajectory_export_data.cpp)

rosbuild_add_executable(roboskin1 src/roboskin1.cpp)
rosbuild_add_executable(roboskin2 src/roboskin2.cpp)

rosbuild_add_executable(min_pub src/min_pub.cpp)
