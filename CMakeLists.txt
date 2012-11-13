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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_library(${PROJECT_NAME} src/set_marker.cpp)

rosbuild_add_executable(collect_data src/collect_data.cpp
				src/planar_seg.cpp
				src/pass_through_gen.cpp)
#rosbuild_add_executable(cylinder_seg src/cylindrical_segmentation.cpp)
rosbuild_add_executable(euclidean_seg src/euclidean_segmentation.cpp)

rosbuild_add_executable(color_seg src/color_segmentation.cpp)

#rosbuild_add_executable(post_seg src/post_segmentation.cpp
#				src/find_extents.cpp
#				src/set_marker.cpp
#				src/find_centroid.cpp)

rosbuild_add_executable(pp_try src/pick_place_try.cpp)
rosbuild_add_executable(push_try src/push_try.cpp src/find_extents.cpp)

rosbuild_add_executable(move_right_arm src/move_right_arm.cpp)
rosbuild_add_executable(move_left_arm src/move_left_arm.cpp)
rosbuild_add_executable(move_elbow src/move_elbow.cpp)
rosbuild_add_executable(arm_test src/move_arm.cpp)

rosbuild_add_executable(move_head_manual src/move_head_manual.cpp)

rosbuild_add_executable(pointcloud_to_hull src/TableTopObject.cpp src/pointcloud_to_hull.cpp)
target_link_libraries(pointcloud_to_hull gpcl octomap octomath)

rosbuild_add_executable(ptohm src/TableTopObject.cpp src/pointcloud_to_hull_megha.cpp)
target_link_libraries(ptohm gpcl octomap octomath)

rosbuild_add_executable(motion_planning src/motion_planning.cpp)