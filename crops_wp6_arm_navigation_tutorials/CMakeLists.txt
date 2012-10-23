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
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(get_state_validity src/get_state_validity.cpp)
#rosbuild_add_executable(get_state_validity_virtual_object src/get_state_validity_virtual_object.cpp)
#rosbuild_add_executable(get_state_validity_virtual_attached_object src/get_state_validity_virtual_attached_object.cpp)
#rosbuild_add_executable(get_joint_trajectory_validity src/get_joint_trajectory_validity.cpp)
rosbuild_add_executable(get_solver_info src/get_solver_info.cpp)
rosbuild_add_executable(get_fk src/get_fk.cpp)
rosbuild_add_executable(get_ik src/get_ik.cpp)
rosbuild_add_executable(get_collision_free_ik src/get_collision_free_ik.cpp)
rosbuild_add_executable(test_planning_scene src/test_planning_scene.cpp)
#rosbuild_add_executable(get_collision_free_ik_other_base src/get_collision_free_ik_other_base.cpp)
#rosbuild_add_executable(collision_free_trajectory_control src/collision_free_trajectory_control.cpp)
rosbuild_add_executable(move_arm_joint_goal src/move_arm_joint_goal.cpp)
rosbuild_add_executable(ik_trajectory src/ik_trajectory_tutorial.cpp)
rosbuild_add_executable(move_arm_simple_pose_goal src/move_arm_simple_pose_goal.cpp)
#rosbuild_add_executable(move_arm_pose_goal src/move_arm_pose_goal.cpp)
#rosbuild_add_executable(move_arm_path_constraints src/move_arm_path_constraints.cpp)
#rosbuild_add_executable(trajectory_filter_test src/trajectory_filter_test.cpp)
rosbuild_add_executable(display_trajectory_tutorial src/display_trajectory_tutorial.cpp)
#rosbuild_add_executable(add_cylinder src/addCylinder.cpp)
#rosbuild_add_executable(attach_cylinder src/attachCylinder.cpp)
