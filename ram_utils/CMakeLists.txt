cmake_minimum_required(VERSION 3.2)
project(ram_utils)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  message_runtime
  ram_msgs
  rosbag
  roscpp
  roslib
  rospy
  std_msgs
  tf2_ros
  unique_id
  uuid_msgs
)

find_package(VTK 8.0.0 REQUIRED)
include(${VTK_USE_FILE})

find_package(yaml-cpp REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  AddEntryExitStrategies.srv
  BufferParams.srv
  EntryExitParameters.srv
  ExportTrajectory.srv
  GetFillParameters.srv
  GetLayerSize.srv
  GetNumberOfLayersLevels.srv
  GetStartPose.srv
  GetTool.srv
  GetTrajectoryFrame.srv
  GetTrajectoryInfos.srv
  GetTrajectorySize.srv
  ImportTrajectory.srv
  ParseSvgFile.srv
  UnmodifiedTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  ram_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_file_extension
  ${PROJECT_NAME}_trajectory_files_manager_1
  ${PROJECT_NAME}_trajectory_files_manager_2
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  ram_msgs
  rosbag
  roscpp
  roslib
  rospy
  std_msgs
  uuid_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Frame / TF broadcaster
add_executable(
  ${PROJECT_NAME}_frames_tf_broadcaster
  src/frames_tf_broadcaster.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_frames_tf_broadcaster
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_frames_tf_broadcaster
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Fill trajectory parameters
add_executable(
  ${PROJECT_NAME}_fill_trajectory_parameters
  src/fill_trajectory_parameters.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_fill_trajectory_parameters
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_fill_trajectory_parameters
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Trajectory info
add_executable(
  ${PROJECT_NAME}_trajectory_info
  src/trajectory_info.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_trajectory_info
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_trajectory_info
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Entry_exit_strategies
add_executable(
  ${PROJECT_NAME}_entry_exit_strategies
  src/entry_exit_strategies.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_entry_exit_strategies
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_entry_exit_strategies
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Trajectory files manager 1 and 2
add_library(
  ${PROJECT_NAME}_trajectory_files_manager_1
  src/trajectory_files_manager_1.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_trajectory_files_manager_1
  ${YAML_CPP_LIBRARIES}
  ${VTK_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_trajectory_files_manager_1
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_trajectory_files_manager_2
  src/trajectory_files_manager_2.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_trajectory_files_manager_2
  ${YAML_CPP_LIBRARIES}
  ${VTK_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_trajectory_files_manager_2
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# File extension library
add_library(
  ${PROJECT_NAME}_file_extension
  src/file_extension.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_file_extension
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_file_extension
  ${catkin_EXPORTED_TARGETS}
)

# Buffer and import/export
add_executable(
  ${PROJECT_NAME}
  src/trajectory_utils.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_file_extension
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_file_extension
  ${catkin_EXPORTED_TARGETS}
)

# Utility to fix trajectory with missing UUIDs
add_executable(
  ${PROJECT_NAME}_fix_uuid
  src/fix_uuid.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_fix_uuid
  ${PROJECT_NAME}_file_extension
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_file_extension
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_entry_exit_strategies
  ${PROJECT_NAME}_file_extension
  ${PROJECT_NAME}_fill_trajectory_parameters
  ${PROJECT_NAME}_fix_uuid
  ${PROJECT_NAME}_frames_tf_broadcaster
  ${PROJECT_NAME}_trajectory_files_manager_1
  ${PROJECT_NAME}_trajectory_files_manager_2
  ${PROJECT_NAME}_trajectory_info
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install python scripts
catkin_install_python(
  PROGRAMS
  scripts/svg_parser.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    ${PROJECT_NAME}_trajectory_info_test
    test/trajectory_info_test.launch
    test/trajectory_info_test.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_trajectory_info_test
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    ${PROJECT_NAME}_trajectory_info_test
    ${PROJECT_NAME}_trajectory_info
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    ${PROJECT_NAME}_services_clients
    test/services_clients.launch
    test/services_clients.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_services_clients
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    ${PROJECT_NAME}_services_clients
    ${PROJECT_NAME}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    trajectory_files_manager_test
    test/trajectory_files_manager_test.launch
    test/trajectory_files_manager_test.cpp
  )
  target_link_libraries(
    trajectory_files_manager_test
    ${PROJECT_NAME}_trajectory_files_manager_1
    ${catkin_LIBRARIES}
  )
endif()
