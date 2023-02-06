cmake_minimum_required(VERSION 3.2)
project(ram_modify_trajectory)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  message_generation
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  unique_id
  uuid_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  BlendRadiusModification.msg
  PoseInformationParameters.msg
)

add_service_files(
  FILES
  AddPoses.srv
  AddToSelection.srv
  ChangeLayerHeight.srv
  DeleteSelectedPoses.srv
  EraseSelection.srv
  GetPosesFromInformation.srv
  GetPosesFromLayersList.srv
  GetPosesFromLayer.srv
  GetPosesFromTrajectory.srv
  GetSelection.srv
  InvertSelection.srv
  ModifySelectedPoses.srv
  PushPullAngle.srv
  ReflectSelectedPoses.srv
  RemoveFromSelection.srv
  ResetSelectedPoses.srv
  RotateSelectedPoses.srv
  ScaleSelectedPoses.srv
  ShiftPoses.srv
  SimplifyTrajectory.srv
  TrajectoryInterruption.srv
  TweakBlendRadius.srv
)

generate_messages(
  DEPENDENCIES
  ram_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  eigen_conversions
  message_runtime
  ram_msgs
  ram_utils
  roscpp
  uuid_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Pose selector
add_executable(
  ${PROJECT_NAME}_pose_selector
  src/pose_selector.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_pose_selector
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}_pose_selector
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Modify trajectory
add_executable(
  ${PROJECT_NAME}
  src/modify_trajectory.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_pose_selector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    ${PROJECT_NAME}_pose_selector_test
    test/pose_selector_test.launch
    test/pose_selector_test.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_pose_selector_test
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    ${PROJECT_NAME}_pose_selector_test
    ${PROJECT_NAME}_pose_selector
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )
endif()
