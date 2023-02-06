cmake_minimum_required(VERSION 3.2)
project(industrial_robot_angle_conversions)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

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
  ${PROJECT_NAME}_angles
  ${PROJECT_NAME}_poses
  CATKIN_DEPENDS
  eigen_conversions
  roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_angles
  src/angles.cpp
)

add_library(
  ${PROJECT_NAME}_poses
  src/poses.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_poses
  ${PROJECT_NAME}_angles
)
add_dependencies(
  ${PROJECT_NAME}_poses
  ${PROJECT_NAME}_angles
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
  ${PROJECT_NAME}_angles
  ${PROJECT_NAME}_poses
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
    ${PROJECT_NAME}_tests
    test/angles_and_poses.launch
    test/angles_and_poses.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}_angles
    ${PROJECT_NAME}_poses
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}_angles
    ${PROJECT_NAME}_poses
    ${catkin_EXPORTED_TARGETS}
  )
endif()
