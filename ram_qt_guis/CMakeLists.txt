cmake_minimum_required(VERSION 3.2)
project(ram_qt_guis)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  class_loader
  eigen_conversions
  geometry_msgs
  industrial_robot_angle_conversions
  ram_display
  ram_modify_trajectory
  ram_msgs
  ram_path_planning
  ram_trajectory
  ram_utils
  roscpp
  rqt_gui
  rviz
  tf2_ros
  tf_conversions
)

## Find Qt5
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)

set(
  PROJECT_QT_PANELS_TARGETS
  ${PROJECT_NAME}_display
  ${PROJECT_NAME}_entry_exit_strategies
  ${PROJECT_NAME}_fill_trajectory
  ${PROJECT_NAME}_frames
  ${PROJECT_NAME}_intralayers_display
  ${PROJECT_NAME}_modify
  ${PROJECT_NAME}_path_planning
  ${PROJECT_NAME}_pose_info
  ${PROJECT_NAME}_traj_info
  ${PROJECT_NAME}_trajectory_utils
  ${PROJECT_NAME}_simulate
)

set(
  PROJECT_TARGETS
  ${PROJECT_NAME}_modify_panel_blend_radius_table
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${PROJECT_NAME}_modify_panel_range_list_selection
  ${PROJECT_NAME}_modify_pose_information_parameters_filter
  ${PROJECT_NAME}_path_planning_contours_widget
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  ${PROJECT_NAME}_path_planning_polygon_offsets_widget
  ${PROJECT_NAME}_path_planning_profile_widget
  ${PROJECT_NAME}_path_planning_revolve_widget
  ${PROJECT_NAME}_pose
  ${PROJECT_NAME}_tree_button
  ${PROJECT_QT_PANELS_TARGETS}
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
  ${PROJECT_TARGETS}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  class_loader
  eigen_conversions
  geometry_msgs
  industrial_robot_angle_conversions
  ram_display
  ram_modify_trajectory
  ram_msgs
  ram_path_planning
  ram_trajectory
  ram_utils
  roscpp
  rqt_gui
  rviz
  tf2_ros
  tf_conversions
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Tree button widget
add_library(
  ${PROJECT_NAME}_tree_button
  include/${PROJECT_NAME}/tree_button.hpp
  src/tree_button.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_tree_button
  Qt5::Widgets
)

# Pose widget
add_library(
  ${PROJECT_NAME}_pose
  include/${PROJECT_NAME}/pose.hpp
  src/pose.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_pose
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

## Qt panel - Frames
add_library(
  ${PROJECT_NAME}_frames
  include/${PROJECT_NAME}/frames.hpp
  src/frames.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_frames
  ${PROJECT_NAME}_pose
  ${PROJECT_NAME}_tree_button
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_frames
  ${PROJECT_NAME}_pose
  ${PROJECT_NAME}_tree_button
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Display
add_library(
  ${PROJECT_NAME}_display
  include/${PROJECT_NAME}/display.hpp
  src/display.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_display
  ${PROJECT_NAME}_tree_button
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_display
  ${PROJECT_NAME}_tree_button
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Path planning
add_library(
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  include/${PROJECT_NAME}/path_planning_widgets/donghong_ding.hpp
  src/path_planning_widgets/donghong_ding.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_path_planning_contours_widget
  include/${PROJECT_NAME}/path_planning_widgets/contours.hpp
  src/path_planning_widgets/contours.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_contours_widget
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_contours_widget
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  include/${PROJECT_NAME}/path_planning_widgets/follow_poses.hpp
  src/path_planning_widgets/follow_poses.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_path_planning_polygon_offsets_widget
  include/${PROJECT_NAME}/path_planning_widgets/polygon_offsets.hpp
  src/path_planning_widgets/polygon_offsets.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_polygon_offsets_widget
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_polygon_offsets_widget
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_path_planning_revolve_widget
  include/${PROJECT_NAME}/path_planning_widgets/revolve.hpp
  src/path_planning_widgets/revolve.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_revolve_widget
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_revolve_widget
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_path_planning_profile_widget
  include/${PROJECT_NAME}/path_planning_widgets/profile.hpp
  src/path_planning_widgets/profile.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_profile_widget
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_profile_widget
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_path_planning
  include/${PROJECT_NAME}/path_planning.hpp
  src/path_planning.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning
  ${PROJECT_NAME}_path_planning_contours_widget
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  ${PROJECT_NAME}_path_planning_polygon_offsets_widget
  ${PROJECT_NAME}_path_planning_profile_widget
  ${PROJECT_NAME}_path_planning_revolve_widget
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_path_planning
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Fill trajectory parameters
add_library(
  ${PROJECT_NAME}_fill_trajectory
  include/${PROJECT_NAME}/fill_trajectory.hpp
  src/fill_trajectory.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_fill_trajectory
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_fill_trajectory
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Trajectory utils
add_library(
  ${PROJECT_NAME}_trajectory_utils
  include/${PROJECT_NAME}/trajectory_utils.hpp
  src/trajectory_utils.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_trajectory_utils
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_trajectory_utils
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Pose info panel
add_library(
  ${PROJECT_NAME}_pose_info
  include/${PROJECT_NAME}/pose_info.hpp
  src/pose_info.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_pose_info
  ${PROJECT_NAME}_pose
  ${PROJECT_NAME}_tree_button
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_pose_info
  ${PROJECT_NAME}_pose
  ${PROJECT_NAME}_tree_button
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Info panel
add_library(
  ${PROJECT_NAME}_traj_info
  include/${PROJECT_NAME}/traj_info.hpp
  src/traj_info.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_traj_info
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_traj_info
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Modify trajectory
add_library(
  ${PROJECT_NAME}_modify_panel_range_list_selection
  include/${PROJECT_NAME}/modify_widgets/range_list_selection.hpp
  src/modify_widgets/range_list_selection.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify_panel_range_list_selection
  Qt5::Widgets
)

add_library(
  ${PROJECT_NAME}_modify_panel_modify_poses
  include/${PROJECT_NAME}/modify_widgets/modify_poses.hpp
  src/modify_widgets/modify_poses.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${PROJECT_NAME}_pose
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${PROJECT_NAME}_pose
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_modify_panel_blend_radius_table
  include/${PROJECT_NAME}/modify_widgets/blend_radius_table.hpp
  src/modify_widgets/blend_radius_table.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify_panel_blend_radius_table
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_modify_panel_blend_radius_table
  ${catkin_EXPORTED_TARGETS}
)

add_library(
    ${PROJECT_NAME}_modify_pose_information_parameters_filter
    include/${PROJECT_NAME}/modify_widgets/pose_information_parameters_filter.hpp
    src/modify_widgets/pose_information_parameters_filter.cpp
)
target_link_libraries(
    ${PROJECT_NAME}_modify_pose_information_parameters_filter
    Qt5::Widgets
)
add_dependencies(
    ${PROJECT_NAME}_modify_pose_information_parameters_filter
    ${catkin_EXPORTED_TARGETS}
)

add_library(
  ${PROJECT_NAME}_modify
  include/${PROJECT_NAME}/modify.hpp
  src/modify.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify
  ${PROJECT_NAME}_modify_panel_blend_radius_table
  ${PROJECT_NAME}_modify_panel_range_list_selection
  ${PROJECT_NAME}_modify_pose_information_parameters_filter
  ${PROJECT_NAME}_modify_panel_modify_poses
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_modify
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Entry exit strategies
add_library(
  ${PROJECT_NAME}_entry_exit_strategies
  include/${PROJECT_NAME}/entry_exit_strategies.hpp
  src/entry_exit_strategies.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_entry_exit_strategies
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_entry_exit_strategies
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Simulate
add_library(
  ${PROJECT_NAME}_simulate
  include/${PROJECT_NAME}/simulate.hpp
  src/simulate.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_simulate
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_simulate
  ${catkin_EXPORTED_TARGETS}
)

# Intralayers display
add_library(
  ${PROJECT_NAME}_intralayers_display
  include/${PROJECT_NAME}/ctk/ctkRangeSlider.hpp
  include/${PROJECT_NAME}/intralayers_display.hpp
  src/ctk/ctkRangeSlider.cpp
  src/ctk/ctkRangeSliderPrivate.cpp
  src/intralayers_display.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_intralayers_display
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_intralayers_display
  ${catkin_EXPORTED_TARGETS}
)

# Needed by Qt / RViz / ROS
class_loader_hide_library_symbols(
  ${PROJECT_QT_PANELS_TARGETS}
)

#############
## Install ##
#############

if("${CMAKE_INSTALL_PREFIX}" MATCHES "^/opt/")
  # Handle release case
  set(DESKTOP_APP_DIR "/usr/share/applications")
  set(DESKTOP_ICON_DIR "/usr/share/icons")
else()
  set(DESKTOP_APP_DIR "$ENV{HOME}/.local/share/applications")
  set(DESKTOP_ICON_DIR ${DESKTOP_APP_DIR})
  set(DESKTOP_APP_SUFFIX " (local)")
endif()

# Create directory for the desktop file/icon
install(DIRECTORY DESTINATION
  ${DESKTOP_APP_DIR}
)
install(DIRECTORY DESTINATION
  ${DESKTOP_ICON_DIR}
)

# Configure bash script
configure_file(
  scripts/${PROJECT_NAME}.bash.in
  scripts/${PROJECT_NAME}.bash
  @ONLY
)

# Install bash scripts
install(
  PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/scripts/${PROJECT_NAME}.bash
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Configure / install desktop launchers
if(DEFINED DESKTOP_APP_SUFFIX)
  configure_file(
    desktop/${PROJECT_NAME}.desktop.in
    desktop/${PROJECT_NAME}_local.desktop
  )
else()
  configure_file(
    desktop/${PROJECT_NAME}.desktop.in
    desktop/${PROJECT_NAME}.desktop
  )
endif()
if(DEFINED DESKTOP_APP_SUFFIX)
  install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/desktop/${PROJECT_NAME}_local.desktop
    DESTINATION
    ${DESKTOP_APP_DIR}
  )
else()
  install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/desktop/${PROJECT_NAME}.desktop
    DESTINATION
    ${DESKTOP_APP_DIR}
  )
endif()

# Install icon
install(
  FILES
  desktop/${PROJECT_NAME}.png
  DESTINATION
  ${DESKTOP_ICON_DIR}
)

## Mark executables and/or libraries for installation
# Qt panel
install(
  TARGETS
  ${PROJECT_TARGETS}
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

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  config
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  FILES
  plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
