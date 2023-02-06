cmake_minimum_required(VERSION 3.2)
project(yaml_generators)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(VTK 8.0.0 REQUIRED)

include(${VTK_USE_FILE})

find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)

# Library to load YAML files into VTK data structures
include_directories(
 include
 ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  ram_trajectory_files_manager
  src/trajectory_files_manager_1.cpp)

target_link_libraries(
  ram_trajectory_files_manager
  ${YAML_CPP_LIBRARIES}
  ${VTK_LIBRARIES}
)

# Display a YAML file with VTK
add_executable(
  display_yaml
  display_yaml.cpp
)
target_link_libraries(
  display_yaml
  ram_trajectory_files_manager
  ${VTK_LIBRARIES}
)

# Resize a YAML file with VTK
add_executable(
  resize_yaml
  resize_yaml.cpp
)
target_link_libraries(
  resize_yaml
  ram_trajectory_files_manager
  ${VTK_LIBRARIES}
)

# Read CSV file and write YAML file (no VTK dependency)
add_executable(
  read_csv_write_yaml
  read_csv_write_yaml.cpp
)
target_link_libraries(
  read_csv_write_yaml
  ram_trajectory_files_manager
)

# Generate YAML files (no VTK dependency)
add_executable(
  circle
  circle.cpp
)
target_link_libraries(
  circle
  ram_trajectory_files_manager
)

add_executable(
  star
  star.cpp
)
target_link_libraries(
  star
  ram_trajectory_files_manager
)

add_executable(
  round_rectangle
  round_rectangle.cpp
)
target_link_libraries(
  round_rectangle
  ram_trajectory_files_manager
)

add_executable(
  wall
  wall.cpp
)
target_link_libraries(
  wall
  ram_trajectory_files_manager
)
