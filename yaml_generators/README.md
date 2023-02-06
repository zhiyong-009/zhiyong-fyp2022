# YAML generators
This directory contains simple C++ programs (not ROS packages) to generate/modify/visualize YAML files.

## Set-up
Copy these instructions in a terminal:
```bash
mkdir -p $HOME/code/yaml_generators/build_release
cd $HOME/code/yaml_generators
ln -s $HOME/code/catkin_workspace/src/ros_additive_manufacturing/yaml_generators src
cd build_release
cmake ../src
make -j4
```

## Listing the programs
To generate a YAML file you need to call a progam, to list all available programs do:
```bash
cd $HOME/code/yaml_generators/build_release
ls
```
Files listed in green are programs.

## Running a program
In this example we will run the `star` program:
```bash
cd $HOME/code/yaml_generators/build_release
./star
```

# Programs documentation
:warning: All distance units are in meters!

## read_csv_write_yaml
```bash
./read_csv_write_yaml
Usage: ./read_csv_write_yaml csv_file  yaml_file
```

- `csv_file` The input CSV file containing X, Y, Z (millimeters) on each line
- `yaml_file` The output YAML file that will be written (distances in meters)

## circle
```bash
./circle
Usage: ./circle radius number_of_points
```

- `radius`: The radius of the generated circle (in meters)
- `number_of_points`: The number of points constituting the circle

Example usage:
```bash
./circle 0.1 10
```

Will generate a circle of 10 points with a radius of 0.1 meters.

## display_yaml
This program allows to visualize a YAML file.
```bash
./display_yaml
Usage: ./display_yaml yaml_file
```

Example usage:
```bash
./display_yaml circle.yaml
Radius is 0.1 and number of points is 10
File circle.yaml has been written
```

## resize_yaml
This program allows to resize the points inside a YAML file.
```bash
./resize_yaml
Usage: ./resize_yaml yaml_file resize_factor
```

- `yaml_file`: The YAML file to be resized
- `resize_factor`: The scaling factor (10 = ten times bigger)

Example usage:
```bash
./resize_yaml circle.yaml 10
```

## round_rectangle
```bash
./round_rectangle
Usage: ./round_rectangle radius points_in_circle_arc length witdh
```

- `radius`: The radius of the rounding of the rectangle
- `points_in_circle_arc`: The number of points in the rounding of the rectangle (per corner)
- `length`: Length of the rectangle
- `witdh`: Width of the rectangle

Example usage:
```bash
./round_rectangle 0.02 6 0.1 0.06
Radius is 0.02
Points in circle arc is 6
Length is 0.1
Width is 0.06
File round_rectangle.yaml has been written
```

## star
```bash
./star
Usage: ./star inner_radius outer_radius number_of_branches
```

- `inner_radius`: The inner radius of the star
- `outer_radius`: The outer radius of the star
- `number_of_branches`: The number of branches

Example usage:
```bash
./star 0.04 0.08 6
Inner radius is 0.04 and outer radius is 0.08
Number of branches is 6
File star.yaml has been written
```

## wall
```bash
./wall
Usage: ./wall deposit_width number_of_zigzags length angle number_of_layers layer_height invert
```

- `deposit_width`: The width of the deposited material in millimeters
- `number_of_zigzags`: How thick the wall will be
- `length`: Length of the wall in millimeters
- `angle`: Angle of deposition (in degrees), 0 means vertical
- `number_of_layers`: Number of layers to generate
- `layer_height`: Height between layers in millimeters
- `invert`: 0, 1 or 2.
  - `0` = All layers are the same
  - `1` = Odd layer numbers will have their trajectory inverted. Last pose is first pose and first pose is last pose.
  - `2` = Odd layer numbers will have their trajectory modified by a symmetry arround the  Y axis

Example usage:
```bash
./wall 1.3 4 25 10 30 1.2 0
Deposit width is 0.0013 meters
Number of zigzags is 4
Length is 0.025 mm
Angle is 0.174533 rad
Number of layers is 30
Layer height is 0.0012 mm
Invert mode is 0
File wall.yaml has been written
```
