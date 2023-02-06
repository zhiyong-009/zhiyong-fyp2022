## FILLED BY PATH_PLANNING

# A universally unique identifier (UUID)
uuid_msgs/UniqueID unique_id

geometry_msgs/Pose pose

# "Z" level (starts at zero)
uint64 layer_level

# Layer index (starts at zero)
uint64 layer_index

# Is the pose the start of a polygon?
bool polygon_start

# Is the pose the end of a polygon?
bool polygon_end

# Is the pose in the entry strategy?
bool entry_pose

# Is the pose in the exit strategy?
bool exit_pose

## FILLED BY OTHER NODES LATER

AdditiveManufacturingParams params
