# 0 = 'Joint'
# 1 = 'Linear'
uint32 movement_type

# 0 = 'Stop/go'
# 1 = 'Blend radius'
uint32 approach_type

# Trajectory smoothing by the robot
# Only used if approach_type = 1
float64 blend_radius

# meters/second
float64 printing_move_speed
float64 non_printing_move_speed

# Watts
float64 laser_power

# User defined unit
# Institut Maupertuis: Wire - meters/second
float64 feed_rate

