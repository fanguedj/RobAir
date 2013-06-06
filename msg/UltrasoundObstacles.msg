# True for dangerous obstacle in front of (resp. behind) the robot
bool front_obstacle
bool rear_obstacle
# Distance of obstacles in centimeters
# Order of sensors must match the Arduino sketch
# North = front
uint32 north_west
uint32 north_left
uint32 north_right
uint32 north_east
uint32 south_east
uint32 south_right
uint32 south_left
uint32 south_west
