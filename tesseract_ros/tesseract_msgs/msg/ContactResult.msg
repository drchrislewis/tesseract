time stamp
float64 distance
byte[2] type_id # ROBOT_LINK = 0, ROBOT_ATTACHED = 1
string[2] link_names
uint64[2] shape_id # Link's child shape index
uint64[2] subshape_id # If mesh or octomap this will be the voxel/triangle index
geometry_msgs/Vector3[2] nearest_points
geometry_msgs/Vector3 normal
geometry_msgs/Vector3[2] cc_nearest_points
float64 cc_time
byte cc_type #CCType_None = 0, CCType_Time0 = 1, CCType_Time1 = 2, CCType_Between = 3
