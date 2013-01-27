# Message used for asking the planner to plan a sequence of moves

#The config cloud containing all objects as seen
sensor_msgs/PointCloud2 object_cloud

# The target object cloud
#sensor_msgs/PointCloud2 target_cloud

float64 table_height

#Bounding box for objects
float64[] bb_min
float64[] bb_max