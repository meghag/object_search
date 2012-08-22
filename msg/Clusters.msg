# Message used for processing clusters/objects

# Number of clusters (visible objects) identified
# by the object segmentation algorithm
int32 num_clusters

# The point clouds corresponding to the visible objects
sensor_msgs/PointCloud2[] clusters

# The whole object cloud
sensor_msgs/PointCloud2 object_cloud
