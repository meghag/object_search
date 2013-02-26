#Represents a grasping or pushing approach:
#A set of boxes that have to include points and a set of boxes
#that must not include points represent how the gripper should be
#positioned in relation to an object that is to be pushed or grasped.
string name

#include points yes=true
std_msgs/Bool[] include_points

#min and max points of box relative to gripper wrist
geometry_msgs/Point[] min
geometry_msgs/Point[] max

#approach poses, 0 = to touch, 1 = push, relative to gripper wrist
geometry_msgs/Pose[] approach
