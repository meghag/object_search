
#ifndef __COLLISION_TESTING__H__
#define __COLLISION_TESTING__H__

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <rosbag/bag.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class CollisionTesting
{
public :

    explicit CollisionTesting(ros::NodeHandle &nh)
        {
            nh_ = &nh;
            kinematic_state = 0L;
        }

    void init(bool fromBag = false, std::string filename = "", std::string fixed_frame_ = "");

    // change the frame the pointcloud is provided in from the standard torso_lift_link to something else
    void setCollisionFrame(std::string frame_id);

    // set pointcloud to collision environment, replacing previous data
    template <class T>
    void setPointCloud(const T &cloud, double pointSize = 0.01);

    // add pointcloud to collision environment
    template <class T>
    void addPointCloud(const T &cloud, double pointSize = 0.01, tf::Transform *relative_transform = 0L);

    // has to be called after adding pointcloud
    void updateCollisionModel();

    // reset collision environment to not carry any point obstacles
    void resetPointCloud();

    //check a given joint state for an arm for collision
    bool inCollision(int arm, double jointState[]);

    bool inCollision(int arm, std::vector<double> jointState);

    bool inCollision(int arm, tf::Pose pose_in_ik_frame);

    arm_navigation_msgs::PlanningScene planning_scene;

    planning_models::KinematicState* kinematic_state;

    planning_environment::CollisionModels *collision_models;

    std::string arm_str[2], long_arm_str[2];

    std::vector<std::string> arm_names[2];

    std::map<std::string, double> nvalues;

    bool publish_markers;

    static ros::Publisher vis_marker_array_publisher_;

    static bool publisher_initialized;

    static ros::NodeHandle *nh_;

    static bool static_planning_scene_initialized;

    static arm_navigation_msgs::PlanningScene static_planning_scene;

    // offset in robot base coordinate system, apply negative motion of object to check for
    void push_offset(tf::Vector3 offset);

    void pop_offset();

    std::vector<geometry_msgs::Point> offset_stack;

    //tf::Transformer transformer_;

    std::vector<std::string> fixed_frame_names;
    std::vector<tf::StampedTransform> fixed_frame_transforms;
};


#endif
