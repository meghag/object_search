
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

    explicit CollisionTesting(ros::NodeHandle &nh) :
        nh_(nh) {  }

    void init();

    // change the frame the pointcloud is provided in from the standard torso_lift_link to something else
    void setCollisionFrame(std::string frame_id);

    // set pointcloud to collision environment, replacing previous data
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize = 0.01);

    // add pointcloud to collision environment
    void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize = 0.01);

    // has to be called after adding pointcloud
    void updateCollisionModel();

    // reset collision environment to not carry any point obstacles
    void resetPointCloud();

    //check a given joint state for an arm for collision
    bool inCollision(int arm, double jointState[]);

    bool inCollision(int arm, std::vector<double> jointState);

    arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res;

    planning_models::KinematicState* kinematic_state;

    planning_environment::CollisionModels *collision_models;

    ros::NodeHandle &nh_;

    std::string arm_str[2], long_arm_str[2];

    std::vector<std::string> arm_names[2];

    std::map<std::string, double> nvalues;

    bool publish_markers;

    static ros::Publisher vis_marker_array_publisher_;

    static bool publisher_initialized;
};


#endif
