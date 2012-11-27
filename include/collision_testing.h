
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

    // set pointcloud to collision environment, replacing previous data
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize = 0.01);

    // add pointcloud to collision environment
    void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize = 0.01);

    // reset collision environment to not carry any point obstacles
    void resetPointCloud();

    //check a given joint state for an arm for collision
    bool inCollision(int arm, double jointState[]);

    arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res;

    planning_models::KinematicState* kinematic_state;

    planning_environment::CollisionModels *collision_models;

    ros::NodeHandle &nh_;

    std::string arm_str[2], long_arm_str[2];

    std::vector<std::string> arm_names[2];

    std::map<std::string, double> nvalues;
};


#endif
