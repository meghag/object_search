#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <rosbag/bag.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";


class CollisionTesting
{
public :

    CollisionTesting(ros::NodeHandle &nh) :
        nh_(nh) {  }

    void init();

    void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize = 0.01);

    void resetPointCloud();

    bool inCollision(double jointState[]);

    arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res;
    ros::NodeHandle &nh_;
};

void CollisionTesting::init()
{
    //arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res = 0;

    planning_scene_res = new arm_navigation_msgs::GetPlanningScene::Response ();

    ros::ServiceClient get_planning_scene_client =
        nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);

    ROS_INFO("Waiting for planning scene service to come up..");
    ros::service::waitForService(GET_PLANNING_SCENE_NAME);
    ROS_INFO("                                        is up.");


    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;

    if (!get_planning_scene_client.call(planning_scene_req, *planning_scene_res))
        ROS_ERROR("Could not get planning scene");


    planning_scene_res->planning_scene.collision_map.header.frame_id = "/torso_lift_link";


    resetPointCloud();

}

void CollisionTesting::resetPointCloud()
{
    planning_scene_res->planning_scene.collision_map.boxes.clear();
}


void CollisionTesting::setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize)
{


    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        //tf::Point tf_pt(,cloud->points[i].y,cloud->points[i].z);

        arm_navigation_msgs::OrientedBoundingBox box;

        box.extents.x = box.extents.y = box.extents.z = pointSize;

        box.axis.x = box.axis.y = 0;
        box.axis.z = 1;

        box.angle = 0;

        //geometry_msgs::Point center; // point32 in msg vs. point accepted by tf

        //tf::pointTFToMsg(tf_pt, center);

        box.center.x = cloud->points[i].x;
        box.center.y = cloud->points[i].y;
        box.center.z = cloud->points[i].z;

        planning_scene_res->planning_scene.collision_map.boxes.push_back(box);
    }
}

