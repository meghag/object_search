#include <collision_testing.h>

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";

ros::Publisher CollisionTesting::vis_marker_array_publisher_;

bool CollisionTesting::publisher_initialized = false;

void CollisionTesting::init()
{
    //arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res = 0;

    arm_str[0] = "r";
    long_arm_str[0] = "right";
    arm_str[1] = "l";
    long_arm_str[1] = "left";

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

    collision_models = new planning_environment::CollisionModels("robot_description");

    collision_models->revertAllowedCollisionToDefault();
    //collision_models->disableCollisionsForNonUpdatedLinks(long_arm_str[k] + "_arm");
    //collision_models->disableCollisionsForNonUpdatedLinks("right_arm");

    for (int k = 0; k < 2; k++)
    {
        std::vector<std::string> temp = collision_models->getKinematicModel()->getModelGroup(long_arm_str[k] + "_arm")->getUpdatedLinkModelNames();
        arm_names[k].insert(arm_names[k].begin(), temp.begin(), temp.end());
    }

    //kinematic_state = collision_models->setPlanningScene(planning_scene_res->planning_scene);

    if (!publisher_initialized)
    {
        vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("good_state_validity_markers_array", 128, true);
        publisher_initialized = true;
    }
}

void CollisionTesting::setCollisionFrame(std::string frame_id)
{
    planning_scene_res->planning_scene.collision_map.header.frame_id = frame_id;
}

void CollisionTesting::resetPointCloud()
{
    planning_scene_res->planning_scene.collision_map.boxes.clear();
}


void CollisionTesting::setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize)
{
    resetPointCloud();
    addPointCloud(cloud, pointSize);
}

void CollisionTesting::updateCollisionModel()
{
    kinematic_state =   collision_models->setPlanningScene(planning_scene_res->planning_scene);
}


void CollisionTesting::addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double pointSize)
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

bool CollisionTesting::inCollision(int arm, double jointState[])
{

    nvalues[arm_str[arm] + "_shoulder_pan_joint"] = jointState[0];
    nvalues[arm_str[arm] + "_shoulder_lift_joint"] = jointState[1];
    nvalues[arm_str[arm] + "_upper_arm_roll_joint"] =jointState[2];
    nvalues[arm_str[arm] + "_elbow_flex_joint"] = jointState[3];
    nvalues[arm_str[arm] + "_forearm_roll_joint"] = jointState[4];
    nvalues[arm_str[arm] + "_wrist_flex_joint"] = jointState[5];
    nvalues[arm_str[arm] + "_wrist_roll_joint"] = jointState[6];
    kinematic_state->setKinematicState(nvalues);

    //kinematic_state->getJointState("world_joint")->setJointStateValues(root);

    kinematic_state->updateKinematicLinks();

    if (arm == 0)
        collision_models->disableCollisionsForNonUpdatedLinks("right_arm");
    else
        collision_models->disableCollisionsForNonUpdatedLinks("left_arm");

    bool collision = collision_models->isKinematicStateInCollision(*kinematic_state);

    if (publish_markers)
    {
        visualization_msgs::MarkerArray sum_arr;

        std_msgs::ColorRGBA good_color;

        good_color.r = collision ? 1 : 0;
        good_color.g = collision ? 0 : 1;
        good_color.b = 0;
        good_color.a = .5;

        char name[255];
        int id = 0;
        sprintf(name,"arm%i",id);
        //for (int k = 0; k < 2; k++)
        collision_models->getRobotMarkersGivenState(*kinematic_state,
                sum_arr,
                good_color,
                (arm == 0) ? "right_arm" : "left_arm",
                ros::Duration(100),
                &arm_names[0]);

        good_color.r = 0;
        good_color.g = 0;
        good_color.b = 1;

        //collision_models->getAllCollisionPointMarkers(*kinematic_state,
        //      sum_arr,
        //    good_color,
        //  ros::Duration(100));

        vis_marker_array_publisher_.publish(sum_arr);
    }

    return collision;

}

bool CollisionTesting::inCollision(int arm, std::vector<double> jointState)
{
    double jointStateArr[7];
    for (size_t i = 0; i < 7; ++i)
        jointStateArr[i] = jointState[i];
    return inCollision(arm,jointStateArr);
}
