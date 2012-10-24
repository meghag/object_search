#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/GetPlanningScene.h>

ros::Publisher vis_marker_array_publisher_good_; //= rh.advertise<visualization_msgs::MarkerArray>("good_state_validity_markers_array", 128, true);

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";

int main (int argc, char **argv)
{

    ros::init(argc, argv, "test_find_graspables");

    ros::NodeHandle nh_;

    arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res = 0;
    planning_scene_res = new arm_navigation_msgs::GetPlanningScene::Response ();

    //get planning scene response
    {
        ros::ServiceClient get_planning_scene_client =
            nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);

        ROS_INFO("Waiting for planning scene service to come up..");
        ros::service::waitForService(GET_PLANNING_SCENE_NAME);
        ROS_INFO("                                        is up.");


        arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;

        if (!get_planning_scene_client.call(planning_scene_req, *planning_scene_res))
            ROS_ERROR("Could not get planning scene");



    }

    planning_environment::CollisionModels *collision_models = 0;

    planning_models::KinematicState* kinematic_state;


    vis_marker_array_publisher_good_ = nh_.advertise<visualization_msgs::MarkerArray>("good_state_validity_markers_array", 128, true);


    std::string arm_str[2], long_arm_str[2];
    arm_str[0] = "r";
    long_arm_str[0] = "right";
    arm_str[1] = "l";
    long_arm_str[1] = "left";
    std::vector<std::string> arm_names[2];

    {

        double jointState[7] = {-1.598047009379525, 0.16332504343852233, -1.209332656362964, -2.110992174861759, 93.14866355338992, -1.5068208800215774, -73.5743854835612};


        collision_models = new planning_environment::CollisionModels("robot_description");

        collision_models->revertAllowedCollisionToDefault();
        //collision_models->disableCollisionsForNonUpdatedLinks(long_arm_str[k] + "_arm");
        collision_models->disableCollisionsForNonUpdatedLinks("right_arm");

        {

            planning_scene_res->planning_scene.collision_map.header.frame_id = "/torso_lift_link";

            arm_navigation_msgs::OrientedBoundingBox box;
            geometry_msgs::Point32 extents;
            //extents.x = extents.y = extents.z = .0125;
            extents.x = extents.y = extents.z = 1.25;
            box.extents = extents;
            geometry_msgs::Point32 axis;
            axis.x = axis.y = 0;
            axis.z = 1;
            box.axis = axis;
            box.angle = 0;

            geometry_msgs::Point32 pt;
            pt.x = atof(argv[1]);
            pt.y = atof(argv[2]);
            pt.z = atof(argv[3]);

            box.center = pt;

            planning_scene_res->planning_scene.collision_map.boxes.push_back(box);
        }

        kinematic_state =   collision_models->setPlanningScene(planning_scene_res->planning_scene);


        tf::Transform root = kinematic_state->getRootTransform();
        std::map<std::string, double> nvalues;
        int k = 0;

        //for (int j=0; j < 7; j++)
          //  jointState[j] = 0;

        nvalues[arm_str[k] + "_shoulder_pan_joint"] = jointState[0];
        nvalues[arm_str[k] + "_shoulder_lift_joint"] = jointState[1];
        nvalues[arm_str[k] + "_upper_arm_roll_joint"] =jointState[2];
        nvalues[arm_str[k] + "_elbow_flex_joint"] = jointState[3];
        nvalues[arm_str[k] + "_forearm_roll_joint"] = jointState[4];
        nvalues[arm_str[k] + "_wrist_flex_joint"] = jointState[5];
        nvalues[arm_str[k] + "_wrist_roll_joint"] = jointState[6];
        kinematic_state->setKinematicState(nvalues);

        //kinematic_state->getJointState("world_joint")->setJointStateValues(root);

        kinematic_state->updateKinematicLinks();

        collision_models->disableCollisionsForNonUpdatedLinks("right_arm");



        bool collision = collision_models->isKinematicStateInCollision(*kinematic_state);

        if (collision)
            ROS_INFO("COLLISION");
        else
            ROS_INFO("NO COLLISION");

        if (0)
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
                    "right_arm", //"arms", //long_arm_str[k] + "_arm",
                    ros::Duration(1),
                    &arm_names[0]);

            good_color.r = 0;
            good_color.g = 0;
            good_color.b = 1;

            collision_models->getAllCollisionPointMarkers(*kinematic_state,
                    sum_arr,
                    good_color,
                    ros::Duration(1));

            vis_marker_array_publisher_good_.publish(sum_arr);
        }

    }

}
