#include <collision_testing.h>
#include <rosbag/view.h>

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";

ros::Publisher CollisionTesting::vis_marker_array_publisher_;

bool CollisionTesting::publisher_initialized = false;

void CollisionTesting::init(bool fromBag, std::string filename, std::string fixed_frame)
{
    //arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res = 0;

    arm_str[0] = "r";
    long_arm_str[0] = "right";
    arm_str[1] = "l";
    long_arm_str[1] = "left";

    //planning_scene_res = new arm_navigation_msgs::GetPlanningScene::Response ();

    if (fromBag)
    {
        rosbag::Bag bag;
        bag.open(filename, rosbag::bagmode::Read);

        rosbag::View view(bag);

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            if ((m.getTopic() == "/planning_scene_res") || (m.getTopic() == "planning_scene_res"))
            {
                std::cout << "Got the planning scene response from bagfile " << filename << std::endl;
                arm_navigation_msgs::GetPlanningSceneResponse::ConstPtr msg_in = m.instantiate<arm_navigation_msgs::GetPlanningSceneResponse>();

                //std::cout << "Planning scene msg :" << msg_in->planning_scene << std::endl;

                planning_scene = msg_in->planning_scene;

                //std::cout << "Planning scene :" << planning_scene_res.planning_scene << std::endl;

                arm_navigation_msgs::PlanningScene &ps = planning_scene;

                //std::vector<std::string> fixed_frame_names;

                double shifterx = 0;
                nh_.param<double>("shifterx", shifterx, 0);
                double shiftery = 0;
                nh_.param<double>("shiftery", shiftery, 0);
                double shifterz = 0;
                nh_.param<double>("shifterz", shifterz, 0);

                ps.robot_state.multi_dof_joint_state.poses[0].position.x += shifterx;
                ps.robot_state.multi_dof_joint_state.poses[0].position.y += shiftery;
                ps.robot_state.multi_dof_joint_state.poses[0].position.z += shifterz;

                tf::Pose tmp;

                tf::StampedTransform world_joint;
                tf::poseMsgToTF(ps.robot_state.multi_dof_joint_state.poses[0],tmp);
                world_joint.setData(tmp);
                world_joint.child_frame_id_ = ps.robot_state.multi_dof_joint_state.child_frame_ids[0];
                world_joint.frame_id_ = ps.robot_state.multi_dof_joint_state.frame_ids[0];
                world_joint.stamp_ = ros::Time(0);
                transformer_.setTransform(world_joint,"self");

                geometry_msgs::TransformStamped ts;
                tf::transformStampedTFToMsg(world_joint, ts);
                std::cout << "world joint " << ts << std::endl;

                fixed_frame_transforms.push_back(world_joint);

                fixed_frame_names.push_back(world_joint.frame_id_);
                //fixed_frame_names.push_back(world_joint.child_frame_id_);// ? brauchen wir das auch

                for (size_t i = 0; i < ps.fixed_frame_transforms.size(); ++i)
                {
                    tf::StampedTransform act;
                    tf::transformStampedMsgToTF(ps.fixed_frame_transforms[i], act);
                    act.setData(act.inverse());
                    act.stamp_ = ros::Time(0);
                    transformer_.setTransform(act,"self");

                    fixed_frame_transforms.push_back(act);

                    if (act.child_frame_id_ != fixed_frame)
                        fixed_frame_names.push_back(act.child_frame_id_);

                    tf::transformStampedTFToMsg(act, ts);
                    std::cout << "fixed joint " << ts << std::endl;
                }

                std::cout << "_______________________________________________________________________" << std::endl;

                /* // change fixed frame for planning
                tf::StampedTransform current;
                transformer_.lookupTransform(fixed_frame, world_joint.child_frame_id_, ros::Time(0), current);
                tf::transformStampedTFToMsg(current, ts);
                std::cout << "world joint after transformer " << ts << std::endl;

    //tf::poseTFToMsg(ts, ps.robot_state.multi_dof_joint_state.poses[0]);


                //ps.robot_state.multi_dof_joint_state.child_frame_ids[0] = ts.child_frame_id;
                //ps.robot_state.multi_dof_joint_state.frame_ids[0] = ts.header.frame_id;

                //ps.robot_state.multi_dof_joint_state.poses[0].position.x = ts.transform.translation.x;
                //ps.robot_state.multi_dof_joint_state.poses[0].position.y = ts.transform.translation.y;
                //ps.robot_state.multi_dof_joint_state.poses[0].position.z = ts.transform.translation.z;
                //ps.robot_state.multi_dof_joint_state.poses[0].orientation = ts.transform.rotation;

                size_t j = 0;
                for (std::vector<std::string>::iterator it=fixed_frames.begin(); it!=fixed_frames.end(); ++it)
                {
                    transformer_.lookupTransform(fixed_frame, *it, ros::Time(0), current);
                    tf::transformStampedTFToMsg(current, ts);
                    std::cout << "after transformer " << ts << std::endl;
                    ps.fixed_frame_transforms[j] = ts;
                    j++;
                }

                //ps.robot_state.joint_state.header.frame_id = "/base_footprint";

                std::cout << "Planning scene :" << ps.robot_state << std::endl;
                */


                /*

                //this is typically showing only odom_combined to base_footprint
                std::cout << "robot state" <<std::endl;
                for (std::vector<std::string>::iterator jt = planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.joint_names.begin();
                    jt !=planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.joint_names.end(); jt++)
                    {
                        std::cout << "joint name " << *jt << std::endl;
                    }

                for (std::vector<std::string>::iterator jt = planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.frame_ids.begin();
                    jt !=planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.frame_ids.end(); jt++)
                    {
                        std::cout << "frame_ids " << *jt << std::endl;
                    }

                for (std::vector<std::string>::iterator jt = planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.child_frame_ids.begin();
                    jt !=planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.child_frame_ids.end(); jt++)
                    {
                        std::cout << "child_frame_ids " << *jt << std::endl;
                    }

                std::cout << planning_scene_res.planning_scene.robot_state.multi_dof_joint_state.poses[0] << std::endl;

                //for (size_t i = 0; i < planning_scene_res.planning_scene.fixed_frame_transforms.size(); ++i)
                  //  std::cout << "fixed frame " << planning_scene_res.planning_scene.fixed_frame_transforms[i] << std::endl;*/
            }
        }
    }
    else
    {
        ros::ServiceClient get_planning_scene_client =
        nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);


        ROS_INFO("Waiting for planning scene service to come up..");
        ros::service::waitForService(GET_PLANNING_SCENE_NAME);
        ROS_INFO("                                        is up.");

        arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
        arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

        if (!get_planning_scene_client.call(planning_scene_req, planning_scene_res))
            ROS_ERROR("Could not get planning scene");
        else
            planning_scene = planning_scene_res.planning_scene;
    }

    //planning_scene_res.planning_scene.collision_map.header.frame_id = "/torso_lift_link";

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

    //kinematic_state = collision_models->setPlanningScene(planning_scene_res.planning_scene);

    if (!publisher_initialized)
    {
        vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("good_state_validity_markers_array", 128, true);
        publisher_initialized = true;
    }
}

void CollisionTesting::setCollisionFrame(std::string frame_id)
{
    planning_scene.collision_map.header.frame_id = frame_id;
}

void CollisionTesting::resetPointCloud()
{
    planning_scene.collision_map.boxes.clear();
}

template <class T>
void CollisionTesting::setPointCloud(const T &cloud, double pointSize)
{
    resetPointCloud();
    addPointCloud(cloud, pointSize);
}

void CollisionTesting::updateCollisionModel()
{
    std::cout << "void CollisionTesting::updateCollisionModel()" << std::endl;
    if (kinematic_state != 0)
        collision_models->revertPlanningScene(kinematic_state);

    kinematic_state = collision_models->setPlanningScene(planning_scene);
    if (kinematic_state == 0)
        ROS_ERROR("KINEMATIC STATE WAS NOT RETURNED BY SETPLANNINGSCENE");

    std::cout << "Collision frame " <<  planning_scene.collision_map.header.frame_id << std::endl;
    std::cout << "Collision map size" <<  planning_scene.collision_map.boxes.size() << std::endl;
}

template <class T>
void CollisionTesting::addPointCloud(const T &cloud, double pointSize, tf::Transform *relative_transform)
{

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        tf::Vector3 tf_pt(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
        tf::Transform trans;
        trans.setOrigin(tf_pt);
        trans.setRotation(tf::Quaternion(0,0,0,1));

        if (relative_transform)
        {
            trans = *relative_transform * trans;
            tf_pt = trans.getOrigin();
        }

        arm_navigation_msgs::OrientedBoundingBox box;
        box.extents.x = box.extents.y = box.extents.z = pointSize;

        box.axis.x = box.axis.y = 0;
        box.axis.z = 1;

        box.angle = 0;

        //geometry_msgs::Point center; // point32 in msg vs. point accepted by tf

        //tf::pointTFToMsg(tf_pt, center);

        //box.center.x = cloud->points[i].x;
        //box.center.y = cloud->points[i].y;
        //box.center.z = cloud->points[i].z;

        box.center.x = tf_pt.x();
        box.center.y = tf_pt.y();
        box.center.z = tf_pt.z();

        planning_scene.collision_map.boxes.push_back(box);
    }

}

template
void CollisionTesting::addPointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >  &cloud, double pointSize, tf::Transform *relative_transform);

template
void CollisionTesting::addPointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >  &cloud, double pointSize, tf::Transform *relative_transform);

bool CollisionTesting::inCollision(int arm, double jointState[])
{

    //std::cout << "incollision" << std::endl;

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

    ROS_INFO("Publishin collision markers? %s", (publish_markers ? "true" : "false") );

    if (publish_markers)
    {
        visualization_msgs::MarkerArray sum_arr;

        std_msgs::ColorRGBA color;

        color.r = collision ? 1 : 0;
        color.g = collision ? 0 : 1;
        color.b = 0;
        color.a = .5;

        char name[255];
        int id = 0;
        sprintf(name,"arm%i",id);
        //for (int k = 0; k < 2; k++)
        collision_models->getRobotMarkersGivenState(*kinematic_state,
                sum_arr,
                color,
                (arm == 0) ? "right_arm" : "left_arm",
                ros::Duration(1000),
                &arm_names[0]);

        color.r = 1;
        color.g = 1;
        color.b = 0;

        collision_models->getAllCollisionPointMarkers(*kinematic_state,
                sum_arr,
                color,
                ros::Duration(100));

        ROS_INFO("Publishin collision markers %zu", sum_arr.markers.size());

        vis_marker_array_publisher_.publish(sum_arr);

        //!TODO remove sleep for speed ;)
        ros::Duration(0.5).sleep();
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
