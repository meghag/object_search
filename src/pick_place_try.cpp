/*********************************************
This program reads in a point cloud from the specified pcd file
and extracts all cylinders out of it iteratively. Not working well.
Only the first cylinder is created correctly.

Date created: July 31, 2012
Author: Megha Gupta
Modified: Aug 14, 2012

***********************************************/


#include <ros/ros.h>
//#include <pcl/ros/conversions.h>
//#include <pcl_ros/transforms.h>
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <arm_navigation_msgs/GetCollisionObjects.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <gripper.h>

#include "tum_os/Clusters.h"

//set service and action names
const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
const std::string COLLISION_PROCESSING_SERVICE_NAME = "/octomap_server";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_COLLISION_OBJECTS_NAME = "/environment_server/get_collision_objects";
const std::string DEFAULT_CLUSTER_GRASP_PLANNER = "/plan_point_cluster_grasp";
const std::string DEFAULT_DATABASE_GRASP_PLANNER = "/database_grasp_planning";


/*
LEFT_RESET.x = 0.6;
LEFT_RESET.y = 0.5;
LEFT_RESET.z = 0;
RIGHT_RESET.x = 0.6;
RIGHT_RESET.y = -0.5;
RIGHT_RESET.z = 0;
*/

typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> PickupClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> PlaceClient;

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;
//typedef actionlib::SimpleActionClient<

//TODO: Need to find a way to make these private, not global and initialize them.
//actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickup_client_(PICKUP_ACTION_NAME, true);
PickupClient* pickup_client_; //(PICKUP_ACTION_NAME, true);
PlaceClient* place_client_; //(PLACE_ACTION_NAME, true);

class PickPlace
{

public:
    PickPlace(ros::NodeHandle & n): n_(n), collision_object_current_id_(0) {
        //pickup_client_(PICKUP_ACTION_NAME, true);
        //place_client_(PLACE_ACTION_NAME, true);

        //wait for pickup server
        while(!pickup_client_->waitForServer(ros::Duration(2.0)) && n_.ok())
            ROS_INFO_STREAM("Waiting for pickup server " << PICKUP_ACTION_NAME);
        if (!n_.ok()) exit(0);

        //wait for place server
        while(!place_client_->waitForServer(ros::Duration(2.0)) && n_.ok())
            ROS_INFO_STREAM("Waiting for place server " << PLACE_ACTION_NAME);
        if (!n_.ok()) exit(0);


        LEFT_RESET.x = 0.3;
        LEFT_RESET.y = 0.5;
        LEFT_RESET.z = 1.0;
        RIGHT_RESET.x = 0.3;
        RIGHT_RESET.y = -0.5;
        RIGHT_RESET.z = 1.0;

        active_arm_ = "left_arm";

        //wait for cluster grasp planner
        /*   while(!place_client.waitForServer(ros::Duration(2.0)) && n_.ok())
          ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
        if (!n_.ok()) exit(0);
        */

        collision_map_pub_ = n_.advertise<arm_navigation_msgs::CollisionMap>("my_collision_map",1);

        target_object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("target_object",1);

        collision_object_pub_ = n_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
        attached_object_pub_ = n_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);

        ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
        get_planning_scene_client_ = n_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);
        get_collision_objects_client_ = n_.serviceClient<arm_navigation_msgs::GetCollisionObjects>(GET_COLLISION_OBJECTS_NAME);

        bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
        clusters_sub_ = n_.subscribe("/clusters", 1, &PickPlace::clusters_callback, this);
    }

private:
    ros::NodeHandle n_;

    //create service and action clients
    ros::ServiceClient get_planning_scene_client_;
    ros::ServiceClient get_collision_objects_client_;
    ros::ServiceClient bbx_client_;
    //PickupClient pickup_client_;
    //PlaceClient  place_client_;

    //Create Publishers & Subscribers
    ros::Subscriber clusters_sub_;
    ros::Publisher collision_map_pub_;
    ros::Publisher target_object_pub_;
    ros::Publisher collision_object_pub_;
    ros::Publisher attached_object_pub_;

    // Other private variables
    std::vector<sensor_msgs::PointCloud2> clusters_;
    int collision_object_current_id_;
    object_manipulation_msgs::GraspableObject target_graspable_object_;
    std::string target_collision_name_;
    object_manipulation_msgs::PickupResult pickup_result_;
    geometry_msgs::PoseStamped pickup_location_;

    geometry_msgs::Point LEFT_RESET;
    geometry_msgs::Point RIGHT_RESET;
    std::string active_arm_;

    //Gripper gripper_;

    void clusters_callback(const tum_os::Clusters::ConstPtr& c)
    {
        ROS_INFO("Inside callback. Received clusters.");

        clusters_ = c->clusters;
        // clusters_[3].header.frame_id = "base_link";
        // clusters_[3].header.stamp = ros::Time::now();
        // target_object_pub_.publish(clusters_[3]);

        // 1. Remove all collision objects
        ROS_INFO("Removing all collision objects.");
        resetCollisionModels();

        for (size_t i = 0; i < clusters_.size(); i++) {
            clusters_[i].header.frame_id = "base_link";
            clusters_[i].header.stamp = ros::Time::now();

            // 2. Get bounding box of this cluster
            ROS_INFO("Finding bounding box of cluster %zu", i+1);
            object_manipulation_msgs::ClusterBoundingBox bbx;
            getClusterBoundingBox(clusters_[i], bbx.pose_stamped, bbx.dimensions);
            pickup_location_ = bbx.pose_stamped;

            // 3. Add this bounding box as a collision object
            ROS_INFO("Adding a collision object for cluster %zu", i+1);
            std::string collision_name;
            processCollisionGeometryForBoundingBox(bbx, collision_name);

            // 4. Create Graspable object for the target object
            //object_manipulation_msgs::GraspableObject go;
            ROS_INFO("Creating graspable object from cluster %zu", i+1);
            create_graspable_object(clusters_[i], target_collision_name_, target_graspable_object_);
            ROS_INFO("Created graspable object %zu", i+1);
            target_object_pub_.publish(clusters_[i]);

            // 5. Pick up
            ROS_INFO("Calling pickup for cluster %zu", i+1);
            if (pickup() == 0)
                // 6. Place
                place();

            ros::Duration(1.0).sleep();

        }
        move_arm(LEFT_RESET, "left_arm", 0);
        move_arm(RIGHT_RESET, "right_arm", 1);

        ROS_INFO("Exiting callback");
    }

    void resetCollisionModels() {
        arm_navigation_msgs::CollisionObject reset_object;
        reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
        reset_object.header.frame_id = "base_link";
        reset_object.header.stamp = ros::Time::now();
        reset_object.id = "all";
        collision_object_pub_.publish(reset_object);
        //collision_object_current_id_ = 0;
    }

    void getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
                               geometry_msgs::PoseStamped &pose_stamped,
                               geometry_msgs::Vector3 &dimensions) {
        object_manipulation_msgs::FindClusterBoundingBox2 srv;
        srv.request.cluster = cluster;
        if (!bbx_client_.call(srv.request, srv.response)) {
            ROS_ERROR("Failed to call cluster bounding box client");
            //throw CollisionMapException("Failed to call cluster bounding box client");
        }
        pose_stamped = srv.response.pose;
        dimensions = srv.response.box_dims;
        if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0) {
            ROS_ERROR("Cluster bounding box client returned an error (0.0 bounding box)");
            //throw CollisionMapException("Bounding box computation failed");
        }
    }

    void processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box,
            std::string &collision_name) {
        ROS_INFO("Adding bounding box with dimensions %f %f %f to collision map",
                 box.dimensions.x, box.dimensions.y, box.dimensions.z);

        arm_navigation_msgs::CollisionObject collision_object;
        collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

        collision_name = getNextObjectName();
        collision_object.id = collision_name;
        target_collision_name_ = collision_name;

        collision_object.header.frame_id = box.pose_stamped.header.frame_id;
        collision_object.header.stamp = ros::Time::now();

        arm_navigation_msgs::Shape shape;
        shape.type = arm_navigation_msgs::Shape::BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = box.dimensions.x;
        shape.dimensions[1] = box.dimensions.y;
        shape.dimensions[2] = box.dimensions.z;
        collision_object.shapes.push_back(shape);
        collision_object.poses.push_back(box.pose_stamped.pose);

        collision_object_pub_.publish(collision_object);
    }

    std::string getNextObjectName() {
        std::ostringstream iss;
        iss << collision_object_current_id_++;
        if (collision_object_current_id_ > 10000) collision_object_current_id_ = 0;
        return "object_" + iss.str();
    }

    void create_graspable_object(sensor_msgs::PointCloud2 cloud2, std::string collision_name,
                                 object_manipulation_msgs::GraspableObject& go) {
        ROS_INFO("Creating graspable object");
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud) == true) {
            cloud.header.frame_id = "base_link";
            cloud.header.stamp = ros::Time::now();
            go.cluster = cloud;
            go.collision_name = collision_name;
            go.reference_frame_id = "base_link";
        }
    }

    bool pickup() {
        //call object pickup
        ROS_INFO("Calling the pickup action");
        object_manipulation_msgs::PickupGoal pickup_goal;
        //pass one of the graspable objects returned
        //by the collision map processor
        pickup_goal.target = target_graspable_object_;
        //pass the name that the object has in the collision environment
        //this name was also returned by the collision map processor
        pickup_goal.collision_object_name = target_collision_name_;

        //pass the collision name of the table, also returned by the collision map processor
        //pickup_goal.collision_support_surface_name = "table";

        //we will be lifting the object along the "vertical" direction
        //which is along the z axis in the base_link frame
        geometry_msgs::Vector3Stamped direction;
        direction.header.stamp = ros::Time::now();
        direction.header.frame_id = "base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = 1;
        pickup_goal.lift.direction = direction;
        //request a vertical lift of 10cm after grasping the object
        pickup_goal.lift.desired_distance = 0.05;
        pickup_goal.lift.min_distance = 0.02;
        //do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = false;
        pickup_goal.use_reactive_execution = false;

        // Which arm to use
        pickup_goal.arm_name = "left_arm";
        active_arm_ = "left_arm";
        ROS_INFO("Trying pickup with left arm");

        while (1) {
            //send the goal
            pickup_client_->sendGoal(pickup_goal);
            while (!pickup_client_->waitForResult(ros::Duration(10.0))) {
                ROS_INFO("Waiting for the pickup action...");
            }
            //object_manipulation_msgs::PickupResult pickup_result =
            pickup_result_ = *(pickup_client_->getResult());
            if (pickup_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("The pickup action has failed with result code %d",
                          pickup_result_.manipulation_result.value);
                geometry_msgs::Point reset_posn;
                if (pickup_goal.arm_name == "left_arm") {
                  //reset_posn = LEFT_RESET;
                  move_arm(LEFT_RESET, pickup_goal.arm_name, 0);
                  ROS_INFO("Trying pickup with right arm");
                  pickup_goal.arm_name = "right_arm";       // Trying with right arm since left failed
                  active_arm_ = "right_arm";
                } else {
                  move_arm(RIGHT_RESET, pickup_goal.arm_name, 1);
                  return -1;
                }
            } else {
                ROS_INFO("Pickup succeeded. Will now try to place.");
            }
        }

        return 0;
    } //End function pickup

    bool place() {
        /// create a place location, offset by 10 cm from the pickup location
        geometry_msgs::PoseStamped place_location = pickup_location_;
        place_location.header.frame_id = target_graspable_object_.reference_frame_id;
        /// identity pose
        place_location.pose.orientation.w = 1;
        place_location.header.stamp = ros::Time::now();
        place_location.pose.position.y -= 0.05;

        /// put the object down
        ROS_INFO("Calling the place action");
        object_manipulation_msgs::PlaceGoal place_goal;
        /// place at the prepared location
        place_goal.place_locations.push_back(place_location);
        /// the collision names of both the objects and the table same as in the pickup action
        place_goal.collision_object_name = target_collision_name_;
        //place_goal.collision_support_surface_name = "table";

        /// information about which grasp was executed on the object, returned by the pickup action
        place_goal.grasp = pickup_result_.grasp;

        /// use the correct arm to place
        place_goal.arm_name = active_arm_;
        /// padding used when determining if the requested place location
        /// would bring the object in collision with the environment
        place_goal.place_padding = 0.05;
        /// how much the gripper should retreat after placing the object
        place_goal.desired_retreat_distance = 0.1;
        place_goal.min_retreat_distance = 0.05;
        /// we will be putting down the object along the "vertical" direction
        /// which is along the z axis in the base_link frame
        geometry_msgs::Vector3Stamped direction;
        direction.header.stamp = ros::Time::now();
        direction.header.frame_id = "base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = -1;
        place_goal.approach.direction = direction;
        /// request a vertical put down motion of 10cm before placing the object
        place_goal.approach.desired_distance = 0.05;
        place_goal.approach.min_distance = 0.02;
        /// we are not using tactile based placing
        place_goal.use_reactive_place = false;

        /// send the goal
        place_client_->sendGoal(place_goal);
        while (!place_client_->waitForResult(ros::Duration(10.0))) {
            ROS_INFO("Waiting for the place action...");
        }
        object_manipulation_msgs::PlaceResult place_result = *(place_client_->getResult());
        if (place_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ERROR("Place failed with error code %d", place_result.manipulation_result.value);
            //gripper_.open();
            geometry_msgs::Point reset_posn = LEFT_RESET;
            if (place_goal.arm_name == "right_arm") {
                reset_posn = RIGHT_RESET;
                move_arm(reset_posn, place_goal.arm_name, 1);
            } else
              move_arm(reset_posn, place_goal.arm_name, 0);
            return -1;
        }

        /// success!
        ROS_INFO("Success! Object moved.");
        geometry_msgs::Point reset_posn = LEFT_RESET;
        if (place_goal.arm_name == "right_arm") {
                reset_posn = RIGHT_RESET;
                move_arm(reset_posn, place_goal.arm_name, 1);
            } else
              move_arm(reset_posn, place_goal.arm_name, 0);
        return 0;
    }///End function place

    int move_arm(geometry_msgs::Point go_to, std::string arm_name, int arm_symbol) {

        //if (std::strcmp(arm_name, "right_arm")
        std::stringstream ss1, ss2;
        ss1 << "move_" << arm_name;
        //ss2 << arm_symbol << "_wrist_roll_link";
        //actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
        actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(ss1.str(),true);
        move_arm.waitForServer();
        ROS_INFO("DUPLO: Connected to server");
        arm_navigation_msgs::MoveArmGoal goalA;

        goalA.motion_plan_request.group_name = arm_name;
        goalA.motion_plan_request.num_planning_attempts = 1;
        goalA.motion_plan_request.planner_id = std::string("");
        goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
        goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

        arm_navigation_msgs::CollisionOperation cop2;
    	cop2.object1 = cop2.COLLISION_SET_ALL;
		cop2.object2 = cop2.COLLISION_SET_ALL;
		cop2.operation = cop2.DISABLE;    //0 = Disable, 1 = Enable
		cop2.penetration_distance = 0.25;
		goalA.operations.collision_operations.push_back(cop2);

        arm_navigation_msgs::SimplePoseConstraint desired_pose;
        //desired_pose.header.frame_id = "torso_lift_link";
        desired_pose.header.frame_id = "base_link";
        desired_pose.header.stamp = ros::Time::now();
        if (arm_symbol == 0)
           desired_pose.link_name = "l_wrist_roll_link";
        else
            desired_pose.link_name = "r_wrist_roll_link";
        //desired_pose.link_name = "r_wrist_roll_link";
        //	desired_pose.link_name = "r_gripper_palm_link";
        desired_pose.pose.position = go_to;
        //	desired_pose.pose.position.z = go_to.z + 0.194;
        /*
          	desired_pose.pose.position.x =  0.6; //0.75;
        	desired_pose.pose.position.y = -0.5;//-0.188;
        	desired_pose.pose.position.z = 0;
        */
        desired_pose.pose.orientation.x = 0;
        desired_pose.pose.orientation.y = 0;
        desired_pose.pose.orientation.z = 0;
        desired_pose.pose.orientation.w = 1;
        /*
        	desired_pose.pose.orientation.x = -0.74;
        	desired_pose.pose.orientation.y = -0.04;
        	desired_pose.pose.orientation.z = 0.67;
        	desired_pose.pose.orientation.w = -0.04;
        */
        desired_pose.absolute_position_tolerance.x = 0.02;
        desired_pose.absolute_position_tolerance.y = 0.02;
        desired_pose.absolute_position_tolerance.z = 0.02;

        desired_pose.absolute_roll_tolerance = 0.04;
        desired_pose.absolute_pitch_tolerance = 0.04;
        desired_pose.absolute_yaw_tolerance = 0.04;

        arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

        bool finished_within_time = false;
        move_arm.sendGoal(goalA);
        finished_within_time = move_arm.waitForResult(ros::Duration(10.0));
        if (!finished_within_time) {
            move_arm.cancelGoal();
            ROS_INFO("DUPLO: Timed out achieving goal A");
            return -1;
        } else {
            actionlib::SimpleClientGoalState state = move_arm.getState();
            bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            if(success) {
                ROS_INFO("DUPLO: Action finished: %s",state.toString().c_str());
                return 0;
            } else {
                ROS_INFO("DUPLO: Action failed: %s",state.toString().c_str());
                return -1;
            }
        }
        return 0;
    }

};//End Class definition

/*

/*
    arm_navigation_msgs::GetPlanningScene planning_scene_srv;
    if (!get_planning_scene_client_.call(planning_scene_srv))
    {
        ROS_ERROR("Could not connect to planning scene server.");
    }
    else
    {
        arm_navigation_msgs::CollisionMap cm = planning_scene_srv.response.planning_scene.collision_map;
        collision_map_pub_.publish(cm);
        ROS_INFO("number of collision objects before = %zu", planning_scene_srv.response.planning_scene.collision_objects.size());
    }
    geometry_msgs::Pose pose;
    pose.position.x = .6;
    pose.position.y = -.6;
    pose.position.z = 1.5;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    std::vector<double> dims;
    dims.push_back(0.1);
    dims.push_back(1.0);
    collision_obj("sample_obj", arm_navigation_msgs::CollisionObjectOperation::ADD,
                  pose, arm_navigation_msgs::Shape::CYLINDER, dims);


  bool remove_collision_objects()
  {
    arm_navigation_msgs::GetCollisionObjects col_obj_srv;
    col_obj_srv.request.include_points = false;
    if (!get_collision_objects_client_.call(col_obj_srv)) {
      ROS_ERROR("Could not connect to collision objects server.");
      return false;
    } else {
      //ROS_INFO("Added collision object");
      std::vector<arm_navigation_msgs::CollisionObject> vco = col_obj_srv.response.collision_objects;
      ROS_INFO("number of collision objects after = %zu", vco.size());

      for (size_t i = 0; i < vco.size(); i++) {
        collision_obj(vco[i].id, 1, vco[i].poses[0], vco[0].shapes[0].type, vco[0].shapes[0].dimensions);
      }

      return true;
    }
  }


bool collision_obj(std::string id, int operation, geometry_msgs::Pose pose,
                   int shape, std::vector<double> dims)
{
    arm_navigation_msgs::CollisionObject co;
    co.id = id;
    co.operation.operation = operation; //0: add, 1: remove, 2: detach and add; 3: attach and remove
    //arm_navigation_msgs::CollisionObjectOperation::ADD;
    co.header.frame_id = "base_link";
    co.header.stamp = ros::Time::now();
    arm_navigation_msgs::Shape object;
    object.type = shape;  //0: sphere; 1: box; 2: cylinder; 3: mesh
    //arm_navigation_msgs::Shape::CYLINDER;
    object.dimensions = dims;
    co.shapes.push_back(object);
    co.poses.push_back(pose);

    arm_navigation_msgs::GetPlanningScene planning_scene_srv;
    planning_scene_srv.request.planning_scene_diff.collision_objects.push_back(co);
    if (!get_planning_scene_client_.call(planning_scene_srv))
    {
        ROS_ERROR("Could not connect to planning scene server.");
        return false;
    }
    else
    {
        ROS_INFO("Added collision object");
        std::vector<arm_navigation_msgs::CollisionObject> vco = planning_scene_srv.response.planning_scene.collision_objects;
        ROS_INFO("number of collision objects after = %zu", vco.size());
        return true;
    }
}

  */


int main(int argc, char **argv)
{
    //initialize the ROS node
    ros::init(argc, argv, "pick_place");
    pickup_client_ = new PickupClient(PICKUP_ACTION_NAME, true);
    place_client_ = new PlaceClient(PLACE_ACTION_NAME, true);
    ros::NodeHandle n;
    PickPlace pp(n);
    ros::spin();
    //return(0);
}

/*
 if (argc < 3){
   ROS_ERROR("Usage: ./pick_place <scene_cloud.pcd> <target_cloud.pcd>");
   return -1;
 }
 */
