/*********************************************
This program reads in a point cloud from the specified pcd file 
and extracts all cylinders out of it iteratively. Not working well.
Only the first cylinder is created correctly.

Date created: July 31, 2012
Author: Megha Gupta

***********************************************/


#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <arm_navigation_msgs/GetCollisionObjects.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include "tum_os/Clusters.h"

//set service and action names
const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
const std::string COLLISION_PROCESSING_SERVICE_NAME = 
		"/octomap_server";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
const std::string DEFAULT_CLUSTER_GRASP_PLANNER = "/plan_point_cluster_grasp";
const std::string DEFAULT_DATABASE_GRASP_PLANNER = "/database_grasp_planning";

typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> PickupClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> PlaceClient;
//typedef actionlib::SimpleActionClient<

class PickPlace
{

public:
  PickPlace(ros::NodeHandle & n):n_(n)
  {
    //wait for pickup server
    while(!pickup_client_.waitForServer(ros::Duration(2.0)) && n_.ok())
      ROS_INFO_STREAM("Waiting for pickup server " << PICKUP_ACTION_NAME);
    if (!n_.ok()) exit(0);  
    pickup_client_(PICKUP_ACTION_NAME, true);

    //wait for place server
    while(!place_client_.waitForServer(ros::Duration(2.0)) && n_.ok())
      ROS_INFO_STREAM("Waiting for place server " << PLACE_ACTION_NAME);
    if (!n_.ok()) exit(0);
    place_client_(PLACE_ACTION_NAME, true);

    //wait for cluster grasp planner
    /*   while(!place_client.waitForServer(ros::Duration(2.0)) && n_.ok())
      ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
    if (!n_.ok()) exit(0);
    */
    
    //wait for collision map processing client
    while (!ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, 
					 ros::Duration(30.0)) && n_.ok()) 
      ROS_INFO("DUPLO: Waiting for collision processing service to come up");
    if (!n_.ok()) exit(0);
    collision_processing_srv_ = 
      n_.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
		(COLLISION_PROCESSING_SERVICE_NAME, true);

    // clusters_sub_ = n_.subscribe("/clusters", 1, &PickPlace::callback, this);

    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    ros::ServiceClient get_planning_scene_client = 
      n_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

    arm_navigation_msgs::GetPlanningScene planning_scene_srv;
    if (!get_planning_scene_client.call(planning_scene_srv)) {
      ROS_ERROR("Could not connect to planning scene server.");
    } else {
      arm_navigation_msgs::CollisionMap cm = planning_scene_srv.response.collision_map;
      collision_map_pub_.publish(cm);
    }
  };
  
private:
  //create service and action clients
  
  PickupClient pickup_client_;
  PlaceClient  place_client_;
  ros::Subscriber clusters_sub_;
  std::vector<sensor_msgs::PointCloud2> clusters_;
  

  void callback(const tum_os::Clusters::ConstPtr& c)
  {
    clusters_ = c->clusters;

    arm_navigation_msgs::GetPlanningScene

    createGraspableObject(clusters_[0]);
    pickup();
  };
 
  //Create a graspable object of the target point cloud
  void createGraspableObject(sensor_msgs::PointCloud2)
  {
    

  }; //End function createGraspableObject
   
  bool pickup()
  {
    //call object pickup
    ROS_INFO("Calling the pickup action");
    object_manipulation_msgs::PickupGoal pickup_goal;
    //pass one of the graspable objects returned 
    //by the collision map processor
    pickup_goal.target = processing_call.response.graspable_objects.at(0);
    //pass the name that the object has in the collision environment
    //this name was also returned by the collision map processor
    pickup_goal.collision_object_name = 
      processing_call.response.collision_object_names.at(0);
    //pass the collision name of the table, also returned by the collision 
    //map processor
    pickup_goal.collision_support_surface_name = 
      processing_call.response.collision_support_surface_name;
    //pick up the object with the right arm
    pickup_goal.arm_name = "right_arm";
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
    pickup_goal.lift.desired_distance = 0.1;
    pickup_goal.lift.min_distance = 0.05;
    //do not use tactile-based grasping or tactile-based lift
    pickup_goal.use_reactive_lift = false;
    pickup_goal.use_reactive_execution = false;
    //send the goal
    pickup_client.sendGoal(pickup_goal);
    while (!pickup_client.waitForResult(ros::Duration(10.0)))
      {
	ROS_INFO("Waiting for the pickup action...");
      }
    object_manipulation_msgs::PickupResult pickup_result = 
      *(pickup_client.getResult());
    if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	ROS_ERROR("The pickup action has failed with result code %d", 
		  pickup_result.manipulation_result.value);
	return -1;
      }
  }; //End function pickup

  bool place()
  {
    //create a place location, offset by 10 cm from the pickup location
    geometry_msgs::PoseStamped place_location;
    place_location.header.frame_id = processing_call.response.graspable_objects.at(0).reference_frame_id; 
    //identity pose
    place_location.pose.orientation.w = 1;  
    place_location.header.stamp = ros::Time::now();
    place_location.pose.position.x += 0.1;

    //put the object down
    ROS_INFO("Calling the place action");
    object_manipulation_msgs::PlaceGoal place_goal;
    //place at the prepared location
    place_goal.place_locations.push_back(place_location);
    //the collision names of both the objects and the table
    //same as in the pickup action
    place_goal.collision_object_name = 
      processing_call.response.collision_object_names.at(0); 
    place_goal.collision_support_surface_name = 
      processing_call.response.collision_support_surface_name;
    //information about which grasp was executed on the object, 
    //returned by the pickup action
    place_goal.grasp = pickup_result.grasp;
    //use the right rm to place
    place_goal.arm_name = "right_arm";
    //padding used when determining if the requested place location
    //would bring the object in collision with the environment
    place_goal.place_padding = 0.02;
    //how much the gripper should retreat after placing the object
    place_goal.desired_retreat_distance = 0.1;
    place_goal.min_retreat_distance = 0.05;
    //we will be putting down the object along the "vertical" direction
    //which is along the z axis in the base_link frame
    direction.header.stamp = ros::Time::now();
    direction.header.frame_id = "base_link";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = -1;
    place_goal.approach.direction = direction;
    //request a vertical put down motion of 10cm before placing the object 
    place_goal.approach.desired_distance = 0.1;
    place_goal.approach.min_distance = 0.05;
    //we are not using tactile based placing
    place_goal.use_reactive_place = false;
    //send the goal
    place_client.sendGoal(place_goal);
    while (!place_client.waitForResult(ros::Duration(10.0)))
      {
	ROS_INFO("Waiting for the place action...");
      }
    object_manipulation_msgs::PlaceResult place_result = 
      *(place_client.getResult());
    if (place_client.getState() != 
	actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	ROS_ERROR("Place failed with error code %d", 
		  place_result.manipulation_result.value);
	return -1;
      }

    //success!
    ROS_INFO("Success! Object moved.");
    return 0;
  };//End function place

};//End Class definition


int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle n;
  PickPlace pp(n);
  ros::spin();
  return(0);
}

 /*
  if (argc < 3){
    ROS_ERROR("Usage: ./pick_place <scene_cloud.pcd> <target_cloud.pcd>");
    return -1;
  }
  */
