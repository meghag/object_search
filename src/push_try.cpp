/*********************************************
This program reads in a point cloud from the specified pcd file
and extracts all cylinders out of it iteratively. Not working well.
Only the first cylinder is created correctly.

Date created: Aug 13, 2012
Author: Megha Gupta
Modified: Aug 20, 2012

***********************************************/


#include <ros/ros.h>
#include <pcl/ros/conversions.h>
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
//const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
//const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
//const std::string COLLISION_PROCESSING_SERVICE_NAME = "/octomap_server";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_COLLISION_OBJECTS_NAME = "/environment_server/get_collision_objects";
//const std::string DEFAULT_CLUSTER_GRASP_PLANNER = "/plan_point_cluster_grasp";
//const std::string DEFAULT_DATABASE_GRASP_PLANNER = "/database_grasp_planning";

//typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> PickupClient;
//typedef actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> PlaceClient;
//typedef actionlib::SimpleActionClient<

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<PointT> pcd);

class Push
{

public:
	Push(ros::NodeHandle & n): n_(n), collision_object_current_id_(0) {
		LEFT_RESET.x = 0.3;
		LEFT_RESET.y = 0.7; //0.5
		LEFT_RESET.z = 1.1; //1.0
		RIGHT_RESET.x = 0.3;
		RIGHT_RESET.y = -0.7;  //-0.5
		RIGHT_RESET.z = 1.1;   //1.0

		active_arm_ = "right_arm";
		active_arm_sym_ = 1;

		collision_map_pub_ = n_.advertise<arm_navigation_msgs::CollisionMap>("my_collision_map",1);

		target_object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("target_object",1);

		collision_object_pub_ = n_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
		attached_object_pub_ = n_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);
		contact_point_pub_ = n_.advertise<geometry_msgs::PoseStamped>("contact_point",1);
		final_point_pub_ = n_.advertise<geometry_msgs::PoseStamped>("final_point",1);

		ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
		get_planning_scene_client_ = n_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);
		get_collision_objects_client_ = n_.serviceClient<arm_navigation_msgs::GetCollisionObjects>(GET_COLLISION_OBJECTS_NAME);

		bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
		clusters_sub_ = n_.subscribe("/clusters", 1, &Push::clusters_callback, this);
	}

private:
	ros::NodeHandle n_;

	//create service and action clients
	ros::ServiceClient get_planning_scene_client_;
	ros::ServiceClient get_collision_objects_client_;
	ros::ServiceClient bbx_client_;

	//Create Publishers & Subscribers
	ros::Subscriber clusters_sub_;
	ros::Publisher collision_map_pub_;
	ros::Publisher target_object_pub_;
	ros::Publisher collision_object_pub_;
	ros::Publisher attached_object_pub_;
	ros::Publisher contact_point_pub_;
	ros::Publisher final_point_pub_;

	// Other private variables
	std::vector<sensor_msgs::PointCloud2> clusters_;
	int collision_object_current_id_;
	//object_manipulation_msgs::GraspableObject target_graspable_object_;
	std::string target_collision_name_;
	//object_manipulation_msgs::PickupResult pickup_result_;
	std::vector<object_manipulation_msgs::ClusterBoundingBox> bboxes_;
	std::vector<std::string> collision_names_;

	geometry_msgs::Point LEFT_RESET;
	geometry_msgs::Point RIGHT_RESET;
	std::string active_arm_;
	int active_arm_sym_;

	Gripper gripper_;

	void clusters_callback(const tum_os::Clusters::ConstPtr& c) {
		ROS_INFO("Inside callback. Received clusters.");

		clusters_ = c->clusters;
		ROS_INFO("There are %zu clusters.", clusters_.size());

		if (clusters_.size() == 0) {
			ROS_ERROR("Found no clusters. Exiting.");
			return;
		}

		// 1. Remove all collision objects
		ROS_INFO("Removing all collision objects.");
		resetCollisionModels();

		std::vector<double> x_locations;
		//std::vector<std::string> collision_names;
		arm_navigation_msgs::CollisionOperation cop;

		for (size_t i = 0; i < clusters_.size(); i++) {
			clusters_[i].header.frame_id = "base_link";
			clusters_[i].header.stamp = ros::Time::now();

			/// Uncomment later

			// 2. Get bounding box of this cluster
			ROS_INFO("Finding bounding box of cluster %zu", i+1);
			object_manipulation_msgs::ClusterBoundingBox bbx;
			getClusterBoundingBox(clusters_[i], bbx.pose_stamped, bbx.dimensions);
			bboxes_.push_back(bbx);

			// 3. Add this bounding box as a collision object
			ROS_INFO("Adding a collision object for cluster %zu", i+1);
			std::string collision_name;
			processCollisionGeometryForBoundingBox(bbx, collision_name);
			collision_names_.push_back(collision_name);


			// 4. Create Graspable object for the target object
			//ROS_INFO("Creating graspable object from cluster %zu", i+1);
			//object_manipulation_msgs::GraspableObject go;
			//create_graspable_object(clusters_[i], target_collision_name_, target_graspable_object_);
			//target_collision_name_ = "object_3";
			//ROS_INFO("Created graspable object %zu", i);

			/// Uncomment later
			/*
			// 5. Find centroid (location) of each cluster
			ROS_INFO("Finding centroid of cluster %zu", i+1);
			pcl::PointCloud<PointT> pcd;
			fromROSMsg(clusters_[i], pcd);
			geometry_msgs::Point location = find_centroid(pcd);
			x_locations.push_back(location.x);
			ROS_INFO("Centroid: x = %f, y = %f, z = %f", location.x, location.y, location.z);

					}

					// 6. Find the closest cluster to the robot
					ROS_INFO("Finding the closest object");
					int closest = 0;
					double x_closest = x_locations[0];
					for (size_t j = 1; j < x_locations.size(); j++) {
			if (x_locations[j] < x_closest) {
				x_closest = x_locations[j];
				closest = j;
			}
					}
					ROS_INFO("The closest object is cluster %d with x location  = %f and name %s", closest+1,
			           x_locations[closest], collision_names[closest].c_str());
			        target_object_pub_.publish(clusters_[closest]);
			        */

			// 7. Push the closest object 2 cm behind
			ROS_INFO("Creating a collision operation");
			//arm_navigation_msgs::CollisionOperation cop;
			//collision_op("gripper", collision_names[closest], 0, 0.2, cop);
			collision_op("gripper", "temp", 0, 0.2, cop);   /// Temp hack


			geometry_msgs::Point contact_point, final_point;
			pcl::PointCloud<PointT> pcd;
			//fromROSMsg(clusters_[closest], pcd);
			fromROSMsg(clusters_[i], pcd);
			ROS_INFO("Calculating the contact point and final point");
			std::vector<geometry_msgs::Point> extents = find_extents(pcd);
			contact_point.x = extents[0].x - 0.3;
			contact_point.y = 0.5*(extents[3].y + extents[2].y);
			contact_point.z = extents[5].z - 0.05;
			//contact_point.z = 0.5*(extents[4].z + extents[5].z);

			final_point = contact_point;
			final_point.x+= 0.25;
			ROS_INFO("Contact Point = %f, %f, %f \t Final point = %f, %f, %f",
			         contact_point.x, contact_point.y, contact_point.z,
			         final_point.x, final_point.y, final_point.z);

			geometry_msgs::PoseStamped topub;
			topub.header.frame_id = "base_link";
			topub.header.stamp = ros::Time::now();
			topub.pose.orientation.x = 0.0;
			topub.pose.orientation.y = 0.0;
			topub.pose.orientation.z = 0.0;
			topub.pose.orientation.w = 1.0;
			topub.pose.position = contact_point;
			contact_point_pub_.publish(topub);
			topub.pose.position = final_point;
			final_point_pub_.publish(topub);

			gripper_.open();
			ROS_INFO("Calling move arm");
			if (!move_arm(contact_point, active_arm_, active_arm_sym_, cop, 100)) {
				///Arm motion to contact point succeeded
				if (!move_arm(final_point, active_arm_, active_arm_sym_, cop, i)) {
					///Arm motion to final point succeeded
					move_arm(contact_point, active_arm_, active_arm_sym_, cop, i);
				}
				//move_arm(LEFT_RESET, "left_arm", 0, cop);
				//move_arm(RIGHT_RESET, "right_arm", 1, cop);
			}
		}
		move_arm(RIGHT_RESET, "right_arm", 1, cop, 100);
		ROS_INFO("Exiting callback");
	}

	geometry_msgs::Point find_centroid(pcl::PointCloud<PointT> pcd) {
		ROS_INFO("Inside find_centroid");
		double avgx = 0.0, avgy = 0.0, avgz = 0.0;
		geometry_msgs::Point position;
		for (std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it1 = pcd.points.begin();
		        it1 != pcd.points.end(); ++it1) {
			avgx += it1->x;
			avgy += it1->y;
			avgz += it1->z;
		}
		position.x = avgx/pcd.points.size();
		position.y = avgy/pcd.points.size();
		position.z = avgz/pcd.points.size();

		return position;
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

	void collision_op(std::string object1, std::string object2,
	                  int operation, double penetration,
	                  arm_navigation_msgs::CollisionOperation& cop) {
		cop.object1 = object1;
		cop.object2 = object2;
		cop.operation = operation;    //0 = Disable, 1 = Enable
		cop.penetration_distance = penetration;

		/*		arm_navigation_msgs::GetPlanningScene planning_scene_srv;
				planning_scene_srv.request.operations.collision_operations.push_back(cop);
				if (!get_planning_scene_client_.call(planning_scene_srv)) {
					ROS_ERROR("Could not connect to planning scene server.");
					return false;
				} else {
					ROS_INFO("Added collision operation");
					return true;
				}
				*/
	}

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
		goalA.accept_partial_plans = true;

		arm_navigation_msgs::SimplePoseConstraint desired_pose;
		//desired_pose.header.frame_id = "torso_lift_link";
		desired_pose.header.frame_id = "base_link";
		desired_pose.header.stamp = ros::Time::now();
		if (arm_symbol == 0)
			desired_pose.link_name = "l_wrist_roll_link";
		else
			desired_pose.link_name = "r_wrist_roll_link";
		desired_pose.pose.position = go_to;

		desired_pose.pose.orientation.x = 0;
		desired_pose.pose.orientation.y = 0;
		desired_pose.pose.orientation.z = 0;
		desired_pose.pose.orientation.w = 1;

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

	int move_arm(geometry_msgs::Point go_to, std::string arm_name, int arm_symbol,
	             arm_navigation_msgs::CollisionOperation cop, int cluster_id) {
		ROS_INFO("Inside move arm");
		//if (std::strcmp(arm_name, "right_arm")
		std::stringstream ss1, ss2;
		ss1 << "move_" << arm_name;
		//ss2 << arm_symbol << "_wrist_roll_link";
		//actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(ss1.str(),true);
		ROS_INFO("Waiting for server");
		move_arm.waitForServer();
		ROS_INFO("Connected to server");
		arm_navigation_msgs::MoveArmGoal goalA;

		ROS_INFO("Creating move arm goal");
		goalA.motion_plan_request.group_name = arm_name;
		goalA.motion_plan_request.num_planning_attempts = 1;
		goalA.motion_plan_request.planner_id = std::string("");
		goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
		goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
		goalA.accept_partial_plans = true;
		cop.operation = cop.DISABLE;
		goalA.operations.collision_operations.push_back(cop);

		if (cluster_id != 100) {
			arm_navigation_msgs::CollisionObject co;
			co.id = collision_names_[cluster_id];
			co.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD; //0: add, 1: remove, 2: detach and add; 3: attach and remove
			//arm_navigation_msgs::CollisionObjectOperation::ADD;
			co.header.frame_id = "base_link";
			co.header.stamp = ros::Time::now();
			arm_navigation_msgs::Shape object;
			object.type = arm_navigation_msgs::Shape::BOX;  //0: sphere; 1: box; 2: cylinder; 3: mesh
			//arm_navigation_msgs::Shape::CYLINDER;
			object.dimensions.resize(3);
			object.dimensions[0] = bboxes_[cluster_id].dimensions.x;
			object.dimensions[1] = bboxes_[cluster_id].dimensions.y;
			object.dimensions[2] = bboxes_[cluster_id].dimensions.z;
			co.shapes.push_back(object);

			co.poses.push_back(bboxes_[cluster_id].pose_stamped.pose);
			goalA.planning_scene_diff.collision_objects.push_back(co);

			/*
			object.dimensions.push_back(0.1);
			object.dimensions.push_back(0.3);
			object.dimensions.push_back(0.6);
			geometry_msgs::Pose pose;
			pose.position.x = .7;
			pose.position.y = -0.1;
			pose.position.z = 1.0;
			pose.orientation.x = 0;
			pose.orientation.y = 0;
			pose.orientation.z = 0;
			pose.orientation.w = 1;
			co.poses.push_back(pose);
			*/

			arm_navigation_msgs::CollisionOperation cop2;
			cop2.object1 = cop2.COLLISION_SET_ALL;
			cop2.object2 = collision_names_[cluster_id];
			cop2.operation = cop2.DISABLE;    //0 = Disable, 1 = Enable
			cop2.penetration_distance = 0.25;
			goalA.operations.collision_operations.push_back(cop2);
		}

		/*
		arm_navigation_msgs::CollisionOperation cop2;
		cop2.object1 = cop2.COLLISION_SET_ALL;
		cop2.object2 = cop2.COLLISION_SET_ALL;
		cop2.operation = cop2.DISABLE;    //0 = Disable, 1 = Enable
		cop2.penetration_distance = 0.25;
		goalA.operations.collision_operations.push_back(cop2);
		*/

		arm_navigation_msgs::SimplePoseConstraint desired_pose;
		//desired_pose.header.frame_id = "torso_lift_link";
		desired_pose.header.frame_id = "base_link";
		desired_pose.header.stamp = ros::Time::now();
		if (arm_symbol == 0)
			desired_pose.link_name = "l_wrist_roll_link";
		else
			desired_pose.link_name = "r_wrist_roll_link";
		desired_pose.pose.position = go_to;

		desired_pose.pose.orientation.x = 0;
		desired_pose.pose.orientation.y = 0;
		desired_pose.pose.orientation.z = 0;
		desired_pose.pose.orientation.w = 1;

		desired_pose.absolute_position_tolerance.x = 0.02;
		desired_pose.absolute_position_tolerance.y = 0.02;
		desired_pose.absolute_position_tolerance.z = 0.02;

		desired_pose.absolute_roll_tolerance = 0.04;
		desired_pose.absolute_pitch_tolerance = 0.04;
		desired_pose.absolute_yaw_tolerance = 0.04;

		arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

		bool finished_within_time = false;
		ROS_INFO("Sending move arm goal");
		move_arm.sendGoal(goalA);
		finished_within_time = move_arm.waitForResult(ros::Duration(15.0));
		if (!finished_within_time) {
			move_arm.cancelGoal();
			ROS_INFO("Timed out achieving goal A");
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


int main(int argc, char **argv)
{
	//initialize the ROS node
	ros::init(argc, argv, "push");
	ros::NodeHandle n;
	Push p(n);
	ros::spin();
	//return(0);
}



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

        //pass the collision name of the table, also returned by the collision
        //map processor
        //pickup_goal.collision_support_surface_name = "table";

        //pick up the object with the right arm
        pickup_goal.arm_name = "left_arm";
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
            return -1;
        }

        return 0;
    } //End function pickup

    bool place() {
        //create a place location, offset by 10 cm from the pickup location
        geometry_msgs::PoseStamped place_location;
        place_location.header.frame_id = target_graspable_object_.reference_frame_id;
        //identity pose
        place_location.pose.orientation.w = 1;
        place_location.header.stamp = ros::Time::now();
        place_location.pose.position.x -= 0.02;

        //put the object down
        ROS_INFO("Calling the place action");
        object_manipulation_msgs::PlaceGoal place_goal;
        //place at the prepared location
        place_goal.place_locations.push_back(place_location);
        //the collision names of both the objects and the table
        //same as in the pickup action
        place_goal.collision_object_name = target_collision_name_;
        //place_goal.collision_support_surface_name = "table";
        //information about which grasp was executed on the object,
        //returned by the pickup action

        //------------------UNCOMMENT THIS LATER----------------------------
        place_goal.grasp = pickup_result_.grasp;

        //use the right rm to place
        place_goal.arm_name = "left_arm";
        //padding used when determining if the requested place location
        //would bring the object in collision with the environment
        place_goal.place_padding = 0.02;
        //how much the gripper should retreat after placing the object
        place_goal.desired_retreat_distance = 0.1;
        place_goal.min_retreat_distance = 0.05;
        //we will be putting down the object along the "vertical" direction
        //which is along the z axis in the base_link frame
        geometry_msgs::Vector3Stamped direction;
        direction.header.stamp = ros::Time::now();
        direction.header.frame_id = "base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = -1;
        place_goal.approach.direction = direction;
        //request a vertical put down motion of 10cm before placing the object
        place_goal.approach.desired_distance = 0.05;
        place_goal.approach.min_distance = 0.02;
        //we are not using tactile based placing
        place_goal.use_reactive_place = false;


        //send the goal
        place_client_->sendGoal(place_goal);
        while (!place_client_->waitForResult(ros::Duration(10.0))) {
            ROS_INFO("Waiting for the place action...");
        }
        object_manipulation_msgs::PlaceResult place_result = *(place_client_->getResult());
        if (place_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ERROR("Place failed with error code %d", place_result.manipulation_result.value);
            return -1;
        }

        //success!
        ROS_INFO("Success! Object moved.");
        return 0;
    }//End function place

  */




/*
 if (argc < 3){
   ROS_ERROR("Usage: ./pick_place <scene_cloud.pcd> <target_cloud.pcd>");
   return -1;
 }
 */

