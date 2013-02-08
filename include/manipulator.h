/*
 *  manipulator.h
 *  
 *  Created by Megha Gupta on 01/25/13.
 *  Copyright 2013 USC. All rights reserved.
 *
 */

#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>

#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <arm_navigation_msgs/GetCollisionObjects.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>

#include <tum_os/Execute_Plan.h>

//#include <arm_navigation_msgs/CollisionObject.h>
//#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include "robothead.h"
#include "gripper.h"
//#include "move_arm.h"
#include "set_marker.h"

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
//typedef pcl::PointXYZRGB PointT;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_COLLISION_OBJECTS_NAME = "/environment_server/get_collision_objects";

class Manipulator
{
public:
	Manipulator (ros::NodeHandle & n);
	~Manipulator (void);

private:
	//Functions
	bool callback(tum_os::Execute_Plan::Request &req, tum_os::Execute_Plan::Response &res);
	bool pick_n_place(size_t idx);

	void resetCollisionModels();

	void getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
			geometry_msgs::PoseStamped &pose_stamped,
			geometry_msgs::Vector3 &dimensions);

	void processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box,
			std::string &collision_name);

	void collision_op(std::string object1, std::string object2,
			int operation, double penetration,
			arm_navigation_msgs::CollisionOperation& cop);

	void add_collision_object(arm_navigation_msgs::CollisionObject& cob, int operation);

	void add_attached_object(arm_navigation_msgs::AttachedCollisionObject& acob, int operation);

	int move_arm(geometry_msgs::Point go_to, int cluster_id, int cluster_op, bool attach, int plan_id, int action);

	vector<geometry_msgs::Point> waypoints(size_t idx);

	void breakpoint();

	//Variables
	ros::NodeHandle n_;
	ros::ServiceServer manipulate_service_;
	ros::Publisher marker_pub_;
	ros::Publisher coll_obj_pub_;
	ros::Publisher target_pub_;
	ros::Publisher bbx_pub_;
	ros::Publisher source_pose_pub_;
	ros::Publisher dest_pose_pub_;
	ros::ServiceClient collision_processing_srv;

	ros::ServiceClient get_planning_scene_client_;
	ros::ServiceClient get_collision_objects_client_;
	ros::ServiceClient bbx_client_;
	//ros::Publisher collision_map_pub_;
	//ros::Publisher target_object_pub_;
	ros::Publisher collision_object_pub_;

	RobotHead head_;
	Gripper gripper_;

	//bool manipulate_service_called_;
	uint32_t shape[4];
	geometry_msgs::PointStamped reset_posn_;
	//vector<sensor_msgs::PointCloud2> vec_target_cloud2_;
	//std::vector<geometry_msgs::Point> table_extent_;

	object_manipulation_msgs::ClusterBoundingBox bbx_;
	std::string collision_name_;

	geometry_msgs::Point LEFT_RESET;
	geometry_msgs::Point RIGHT_RESET;
	geometry_msgs::Point RESET_POSN;
	geometry_msgs::Point RED;
	geometry_msgs::Point BLUE;
	geometry_msgs::Point GREEN;
	geometry_msgs::Point active_reset_;
	std::string active_arm_;
	//int active_arm_sym_;
	char active_arm_sym_;

	vector<int> cluster_idx_;
	vector<sensor_msgs::PointCloud2> object_to_move_;
	vector<geometry_msgs::Pose> source_pose_;
	vector<geometry_msgs::Pose> dest_pose_;
	sensor_msgs::PointCloud2 target_cloud2_;
	double table_height_;
};

#endif
