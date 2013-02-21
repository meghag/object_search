/*
 *  planner.h
 *  
 *
 *  Created by Megha Gupta on 12/10/12.
 *  Copyright 2012 USC. All rights reserved.
 *
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <cmath>
#include <ctime>
#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/OctomapROS.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
//#include <tf_conversions/tf_eigen.h>

extern "C" {
#include <gpcl/gpc.h>
}

#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>

#include "pcd_utils.h"
#include "TableTopObject.h"
//#include <tum_os/Clusters.h>
#include <tum_os/PlanService.h>
#include <tum_os/Execute_Plan.h>
#include <tum_os/Get_New_PCD.h>

using namespace pcl;
using namespace octomap;
using namespace std;

//typedef pcl::PointXYZRGB PointT;

//std::string fixed_frame_ = "tum_os_table";
std::string fixed_frame_ = "base_link";
//std::string fixed_frame_ = "head_mount_kinect_ir_link";//"map";
std::string mount_frame_ = "head_mount_link";
std::string rgb_optical_frame_ = "head_mount_kinect_ir_link";
std::string rgb_topic_ = "/head_mount_kinect/depth_registered/points";
std::string ik_frame_ = "/torso_lift_link";

const double MAX_BEAM_LENGTH = 1.0;
const octomap::point3d STEREO_ORIGIN(0.07,0.06,1.2);
const float table_height = 0.5;		
point3d BBX_MIN(0.9,-0.4,0.55);
point3d BBX_MAX(1.9,0.4,1);
bool data_from_bag = false;
bool data_to_bag = false;
std::string data_bag_name = "data.bag";
int bases[] = {2,3,5,7,11,13,17,19,23,29};
//int MAX_HORIZON = 2;

std::map<std::string, ros::Publisher*> belief_publishers;
std::map<std::string, ros::Publisher*> cloud_publishers;

struct Move {
	unsigned int cluster_idx;		//An index to refer to the object in the collection of visible objects.
	sensor_msgs::PointCloud2 objectToMove;
	tf::Pose sourcePose;
	tf::Pose destPose;

	//Constructor
	Move(unsigned int c, sensor_msgs::PointCloud2 pcd, tf::Pose s, tf::Pose p):cluster_idx(c), objectToMove(pcd), sourcePose(s), destPose(p){}
};

class Planner
{
public:
	Planner (ros::NodeHandle& n, int horizon);	//, octomap::OcTree & tree);
	~Planner (void);

private:
	//Functions
	//void planRequestCallback(const tum_os::PlanRequest::ConstPtr& plan_request);
	bool planRequestCallback(tum_os::PlanService::Request &plan_request, tum_os::PlanService::Response &plan_response);
	void call_plan(sensor_msgs::PointCloud2 objectCloud2);
	void pub_belief(const std::string &topic_name,const std::vector<tf::Pose> poses);
	void pubCloud(const std::string &topic_name, const pcl::PointCloud<PointT>::Ptr &cloud, std::string frame_id);
	tf::Stamped<tf::Pose> getPose(const std::string target_frame, const std::string lookup_frame, ros::Time tm);
	//void cluster(sensor_msgs::PointCloud2& cloud2, vector<sensor_msgs::PointCloud2>& clusters);

	TableTopObject createTTO(sensor_msgs::PointCloud2& cloud2);
	void findGridLocations(vector<sensor_msgs::PointCloud2> config);
	bool inFront(pcl::PointCloud<PointT> cloud, int cluster_idx);
	void make_grid(vector<sensor_msgs::PointCloud2> config);
	void display_grid();

	void samplePose(sensor_msgs::PointCloud2 target_cloud2,
					TableTopObject otherCloudTTO,
					vector<tf::Pose>& object_posterior_belief,
					bool check_hidden,
					bool check_visible);

	bool checkHiddenOrVisible(sensor_msgs::PointCloud2 object_cloud2,
					 TableTopObject otherCloudTTO,
					 tf::Transform ownTransform,
					 tf::Transform otherTransform,
					 bool check_hidden,
					 bool check_visible);

	bool generatePercentageIfRemoved(vector<tf::Pose> object_belief,
			vector<sensor_msgs::PointCloud2> movable_clusters,
			vector<int> movable_idx,
			map<int, double>& percentage);

	double generatePercentageIfDisplaced(vector<tf::Pose> object_belief,
									 vector<sensor_msgs::PointCloud2> new_config);

	void findMovable(vector<sensor_msgs::PointCloud2> config,
			vector<sensor_msgs::PointCloud2>& movable_clusters,
			vector<int>& movable_idx);

	void findPossibleMoves(sensor_msgs::PointCloud2& config,
			vector<sensor_msgs::PointCloud2> movable_clusters,
			vector<int> movable_idx,
			vector<Move>& possible_moves);

	void plan(int horizon,
			vector<sensor_msgs::PointCloud2> config,
			sensor_msgs::PointCloud2 other_cloud,
			vector<Move>& best_next_action_sequence,
			double& total_percentage_revealed_so_far);

	/*
	void random_plan(int horizon,
			vector<sensor_msgs::PointCloud2> config,
			sensor_msgs::PointCloud2 other_cloud,
			vector<Move>& action_sequence_so_far,
			double& total_percentage_revealed_so_far);
	*/

	void simulateMove(vector<sensor_msgs::PointCloud2> config,
			Move move, vector<sensor_msgs::PointCloud2>& new_config);

	void execute_plan();

	void getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
				geometry_msgs::PoseStamped &pose_stamped,
				geometry_msgs::Vector3 &dimensions);

	//Publishers, Subscribers, Service servers & clients
	ros::NodeHandle n_;
	//ros::Subscriber planRequestSub_;
	ros::ServiceServer planRequestServer_;
	ros::Publisher objectCloudPub_;
	ros::Publisher gridPub_;
	ros::Publisher clustersPub_;
	ros::Publisher movablePub_;
	ros::Publisher newPosePub_;
	ros::Publisher source_pose_pub_;
	ros::Publisher dest_pose_pub_;
	ros::ServiceClient manipulateClient_;
	ros::ServiceClient newpcdClient_;
	ros::ServiceClient bbx_client_;
	
	//Variables
	sensor_msgs::PointCloud2 targetCloud2_;
	sensor_msgs::PointCloud2 objectCloud2_;
	std::vector<sensor_msgs::PointCloud2> clustersDetected_;
	double tableHeight_;
	tf::TransformListener listener;
	tf::Stamped<tf::Pose> base_to_camera_;
	tf::Vector3 BB_MIN;
	tf::Vector3 BB_MAX;
	vector<Move> action_sequence_;
	bool new_data_wanted_;
	int MAX_HORIZON;
	float grid_resolution_;
	vector<vector<int> > grid_locations_;
};

#endif
