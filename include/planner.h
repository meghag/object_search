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
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
//#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/extract_clusters.h>

extern "C" {
#include <gpcl/gpc.h>
}

#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

//#include "set_marker.h"
//#include "create_marker_array.h"
#include "pcd_utils.h"

#include "TableTopObject.h"
//#include <tum_os/Clusters.h>
#include <tum_os/PlanRequest.h>
#include <tum_os/Execute_Plan.h>
#include <tum_os/Get_New_PCD.h>

//#include "object_search_pkg/Plan_Actions.h"

using namespace pcl;
using namespace octomap;
using namespace std;

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
	//bool planActionsCallback(object_search_pkg::Plan_Actions::Request &req,
	//		object_search_pkg::Plan_Actions::Response &res);

	void planRequestCallback(const tum_os::PlanRequest::ConstPtr& plan_request);
	void pub_belief(const std::string &topic_name,const std::vector<tf::Pose> poses);
	void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id);
	tf::Stamped<tf::Pose> getPose(const std::string target_frame, const std::string lookup_frame, ros::Time tm);
	//void cluster(sensor_msgs::PointCloud2& cloud2, vector<sensor_msgs::PointCloud2>& clusters);

	TableTopObject createTTO(sensor_msgs::PointCloud2& cloud2);

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

	bool generatePercentage(vector<tf::Pose> object_belief,
			vector<sensor_msgs::PointCloud2> visible_clusters,
			vector<int> visible_idx,
			map<int, double>& percentage);

	void findVisible(vector<sensor_msgs::PointCloud2> config,
			vector<sensor_msgs::PointCloud2>& visible_clusters,
			vector<int>& visible_idx);

	void findPossibleMoves(sensor_msgs::PointCloud2& config,
			vector<sensor_msgs::PointCloud2> visible_clusters,
			vector<int> visible_idx,
			vector<Move>& possible_moves);

	void plan(int horizon,
			vector<sensor_msgs::PointCloud2> config,
			sensor_msgs::PointCloud2 other_cloud,
			vector<Move>& best_next_action_sequence,
			double& total_percentage_revealed_so_far);

	void simulateMove(vector<sensor_msgs::PointCloud2> config,
			Move move, vector<sensor_msgs::PointCloud2>& new_config);

	void execute_plan();

	//bool callGraspingService (void);
	//int findHiddenVoxels(PointCloud<PointXYZ>::Ptr& object);
	//vector<geometry_msgs::Point> find_corners(PointCloud<PointXYZ>::Ptr& cloud);
	//bool callNewDataService (void);

	//Publishers, Subscribers, Service servers & clients
	ros::NodeHandle n_;
	ros::Subscriber planRequestSub_;
	ros::Publisher objectCloudPub_;
	ros::Publisher clustersPub_;
	ros::Publisher visiblePub_;
	ros::Publisher newPosePub_;
	ros::Publisher source_pose_pub_;
	ros::Publisher dest_pose_pub_;
	ros::ServiceClient manipulateClient_;
	ros::ServiceClient newpcdClient_;
	
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

	//map<pair<int,PointCloud<PointXYZ> > > knownObjects_;			//Map of known object point clouds with keys as object id
	//map<pair<int,PointCloud<PointXYZ> > > visibleObjects_;			//Map of visible object point clouds with keys as object id
	//float treeResolution_;
	//octomap::OcTree tree_;
};

#endif
