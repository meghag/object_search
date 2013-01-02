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
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/OctomapROS.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "find_extents.h"
#include "set_marker.h"
//#include "create_marker_array.h"
//#include "pass_through_gen.h"
//#include "findHiddenVoxels.h"

#include "TableTopObject.h"
#include "sampling.h"

//#include "object_search_pkg/Plan_Actions.h"
//#include "object_search_pkg/Get_New_PCD.h"

using namespace pcl;
using namespace octomap;
using namespace std;

const double MAX_BEAM_LENGTH = 1.0;
const octomap::point3d STEREO_ORIGIN(0.07,0.06,1.2);
const float table_height = 0.5;		
point3d BBX_MIN(0.9,-0.4,0.55);
point3d BBX_MAX(1.9,0.4,1);

namespace object_search{
	class Planner
	{
		public:
			Planner (ros::NodeHandle & n, octomap::OcTree & tree);
			~Planner (void);

		private:
			//Functions
			bool planActionsCallback(object_search_pkg::Plan_Actions::Request &req,
			                         object_search_pkg::Plan_Actions::Response &res);
			//bool callGraspingService (void);
			int findHiddenVoxels(PointCloud<PointXYZ>::Ptr& object);
			vector<geometry_msgs::Point> find_corners(PointCloud<PointXYZ>::Ptr& cloud);
			//bool callNewDataService (void);

			//Variables
			ros::NodeHandle n_;
			//ros::Publisher octreePub_;
			//ros::Publisher objectCloudPub_;
			//ros::Publisher raycastPub_;
			//ros::ServiceServer planActionsServer_;
			ros::Subscriber visibleObjectsSub_;
			ros::Subscriber octreeSub_;
			//ros::ServiceClient graspDuploClient_;
			//ros::ServiceClient planActionsClient_;

			//pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
			//float treeResolution_;
			//octomap::OcTree tree_;
			//pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud_;
			//sensor_msgs::PointCloud2 objectCloud2_;
			map<pair<int,PointCloud<PointXYZ> > > knownObjects_;			//Map of known object point clouds with keys as object id
			map<pair<int,PointCloud<PointXYZ> > > visibleObjects_;			//Map of visible object point clouds with keys as object id
			set<int> occupiedCells_;
			set<int> freeCells_;			
	};
}

#endif
