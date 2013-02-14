/*********************************************
This node subscribes to the topic defined by 'cloud_topic' and reads in a
point cloud. It then transforms it to base link and extracts out the object
cloud by pass-through filtering and planar segmentation.
The object cloud is saved as a pcd file called 'object_cloud.pcd'

Date created: Feb 9, 2013
Author: Megha Gupta

 ***********************************************/

#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "laser_geometry/laser_geometry.h"
#include <pr2_arm_navigation_perception/BuildCloudAngle.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/tf.h>

#include "pcd_utils.h"

#include <tum_os/PlanRequest.h>
#include <tum_os/Get_New_PCD.h>

//std::string cloud_topic = "/head_mount_kinect/depth_registered/points";
// std::string cloud_topic = "/narrow_stereo_textured/points2";

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

using namespace std;
using namespace pcl;

tf::Vector3 BB_MIN(0.55,-0.8,0.9);
tf::Vector3 BB_MAX(1.1,0.8,1.25);

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanReader{

public:

	ros::NodeHandle n_;
	tf::TransformListener listener_;
	ros::Publisher scan_pub_;
	ros::Publisher input_pub_;
	ros::Publisher planar_pub_;
	ros::Publisher object_pub_;
	ros::Publisher planRequestPub_;

private:
	bool new_data_wanted_;
	sensor_msgs::PointCloud cloud_;
	sensor_msgs::PointCloud2 cloud2_;

public:
	LaserScanReader(ros::NodeHandle n) :
		n_(n)
	{
		scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/assembled_laser_cloud",1);

		input_pub_ = n_.advertise<sensor_msgs::PointCloud2>("input_cloud",1);
		planar_pub_ = n_.advertise<sensor_msgs::PointCloud2>("planar_cloud",1);
		object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("object_cloud_from_cam",1);
		planRequestPub_ = n_.advertise<tum_os::PlanRequest>("plan_request",1);
		new_data_wanted_ = true;
		scan();
	}

	bool scan()
	{
		pr2_arm_navigation_perception::BuildCloudAngle::Request snap_req;
		pr2_arm_navigation_perception::BuildCloudAngle::Response snap_res;

		snap_req.angle_begin = -0.6;			//angle above the XY plane
		snap_req.angle_end = 0.5;				//angle below the XY plane
		snap_req.duration = 8.0;

		if (!ros::service::call("laser_assembler_server/single_sweep_cloud", snap_req, snap_res)){
			ROS_ERROR("PointCloud: error setting laser snapshotter service");
			return false;

		}else{
			ROS_INFO("PointCloud : laser snapshotter with angle_begin %f, angle_end %f and duration %f",
					snap_req.angle_begin, snap_req.angle_end, snap_req.duration);
		}

		ROS_INFO("PointCloud: received point cloud of size %i from point cloud assembler", (int)snap_res.cloud.points.size());

		stopTilt();

		cloud_ = snap_res.cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		sensor_msgs::convertPointCloudToPointCloud2(cloud_, cloud2_);
		pcl::fromROSMsg(cloud2_, *pcl_cloud);
		scan_pub_.publish(cloud2_);

		getObjectCloud(pcl_cloud);

		return true;

	}

	bool stopTilt()
	{
		pr2_msgs::SetPeriodicCmd::Request scan_req;
		pr2_msgs::SetPeriodicCmd::Response scan_res;

		float min_ang = -0.0;
		float max_ang = 0.0;

		scan_req.command.amplitude  = fabs(min_ang - max_ang) / 2.0;
		scan_req.command.offset = (min_ang + max_ang) / 2.0;
		scan_req.command.period = 3.0; //2.3125;
		scan_req.command.profile = "linear";


		if (!ros::service::call("laser_tilt_controller/set_periodic_cmd", scan_req, scan_res)){
			ROS_ERROR("TiltLaser: error setting laser scanner periodic command");
			return false;
		}else{
			ROS_INFO("TiltLaser : commanded tilt laser scanner with period %f, amplitude %f and offset %f",
					scan_req.command.period, scan_req.command.amplitude, scan_req.command.offset);
			return true;
		}
	}

	void getObjectCloud(pcl::PointCloud<PointT>::Ptr cloud)
	{
		pcl::PointCloud<PointT>::Ptr planarCloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr objectCloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr pcdFiltered(new pcl::PointCloud<PointT>);	//Filtered cloud
		sensor_msgs::PointCloud2::Ptr objectCloud2 (new sensor_msgs::PointCloud2 ());
		sensor_msgs::PointCloud2::Ptr filteredObjectCloud2 (new sensor_msgs::PointCloud2 ());

		sensor_msgs::PointCloud2 topub;

		/****************** Filter out the non-table points ******************/
		//pass_through_gen(cloud,pcdFiltered,true,0,1,true,-0.32,0.32,true,1.012,1.3);   // Kinect and shelf
		//pass_through_gen(cloud,pcdFiltered,true,0,1,true,-0.32,0.32,true,1.07,1.3);    // Narrow stereo and shelf
		//pass_through_gen(cloud,pcdFiltered,true,0,1.4,true,-1.0,1.0,true,0.81,1.1);      // Narrow stereo and simulation table
		pass_through_gen(cloud,pcdFiltered,true,BB_MIN.x(),BB_MAX.x(),true,BB_MIN.y(),BB_MAX.y(),true,BB_MIN.z(),BB_MAX.z()-0.15);      // Narrow stereo and simulation table
		if (pcdFiltered->points.size() > 0) {
			new_data_wanted_ = false;
			toROSMsg(*pcdFiltered, topub);
			input_pub_.publish(topub);

			/* ********* Segmentation of the cloud into table and object clouds **********/
			planar_seg(pcdFiltered,planarCloud,objectCloud,(string)"planar_cloud.pcd",(string)"object_cloud.pcd");

			toROSMsg(*planarCloud, topub);
			planar_pub_.publish(topub);

			tf::Vector3 min, max;
			minmax3d(min, max, planarCloud);
			pass_through_gen(pcdFiltered,objectCloud,true,min.x(),max.x()-0.05,
					true,min.y()+0.05,max.y()-0.05,true, max.z(), BB_MAX.z());
			toROSMsg(*objectCloud, *objectCloud2);

			/*********** Voxel grid downsampling *********/
			std::cerr << "Object Point Cloud before filtering: " << objectCloud2->width * objectCloud2->height
					<< " data points (" << pcl::getFieldsList (*objectCloud2) << ").";
			// Create the filtering object
			pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
			sor.setInputCloud (objectCloud2);
			sor.setLeafSize (0.005f, 0.005f, 0.005f);
			sor.filter (*filteredObjectCloud2);
			std::cerr << "PointCloud after filtering: " << filteredObjectCloud2->width * filteredObjectCloud2->height
					<< " data points (" << pcl::getFieldsList (*filteredObjectCloud2) << ").";

			object_pub_.publish(*filteredObjectCloud2);

			//Create a plan request
			tum_os::PlanRequest plan_request;
			plan_request.object_cloud = *filteredObjectCloud2;
			plan_request.table_height = max.z();
			plan_request.bb_min.resize(3);
			plan_request.bb_min[0] = min.x()+0.05;
			plan_request.bb_min[1] = min.y()+0.05;			//0.1; temp hack to not use left arm
			plan_request.bb_min[2] = max.z();
			plan_request.bb_max.resize(3);
			plan_request.bb_max[0] = max.x()-0.05;
			plan_request.bb_max[1] = max.y()-0.05;
			plan_request.bb_max[2] = BB_MAX.z();
			planRequestPub_.publish(plan_request);
			ROS_INFO("bb_min: %f, %f, %f     bb_max: %f, %f, %f", min.x(), min.y(), max.z(),
					max.x(), max.y(), BB_MAX.z());
		}
	};//End of function getObjectCloud
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_scan_reader");
	ros::NodeHandle n;
	LaserScanReader lsr(n);

	ros::spin();
	return 0;
}

/*
  bool getNewDataCallback(tum_os::Get_New_PCD::Request &req,
                   tum_os::Get_New_PCD::Response &res)
  	{
  		ROS_INFO("Inside new data wanted callback.");
  		new_data_wanted_ = req.new_pcd_wanted;
  		ROS_INFO("Set new_data_wanted_ to true.");
  		res.result = true;
  		return true;
  	}
}; //End of class definition
 */
