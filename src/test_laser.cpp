/*
 * test_laser.cpp
 *
 *  Created on: Jun 10, 2010
 *      Author: Christian Potthast
 */

//#include "nbv_utilities/point_cloud.h"
#include <ros/ros.h>

// Service
#include <pr2_arm_navigation_perception/BuildCloudAngle.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <tf/transform_listener.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/transforms.h"

#include "sensor_msgs/point_cloud_conversion.h"


class TestLaser
{

private:
	ros::NodeHandle node_handle_;
	//ros::ServiceServer point_cloud_srv_;
	//ros::ServiceServer point_cloud_;
	//ros::ServiceServer sendCloud_;


public:
	ros::Publisher scan_pub_;
	sensor_msgs::PointCloud cloud;
	sensor_msgs::PointCloud2 cloud2;

	//PointCloud(ros::NodeHandle& n);
	//bool broadcastCloud(nbv_utilities::PcdFile::Request &req, nbv_utilities::PcdFile::Response &res);
	//bool broadcastPointCloud(nbv_utilities::PcdFile::Request &req, nbv_utilities::PcdFile::Response &res);
	//bool sendCloud(nbv_utilities::PcdFile::Request &req,
	//              nbv_utilities::PcdFile::Response &res);
	//bool scan();
	//bool stopTilt();

	TestLaser(ros::NodeHandle& n)
	{
		node_handle_ = n;
		//point_cloud_srv_ = node_handle_.advertiseService("nbv_utilities/PointCloud", &PointCloud::broadcastCloud, this);
		//  point_cloud_ = n_.advertiseService("point_cloud/cloud", &PointCloud::broadcastPointCloud, this);
		//sendCloud_ = node_handle_.advertiseService("nbv_utilities/SendCloud", &PointCloud::sendCloud, this);
		scan_pub_ = node_handle_.advertise<sensor_msgs::PointCloud>("laser_cloud",1);

		scan();
	}

	bool scan()
	{
		pr2_arm_navigation_perception::BuildCloudAngle::Request snap_req;
		pr2_arm_navigation_perception::BuildCloudAngle::Response snap_res;

		snap_req.angle_begin = -0.6;			//angle above the XY plane
		snap_req.angle_end = 0.5;				//angle below the XY plane
		snap_req.duration = 4.0;

		if (!ros::service::call("laser_assembler_server/single_sweep_cloud", snap_req, snap_res)){
			ROS_ERROR("PointCloud: error setting laser snapshotter service");
			return false;

		}else{
			ROS_INFO("PointCloud : laser snapshotter with angle_begin %f, angle_end %f and duration %f",
					snap_req.angle_begin, snap_req.angle_end, snap_req.duration);
		}

		ROS_INFO("PointCloud: received point cloud of size %i from point cloud assembler", (int)snap_res.cloud.points.size());

		stopTilt();

		cloud = snap_res.cloud;
		scan_pub_.publish(cloud);

		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
		pcl::fromROSMsg(cloud2, pcl_cloud);

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
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_laser");
	ros::NodeHandle n;
	TestLaser testlaser(n);

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	return 0;
}
