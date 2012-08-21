//#include "find_extents.h"
#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

geometry_msgs::Point find_centroid(pcl::PointCloud<PointT> pcd)
{ 
	ROS_INFO("Inside find_centroid");
	double avgx = 0.0, avgy = 0.0, avgz = 0.0;
	geometry_msgs::Point position;
	for (std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it1 = pcd.points.begin(); 
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
