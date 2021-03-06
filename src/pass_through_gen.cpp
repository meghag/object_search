#include "pass_through_gen.h"

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

int pass_through_gen(pcl::PointCloud<PointT>::Ptr& pcd_orig,
                 pcl::PointCloud<PointT>::Ptr& pcd_filtered,
                 bool filterx, float xmin, float xmax, bool filtery, float ymin, float ymax,
                 bool filterz, float zmin, float zmax)
{
	pcl::PointCloud<PointT>::Ptr cloudx(new pcl::PointCloud<PointT>);	//Filtered cloud
	pcl::PointCloud<PointT>::Ptr cloudy(new pcl::PointCloud<PointT>);	//Filtered cloud

	ROS_INFO("Inside pass through.");
	ROS_INFO("pass thru before filtering: %zu data points", pcd_orig->points.size());
//	for (size_t i = 0; i < 10; ++i)
//		ROS_INFO("\t%f\t%f\t%f",pcd_orig->points[i].x, pcd_orig->points[i].y, pcd_orig->points[i].z);

	/****************** Filter out the non-table points ******************/
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (pcd_orig);
	if (filterx) {
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (xmin, xmax);
		//  pass.setFilterLimitsNegative (true);
		pass.filter (*pcd_filtered);
	} else
		pcd_filtered = pcd_orig;
	if (filtery) {
		pass.setInputCloud (pcd_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (ymin, ymax);
		//  pass.setFilterLimitsNegative (true);
		pass.filter (*pcd_filtered);
	} else
		pcd_filtered = pcd_orig;
	if (filterz) {
		pass.setInputCloud (pcd_filtered);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (zmin, zmax);
		//  pass.setFilterLimitsNegative (true);
		pass.filter (*pcd_filtered);
	}

	ROS_INFO("Filtered cloud size: %zu", pcd_filtered->points.size());

  return (0);
}
