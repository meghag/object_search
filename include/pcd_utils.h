#include <ros/ros.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace pcl;

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

int pass_through_gen(pcl::PointCloud<PointT>::Ptr& pcd_orig,
		     pcl::PointCloud<PointT>::Ptr& pcd_filtered,
		     bool filterx, float xmin, float xmax,
		     bool filtery,float ymin, float ymax,
		     bool filterz, float zmin, float zmax);

int planar_seg(pcl::PointCloud<PointT>::Ptr orig_cloud,
               pcl::PointCloud<PointT>::Ptr p_cloud,
               pcl::PointCloud<PointT>::Ptr o_cloud,
               string fname1, string fname2);

geometry_msgs::Point find_centroid(pcl::PointCloud<PointT> pcd);

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<PointT> pcd);

char find_color(pcl::PointCloud<PointT> pcd);

char find_color(sensor_msgs::PointCloud2 pcd2);

void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<PointT>::Ptr &cloud);

sensor_msgs::PointCloud2 concatClouds(vector<sensor_msgs::PointCloud2>& clouds);

void cluster(sensor_msgs::PointCloud2& cloud2,
		float cluster_tolerance,
		int min_cluster_size,
		int max_cluster_size,
		vector<sensor_msgs::PointCloud2>& clusters);

bool touchesTable(pcl::PointCloud<PointT> cloud, double table_height);
