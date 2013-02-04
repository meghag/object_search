#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

geometry_msgs::Point find_centroid(pcl::PointCloud<PointT> pcd);

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<PointT> pcd);

char find_color(pcl::PointCloud<PointXYZRGB> pcd);

char find_color(sensor_msgs::PointCloud2 pcd2);

void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

sensor_msgs::PointCloud2 concatClouds(vector<sensor_msgs::PointCloud2>& clouds);

void cluster(sensor_msgs::PointCloud2& cloud2,
		float cluster_tolerance,
		int min_cluster_size,
		int max_cluster_size,
		vector<sensor_msgs::PointCloud2>& clusters);

bool touchesTable(pcl::PointCloud<PointXYZRGB> cloud, double table_height);
