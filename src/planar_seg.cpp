#include "planar_seg.h"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;

int planar_seg(pcl::PointCloud<PointT>::Ptr orig_cloud,
               pcl::PointCloud<PointT>::Ptr p_cloud,
               pcl::PointCloud<PointT>::Ptr o_cloud,
               string fname1, string fname2)
{
  /******************** Planar Segmentation ***************************/

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  //PCDWriter writer;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  seg.setInputCloud (orig_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

 // Extract the inliers
    extract.setInputCloud (orig_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*p_cloud);
    std::cerr << "PointCloud representing the planar component: " << p_cloud->width * p_cloud->height << " data points." << std::endl;


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*o_cloud);
  //pcl::io::savePCDFileASCII ((string)fname1, *p_cloud);
  //std::cerr << "Saved " << p_cloud->points.size () << " data points to " << fname1 << std::endl;

  pcl::io::savePCDFileASCII ((string)fname2, *o_cloud);
  std::cerr << "Saved " << o_cloud->points.size () << " data points to " << fname2 << std::endl;

  //for (size_t i = 0; i < cloud.points.size (); ++i)
  //  std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
