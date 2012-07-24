#ifndef PASS_THROUGH_GEN_H
#define PASS_THROUGH_GEN_H

#include <ros/ros.h>
//#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//using namespace std;
using namespace pcl;

int pass_through_gen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcd_orig,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcd_filtered,
                     bool filterx, float xmin, float xmax, bool filtery, float ymin, float ymax,
                     bool filterz, float zmin, float zmax);

#endif
