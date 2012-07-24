/*
 *  planar_seg.h
 *  
 *
 *  Created by Megha Gupta on 12/20/11.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef PLANAR_SEG_H
#define PLANAR_SEG_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int planar_seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_cloud, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud,
               std::string fname1, std::string fname2);

#endif
