/*
 * laser_assembler_server.h
 *
 *  Created on: Jun 12, 2010
 *      Author: potthast
 */

#ifndef LASER_ASSEMBLER_SERVER_H_
#define LASER_ASSEMBLER_SERVER_H_

#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans.h"
#include "pr2_arm_navigation_perception/BuildCloudAngle.h"
#include "pr2_msgs/SetPeriodicCmd.h"

// Messages
#include "sensor_msgs/PointCloud.h"
#include "pr2_msgs/LaserScannerSignal.h"

#include <boost/thread/mutex.hpp>

class LaserAssemblerServer
{
private:
  ros::Time laser_time_;
  boost::mutex laser_mutex_;
  ros::ServiceServer cloud_server_;
  ros::Subscriber sub_;
  ros::NodeHandle node_handle_;

public:
  LaserAssemblerServer(ros::NodeHandle& n);
  bool buildSingleSweepCloud(pr2_arm_navigation_perception::BuildCloudAngle::Request &req,
                             pr2_arm_navigation_perception::BuildCloudAngle::Response &res);
  void scannerSignalCallback(const pr2_msgs::LaserScannerSignalConstPtr& laser_scanner_signal);
};

#endif /* LASER_ASSEMBLER_SERVER_H_ */
