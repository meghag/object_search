/*
 * laser_assembler_server.cpp
 *
 *  Created on: Jun 12, 2010
 *      Author: potthast
 */

#include "laser_assembler_server.h"


LaserAssemblerServer::LaserAssemblerServer(ros::NodeHandle& n)
{
  node_handle_ = n;
  cloud_server_ = node_handle_.advertiseService("laser_assembler_server/single_sweep_cloud", &LaserAssemblerServer::buildSingleSweepCloud, this);
  sub_ = node_handle_.subscribe("laser_tilt_controller/laser_scanner_signal", 40, &LaserAssemblerServer::scannerSignalCallback, this);
}

bool LaserAssemblerServer::buildSingleSweepCloud(pr2_arm_navigation_perception::BuildCloudAngle::Request &req,
                                          pr2_arm_navigation_perception::BuildCloudAngle::Response &res)
{
  laser_time_ = ros::Time().fromSec(0);

  // send command to tilt laser scanner
  pr2_msgs::SetPeriodicCmd::Request scan_req;
  pr2_msgs::SetPeriodicCmd::Response scan_res;
  scan_req.command.amplitude = fabs(req.angle_end - req.angle_begin) / 2.0;
  scan_req.command.offset = (req.angle_end + req.angle_begin) / 2.0;
  scan_req.command.period = req.duration * 2.0;
  scan_req.command.profile = "linear";
  if (!ros::service::call("laser_tilt_controller/set_periodic_cmd", scan_req, scan_res))
    ROS_ERROR("LaserAssemblerServer: error setting laser scanner periodic command");
  else
    ROS_INFO("LaserAssemblerServer: commanded tilt laser scanner with period %f, amplitude %f and offset %f",
             scan_req.command.period, scan_req.command.amplitude, scan_req.command.offset);

  // wait for signal from laser to know when scan is finished
  ros::Time begin_time = scan_res.start_time;
  ros::Duration timeout = ros::Duration().fromSec(5.0);
  while (laser_time_ < begin_time)
  {
    boost::mutex::scoped_lock laser_lock(laser_mutex_);
    if (ros::Time::now() > begin_time + ros::Duration().fromSec(req.duration) + timeout)
    {
      ROS_ERROR("LaserAssemblerServer: Timeout waiting for laser scan to come in");
      return false;
    }
    laser_lock.unlock();
    ros::Duration().fromSec(0.05).sleep();
  }
  ros::Time end_time = laser_time_;
  ROS_INFO("LaserAssemblerServer: generated point cloud from time %f to %f", begin_time.toSec(), end_time.toSec());

  // get a point cloud from the point cloud assembler
  laser_assembler::AssembleScans::Request assembler_req;
  laser_assembler::AssembleScans::Response assembler_res;
  assembler_req.begin = begin_time;
  assembler_req.end = end_time;
  if (!ros::service::call("assemble_scans", assembler_req, assembler_res))
    ROS_ERROR("LaserAssemblerServer: error receiving point cloud from point cloud assembler");
  else
    ROS_INFO("LaserAssemblerServer: received point cloud of size %i from point cloud assembler",
             (int)assembler_res.cloud.points.size());

  res.cloud = assembler_res.cloud;
  return true;
}

void LaserAssemblerServer::scannerSignalCallback(const pr2_msgs::LaserScannerSignalConstPtr& laser_scanner_signal)
{
  boost::mutex::scoped_lock laser_lock(laser_mutex_);
  // note time when tilt laser is pointing down
  if (laser_scanner_signal->signal == 1)
  {
    laser_time_ = laser_scanner_signal->header.stamp;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_assembler_server");
  ros::NodeHandle n;
  LaserAssemblerServer las(n);
  //ros::Rate loop_rate(5);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

 // ros::spin();

  return 0;
}
