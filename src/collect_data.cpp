/*********************************************
This node subscribes to the topic defined by 'cloud_topic' and reads in a
point cloud. It then transforms it to base link and extracts out the object
cloud by pass-through filtering and planar segmentation.
The object cloud is saved as a pcd file called 'object_cloud.pcd'

Date created: July 22, 2012
Author: Megha Gupta

***********************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

//std::string cloud_topic = "/head_mount_kinect/depth_registered/points";
std::string cloud_topic = "/narrow_stereo_textured/points2";

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

using namespace std;
using namespace pcl;

int pass_through_gen(pcl::PointCloud<PointT>::Ptr& pcd_orig,
		     pcl::PointCloud<PointT>::Ptr& pcd_filtered,
		     bool filterx, float xmin, float xmax,
		     bool filtery,float ymin, float ymax,
		     bool filterz, float zmin, float zmax);

int planar_seg(pcl::PointCloud<PointT>::Ptr orig_cloud,
	       pcl::PointCloud<PointT>::Ptr p_cloud,
	       pcl::PointCloud<PointT>::Ptr o_cloud,
	       string fname1, string fname2);

class CollectData
{
public:
  CollectData(ros::NodeHandle & n): n_(n), tf_(), target_frame_("base_link"), new_data_wanted_(true)
  {
    input_pub_ = n_.advertise<sensor_msgs::PointCloud2>("input_cloud",1);
    planar_pub_ = n_.advertise<sensor_msgs::PointCloud2>("planar_cloud",1);
    object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("object_cloud",1);

    pcd_sub_.subscribe(n_, cloud_topic, 1);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pcd_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&CollectData::msgCallback, this, _1) );
  } ;

private:
  ros::NodeHandle n_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcd_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
  std::string target_frame_;
  ros::Publisher input_pub_;
  ros::Publisher planar_pub_;
  ros::Publisher object_pub_;
  bool new_data_wanted_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& pcd2)
  {
    if (new_data_wanted_) {
    sensor_msgs::PointCloud2 pcd2_transformed;
    sensor_msgs::PointCloud pcd1, pcd1_transformed;
    sensor_msgs::convertPointCloud2ToPointCloud(*pcd2,pcd1);
    try
    {
      tf_.transformPointCloud(target_frame_, pcd1, pcd1_transformed);
      sensor_msgs::convertPointCloudToPointCloud2(pcd1_transformed,pcd2_transformed);
      input_pub_.publish(pcd2_transformed);

      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      fromROSMsg(pcd2_transformed, *cloud);

      ROS_INFO("Frame_id after transformation: %s", pcd2_transformed.header.frame_id.c_str());
      getObjectCloud(cloud);
    }
    catch (tf::TransformException &ex)
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
    }
  };//End of callback function

  void getObjectCloud(pcl::PointCloud<PointT>::Ptr cloud)
  {
    pcl::PointCloud<PointT>::Ptr planarCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr objectCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr pcdFiltered(new pcl::PointCloud<PointT>);	//Filtered cloud

    sensor_msgs::PointCloud2 topub;

    /****************** Filter out the non-table points ******************/
    //pass_through_gen(cloud,pcdFiltered,true,0,1,true,-0.32,0.32,true,1.012,1.3);   // Kinect and shelf
    //pass_through_gen(cloud,pcdFiltered,true,0,1,true,-0.32,0.32,true,1.07,1.3);    // Narrow stereo and shelf
    pass_through_gen(cloud,pcdFiltered,true,0,1.4,true,-0.4,0.4,true,0.81,1.1);      // Narrow stereo and simulation table
    if (pcdFiltered->points.size() > 0) {
      new_data_wanted_ = false;
      toROSMsg(*pcdFiltered, topub);
      input_pub_.publish(topub);

      /* ********* Segmentation of the cloud into table and object clouds **********/
      planar_seg(pcdFiltered,planarCloud,objectCloud,(string)"planar_cloud.pcd",(string)"object_cloud.pcd");

      toROSMsg(*planarCloud, topub);
      planar_pub_.publish(topub);

      toROSMsg(*objectCloud, topub);
      object_pub_.publish(topub);
    }
  };//End of function getObjectCloud

}; //End of class definition


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "collect_data"); //Init ROS
  ros::NodeHandle n;
  CollectData cd(n); //Construct class
  ros::spin(); // Run until interrupted
};
