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

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/tf.h>

#include "pcd_utils.h"

//#include <tum_os/PlanRequest.h>
#include <tum_os/Get_New_PCD.h>
#include <tum_os/PlanService.h>


std::string cloud_topic = "/head_mount_kinect/depth_registered/points";
// std::string cloud_topic = "/narrow_stereo_textured/points2";

//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;

using namespace std;
using namespace pcl;

class ReadData
{
public:
  ReadData(ros::NodeHandle & n, char* bb_min, char* bb_max): n_(n), tf_(), target_frame_("base_link"), new_data_wanted_(true)
  {
    input_pub_ = n_.advertise<sensor_msgs::PointCloud2>("input_cloud",1);
    planar_pub_ = n_.advertise<sensor_msgs::PointCloud2>("planar_cloud",1);
    object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("object_cloud_from_cam",1);
    //planRequestPub_ = n_.advertise<tum_os::PlanRequest>("plan_request",1);
    planRequestClient_ = n_.serviceClient<tum_os::PlanService>("/plan");
    newpcdServer_ = n_.advertiseService("get_new_pcd", &ReadData::getNewDataCallback, this);

    pcd_sub_.subscribe(n_, cloud_topic, 1);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pcd_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&ReadData::msgCallback, this, _1) );

    BB_MIN = tf::Vector3(0.3,-0.6,atof(bb_min));
    BB_MAX = tf::Vector3(1.0,0.6,atof(bb_max));
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
  //ros::Publisher planRequestPub_;
  ros::ServiceClient planRequestClient_;
  ros::ServiceServer newpcdServer_;
  bool new_data_wanted_;
  tf::Vector3 BB_MIN;	//(0.4,-0.6,0.9);
  tf::Vector3 BB_MAX;	//(1.5,0.6,1.3);

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
      new_data_wanted_ = false;
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
    sensor_msgs::PointCloud2::Ptr objectCloud2 (new sensor_msgs::PointCloud2 ());
    sensor_msgs::PointCloud2::Ptr filteredObjectCloud2 (new sensor_msgs::PointCloud2 ());

    sensor_msgs::PointCloud2 topub;

    /****************** Filter out the non-table points ******************/
    //pass_through_gen(cloud,pcdFiltered,true,0,1,true,-0.32,0.32,true,1.012,1.3);   // Kinect and shelf
    //pass_through_gen(cloud,pcdFiltered,true,0,1,true,-0.32,0.32,true,1.07,1.3);    // Narrow stereo and shelf
    //pass_through_gen(cloud,pcdFiltered,true,0,1.4,true,-1.0,1.0,true,0.81,1.1);      // Narrow stereo and simulation table
    pass_through_gen(cloud,pcdFiltered,true,BB_MIN.x(),BB_MAX.x(),true,BB_MIN.y(),BB_MAX.y(),true,BB_MIN.z(),BB_MAX.z()-0.05);      // Narrow stereo and simulation table
    if (pcdFiltered->points.size() == 0) {
    	new_data_wanted_ = true;
    	return;
    }

      toROSMsg(*pcdFiltered, topub);
      for (int c = 0; c < 100; c++)
    	  input_pub_.publish(topub);

      /* ********* Segmentation of the cloud into table and object clouds **********/
      planar_seg(pcdFiltered,planarCloud,objectCloud,(string)"planar_cloud.pcd",(string)"object_cloud.pcd");

      toROSMsg(*planarCloud, topub);
      for (int c = 0; c < 100; c++)
    	  planar_pub_.publish(topub);

      tf::Vector3 min, max;
      minmax3d(min, max, planarCloud);
      pass_through_gen(pcdFiltered,objectCloud,true,min.x(),max.x()-0.05,true,min.y()+0.05,max.y()-0.05,true, max.z()+0.01, BB_MAX.z());
      toROSMsg(*objectCloud, *objectCloud2);

      if (pcdFiltered->points.size() == 0) {
    	  new_data_wanted_ = true;
    	  return;
      }

      /*********** Voxel grid downsampling *********/
      std::cerr << "Object Point Cloud before filtering: " << objectCloud2->width * objectCloud2->height
    		  << " data points (" << pcl::getFieldsList (*objectCloud2) << ").";
      // Create the filtering object
      pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
      sor.setInputCloud (objectCloud2);
      sor.setLeafSize (0.005f, 0.005f, 0.005f);
      sor.filter (*filteredObjectCloud2);
      std::cerr << "PointCloud after filtering: " << filteredObjectCloud2->width * filteredObjectCloud2->height
    		  << " data points (" << pcl::getFieldsList (*filteredObjectCloud2) << ")." << std::endl;

      for (int c = 0; c < 100; c++)
    	  object_pub_.publish(*filteredObjectCloud2);

      //Create a plan request
      tum_os::PlanService plan_srv;
      plan_srv.request.object_cloud = *filteredObjectCloud2;
      plan_srv.request.table_height = max.z()+0.01;
      plan_srv.request.bb_min.resize(3);
      plan_srv.request.bb_min[0] = min.x();
      plan_srv.request.bb_min[1] = min.y();			//0.1; temp hack to not use left arm
      plan_srv.request.bb_min[2] = max.z();
      plan_srv.request.bb_max.resize(3);
      plan_srv.request.bb_max[0] = max.x();
      plan_srv.request.bb_max[1] = max.y();
      plan_srv.request.bb_max[2] = BB_MAX.z();
      //planRequestPub_.publish(plan_request);
      ROS_INFO("Calling planning service.");
      if (!planRequestClient_.call(plan_srv.request, plan_srv.response)) {
    	  ROS_ERROR("Failed to call planning service");
    	  new_data_wanted_ = true;
    	  return;
    	  //throw CollisionMapException("Failed to call cluster bounding box client");
      } else if (plan_srv.response.result) {
    	  new_data_wanted_ = true;
    	  ros::Duration(2.0).sleep();
      }
      ROS_INFO("bb_min: %f, %f, %f     bb_max: %f, %f, %f", min.x(), min.y(), max.z(),
    		  max.x(), max.y(), BB_MAX.z());
  };//End of function getObjectCloud

  bool getNewDataCallback(tum_os::Get_New_PCD::Request &req,
                   tum_os::Get_New_PCD::Response &res)
  	{
  		ROS_INFO("Inside new data wanted callback.");
  		new_data_wanted_ = req.new_pcd_wanted;
  		ROS_INFO("Set new_data_wanted_ to true.");
  		res.result = true;
  		return true;
  	}
}; //End of class definition


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "read_data"); //Init ROS

	char* bb_min, *bb_max;
	/*if (argc == 1) {
		bb_min = "1.0";
		bb_max = "1.3";
	} else {*/
		for (int i = 1; i < argc; i++) {
			if (i + 1 != argc) {// Check that we haven't finished parsing already
				if (string(argv[i]) == "-bbmin")
					bb_min = argv[i + 1];
				else if (string(argv[i]) == "-bbmax")
					bb_max = argv[i + 1];
			}
		}
	//}

	ros::NodeHandle n;
	ReadData cd(n, bb_min, bb_max); //Construct class
	ros::spin(); // Run until interrupted
};