/*********************************************
This program reads in a point cloud from the specified pcd file 
and extracts clusters out of it based on euclidean color-based clustering.
Works well for standard objects on a shelf when two objects of the same color
are not in contact.

Date created: July 24, 2012
Author: Megha Gupta
Modified: Aug 2, 2012

***********************************************/


#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include "set_marker.h"

#include <object_manipulation_msgs/FindClusterBoundingBox2.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/CollisionObject.h>

#include "tum_os/Clusters.h"


using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;
typedef std::vector<pcl::PointCloud<PointT> > pcd_vector;
typedef std::pair<geometry_msgs::PoseStamped, geometry_msgs::Vector3> bbx;

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<PointT> pcd);

visualization_msgs::Marker set_marker(std::string frame_id, string ns, int id, int shape, 
		    geometry_msgs::Point position, float scale, float r, float g, float b, float a);

geometry_msgs::Point find_centroid(pcl::PointCloud<PointT> pcd);

class PostSegmenter {

public:
  PostSegmenter(ros::NodeHandle & n):n_(n)
  {
     object_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("object_cloud",1);
     object_minz_pub_ = n_.advertise<visualization_msgs::Marker>("minz_object",1);
     clusters_minz_pub_ = n_.advertise<visualization_msgs::MarkerArray>("minz_clusters",6);
     clusters_sub_ = n_.subscribe("clusters",1, &PostSegmenter::callback, this);

     bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
  };

private:
  ros::NodeHandle n_;
  ros::Publisher object_cloud_pub_;
  ros::Publisher object_minz_pub_;
  ros::Publisher clusters_minz_pub_;
  ros::Subscriber clusters_sub_;
  ros::ServiceClient bbx_client_;

  int num_clusters_;
  std::vector<sensor_msgs::PointCloud2> clusters2_;
  sensor_msgs::PointCloud2 object_cloud2_;
  vector<bbx> bounding_boxes_;

  void callback(const tum_os::Clusters::ConstPtr& c)
  {
    num_clusters_ = c->num_clusters;
    clusters2_ = c->clusters;
    object_cloud2_ = c->object_cloud;

    object_cloud_pub_.publish(object_cloud2_);
    //    fromROSMsg(c->object_cloud, object_cloud_);

    if (num_clusters_ == 0) {
      ROS_ERROR("No visible objects.");
      return;
    }

    find_accessible_();
    //    find_bounding_boxes_();
    //add_to_collision_map_();
    
  }; //End callback function

  void find_accessible_()
  {
     vector<geometry_msgs::Point> object_centroids;
     pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
     pcl::PointCloud<PointT>::Ptr object_cloud(new pcl::PointCloud<PointT>);
     std::vector<geometry_msgs::Point> oc_extents, cluster_extents;
     object_manipulation_msgs::FindClusterBoundingBox2 bbx_srv;

     fromROSMsg(object_cloud2_, *object_cloud);
     oc_extents = find_extents(*object_cloud);
     float z_min = oc_extents[4].z;
     ROS_INFO("Min z for the whole object cloud = %f", z_min);
     object_minz_pub_.publish(set_marker("base_link","ns1",0,visualization_msgs::Marker::CUBE,
				     oc_extents[4],0.02,0.0f,0.0f,1.0f,0.0));

      /** Now that we have all clusters, find out which objects are in front. **/
     // An object is in front if: 1) it touches the bottom, 
     // 2) the centroid of cluster is midway between top and bottom
  
     // For each cluster, find a position and orientation
     // Position by centroid; Orientation by fitting a cylinder?
  
     pcd_vector visible_in_the_front, visible_in_the_back;
     visualization_msgs::MarkerArray marr;

     for (int i = 0; i < num_clusters_; i++) {
       fromROSMsg(clusters2_[i], *cluster_cloud);
       object_centroids.push_back(find_centroid(*cluster_cloud));
       cluster_extents = find_extents(*cluster_cloud);
       ROS_INFO("Cluster %d: z_min = %f", i+1, cluster_extents[4].z);
       marr.markers.push_back(set_marker("base_link","ns2",i+1,visualization_msgs::Marker::SPHERE,
					 cluster_extents[4],0.02,0.0f,1.0f,1.0f,0.0));
       if (abs(cluster_extents[4].z - z_min) < 0.01)
	 visible_in_the_front.push_back(*cluster_cloud);
       else
	 visible_in_the_back.push_back(*cluster_cloud);

       //Find a bounding box for each cluster
       bbx_srv.request.cluster = clusters2_[i];
       if (!bbx_client_.call(bbx_srv)) {
	 ROS_ERROR("Failed to call find bounding box service");
       } else {
	 if (bbx_srv.response.error_code == 1)
	   ROS_ERROR("Failed to find bounding box for cluster %d", i+1);
	 else 
	   bounding_boxes_.push_back(make_pair(bbx_srv.response.pose, bbx_srv.response.box_dims));	   
       }
     }
     clusters_minz_pub_.publish(marr);
     ROS_INFO("No. of objects in front = %zu, and in the back = %zu", 
	      visible_in_the_front.size(),visible_in_the_back.size());
    
  };//End find_accessible_ function

  void add_to_collision_map_()
  {
    arm_navigation_msgs::CollisionObject co;
    arm_navigation_msgs::PlanningScene ps;
    for (size_t i = 0; i < clusters_.size(); i++) {
      co.header = cluster.header;
      co.id = itoa(i);
      co.padding = -1;             //For zero padding
      co.operation = arm_navigation_msgs::CollisionObject::ADD;
      co.shapes.push_back(arm_navigation_msgs::CollisionObject::BOX);
      co.poses.push_back(bounding_boxes_[i].first.pose);
      
      ps.collision_objects.push_back(co);
      
    }
    //    PlanningScene
  };

 }; //End class definition



int main (int argc, char** argv)
{
  ros::init(argc, argv, "post_seg"); //Init ROS
  ros::NodeHandle n;
  PostSegmenter ps(n);
  ros::spin();
  return 0;
}


/*
  if (argc < 2) {
    ROS_ERROR("Usage: %s <number of clusters>", argv[0]);
    return (-1);
  }

   sensor_msgs::PointCloud2 to_pub;
  if (pcl::io::loadPCDFile<PointT> ("object_cloud.pcd", *object_cloud) == -1) {
    //Load the object cloud
    ROS_ERROR("Couldn't read file.");
    return (-1);
  }
  ROS_INFO("Loaded %d data points from pcd file.", object_cloud->width * object_cloud->height);
  toROSMsg(*object_cloud,to_pub);
  object_cloud_pub.publish(to_pub);
 
  num_clusters = atoi(argv[1]);
 
  stringstream ss;
    ss << "cluster_" << i << ".pcd"; 
    cout << ss.str() << endl;
    if (pcl::io::loadPCDFile<PointT> (ss.str(), *cluster_cloud) == -1) {
      //Load the cluster file
      ROS_ERROR("Couldn't read file.");
      return (-1);
    }
        if (i == 1) 
      object_cloud = *cluster_cloud;
    else
      object_cloud += *cluster_cloud;
    
    ROS_INFO("Loaded %d data points from pcd file.", cluster_cloud->width * cluster_cloud->height);
    clusters.push_back(*cluster_cloud);
*/

