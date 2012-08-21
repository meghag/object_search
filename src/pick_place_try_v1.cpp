/*********************************************
This program reads in a point cloud from the specified pcd file
and extracts all cylinders out of it iteratively. Not working well.
Only the first cylinder is created correctly.

Date created: July 31, 2012
Author: Megha Gupta
Modified: Aug 8, 2012

***********************************************/


#include <ros/ros.h>
//#include <pcl/ros/conversions.h>
//#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <actionlib/client/simple_action_client.h>
//#include <tabletop_object_detector/TabletopDetection.h>
//#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <arm_navigation_msgs/GetCollisionObjects.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>
#include "tum_os/Clusters.h"

//set service and action names
const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
const std::string COLLISION_PROCESSING_SERVICE_NAME =
    "/octomap_server";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_COLLISION_OBJECTS_NAME = "/environment_server/get_collision_objects";
const std::string DEFAULT_CLUSTER_GRASP_PLANNER = "/plan_point_cluster_grasp";
const std::string DEFAULT_DATABASE_GRASP_PLANNER = "/database_grasp_planning";

typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> PickupClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> PlaceClient;
//typedef actionlib::SimpleActionClient<

class PickPlace
{

public:
    PickPlace(ros::NodeHandle & n):
        n_(n),
        collision_object_current_id_(0) //, pickup_client_(PICKUP_ACTION_NAME, true), place_client_(PLACE_ACTION_NAME, true)
    {
        //wait for pickup server
        /*    while(!pickup_client_.waitForServer(ros::Duration(2.0)) && n_.ok())
          ROS_INFO_STREAM("Waiting for pickup server " << PICKUP_ACTION_NAME);
        if (!n_.ok()) exit(0);

        //wait for place server
        while(!place_client_.waitForServer(ros::Duration(2.0)) && n_.ok())
          ROS_INFO_STREAM("Waiting for place server " << PLACE_ACTION_NAME);
        if (!n_.ok()) exit(0);
        */

        //wait for cluster grasp planner
        /*   while(!place_client.waitForServer(ros::Duration(2.0)) && n_.ok())
          ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
        if (!n_.ok()) exit(0);


        //wait for collision map processing client
        while (!ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME,
        				 ros::Duration(30.0)) && n_.ok())
          ROS_INFO("DUPLO: Waiting for collision processing service to come up");
        if (!n_.ok()) exit(0);
        collision_processing_srv_ =
          n_.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
        	(COLLISION_PROCESSING_SERVICE_NAME, true);
        */

        collision_map_pub_ = n_.advertise<arm_navigation_msgs::CollisionMap>("my_collision_map",1);

        target_object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("target_object",1);

        collision_object_pub_ = n_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
        attached_object_pub_ = n_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);

        ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
        get_planning_scene_client_ = n_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);
        get_collision_objects_client_ = n_.serviceClient<arm_navigation_msgs::GetCollisionObjects>(GET_COLLISION_OBJECTS_NAME);

/*
        arm_navigation_msgs::GetPlanningScene planning_scene_srv;
        if (!get_planning_scene_client_.call(planning_scene_srv))
        {
            ROS_ERROR("Could not connect to planning scene server.");
        }
        else
        {
            arm_navigation_msgs::CollisionMap cm = planning_scene_srv.response.planning_scene.collision_map;
            collision_map_pub_.publish(cm);
            ROS_INFO("number of collision objects before = %zu", planning_scene_srv.response.planning_scene.collision_objects.size());
        }
        geometry_msgs::Pose pose;
        pose.position.x = .6;
        pose.position.y = -.6;
        pose.position.z = 1.5;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        std::vector<double> dims;
        dims.push_back(0.1);
        dims.push_back(1.0);
        collision_obj("sample_obj", arm_navigation_msgs::CollisionObjectOperation::ADD,
                      pose, arm_navigation_msgs::Shape::CYLINDER, dims);
*/
        bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
        clusters_sub_ = n_.subscribe("/clusters", 1, &PickPlace::callback, this);
    };

private:
    ros::NodeHandle n_;

    //create service and action clients
    // PickupClient pickup_client_;
    //PlaceClient  place_client_;
    ros::ServiceClient get_planning_scene_client_;
    ros::ServiceClient get_collision_objects_client_;

    ros::Subscriber clusters_sub_;
    std::vector<sensor_msgs::PointCloud2> clusters_;
    ros::Publisher collision_map_pub_;
    ros::Publisher target_object_pub_;
    ros::Publisher collision_object_pub_;
    ros::Publisher attached_object_pub_;

    ros::ServiceClient bbx_client_;

    int collision_object_current_id_;

    void callback(const tum_os::Clusters::ConstPtr& c)
    {
        clusters_ = c->clusters;
        target_object_pub_.publish(clusters_[0]);

        // 1. Remove all collision objects
        resetCollisionModels();

        for (size_t i = 0; i < clusters_.size(); i++)
        {
            // 2. Get bounding box of this cluster
            object_manipulation_msgs::ClusterBoundingBox bbx;
            getClusterBoundingBox(clusters_[0], bbx.pose_stamped, bbx.dimensions);
            //geometry_msgs::PoseStamped bbx_pose;
            //geometry_msgs::Vector3 bbx_dims;
            //find_bbx(clusters_[0], bbx_pose, bbx_dims);

            // 3. Add this bounding box as a collision object
            std::string collision_name;
            processCollisionGeometryForBoundingBox(bbx, collision_name);

            /*
            std::vector<double> dims;
            dims.push_back(bbx_dims.x);
            dims.push_back(bbx_dims.y);
            dims.push_back(bbx_dims.z);
            collision_obj("object1", arm_navigation_msgs::CollisionObjectOperation::ADD,
                          bbx_pose.pose, arm_navigation_msgs::Shape::BOX, dims);
            */
        }

        // 4. Create Graspable object for the target object
        object_manipulation_msgs::GraspableObject go;
        create_graspable_object(clusters_[0], "object_1", go);


        // 5. Pick up
        // 6. Place

    }

    void resetCollisionModels()
    {
        arm_navigation_msgs::CollisionObject reset_object;
        reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
        reset_object.header.frame_id = "base_link";
        reset_object.header.stamp = ros::Time::now();
        reset_object.id = "all";
        collision_object_pub_.publish(reset_object);
        //collision_object_current_id_ = 0;
    }

    void getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
                               geometry_msgs::PoseStamped &pose_stamped,
                               geometry_msgs::Vector3 &dimensions)
    {
        object_manipulation_msgs::FindClusterBoundingBox2 srv;
        srv.request.cluster = cluster;
        if (!bbx_client_.call(srv.request, srv.response))
        {
            ROS_ERROR("Failed to call cluster bounding box client");
            //throw CollisionMapException("Failed to call cluster bounding box client");
        }
        pose_stamped = srv.response.pose;
        dimensions = srv.response.box_dims;
        if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
        {
            ROS_ERROR("Cluster bounding box client returned an error (0.0 bounding box)");
            //throw CollisionMapException("Bounding box computation failed");
        }
    }

    void processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box,
            std::string &collision_name)
    {
        ROS_INFO("Adding bounding box with dimensions %f %f %f to collision map",
                 box.dimensions.x, box.dimensions.y, box.dimensions.z);

        arm_navigation_msgs::CollisionObject collision_object;
        collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

        collision_name = getNextObjectName();
        collision_object.id = collision_name;

        collision_object.header.frame_id = box.pose_stamped.header.frame_id;
        collision_object.header.stamp = ros::Time::now();

        arm_navigation_msgs::Shape shape;
        shape.type = arm_navigation_msgs::Shape::BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = box.dimensions.x;
        shape.dimensions[1] = box.dimensions.y;
        shape.dimensions[2] = box.dimensions.z;
        collision_object.shapes.push_back(shape);
        collision_object.poses.push_back(box.pose_stamped.pose);

        collision_object_pub_.publish(collision_object);
    }

    std::string getNextObjectName()
    {
        std::ostringstream iss;
        iss << collision_object_current_id_++;
        if (collision_object_current_id_ > 10000) collision_object_current_id_ = 0;
        return "object_" + iss.str();
    }

    void create_graspable_object(sensor_msgs::PointCloud2 cloud2, std::string collision_name,
                                 object_manipulation_msgs::GraspableObject& go)
    {
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud) == true)
        {
            cloud.header.frame_id = "base_link";
            cloud.header.stamp = ros::Time::now();
            go.cluster = cloud;
            go.collision_name = collision_name;
            go.reference_frame_id = "base_link";
        }
    }
};//End Class definition

    /*
      bool remove_collision_objects()
      {
        arm_navigation_msgs::GetCollisionObjects col_obj_srv;
        col_obj_srv.request.include_points = false;
        if (!get_collision_objects_client_.call(col_obj_srv)) {
          ROS_ERROR("Could not connect to collision objects server.");
          return false;
        } else {
          //ROS_INFO("Added collision object");
          std::vector<arm_navigation_msgs::CollisionObject> vco = col_obj_srv.response.collision_objects;
          ROS_INFO("number of collision objects after = %zu", vco.size());

          for (size_t i = 0; i < vco.size(); i++) {
            collision_obj(vco[i].id, 1, vco[i].poses[0], vco[0].shapes[0].type, vco[0].shapes[0].dimensions);
          }

          return true;
        }
      }


    bool collision_obj(std::string id, int operation, geometry_msgs::Pose pose,
                       int shape, std::vector<double> dims)
    {
        arm_navigation_msgs::CollisionObject co;
        co.id = id;
        co.operation.operation = operation; //0: add, 1: remove, 2: detach and add; 3: attach and remove
        //arm_navigation_msgs::CollisionObjectOperation::ADD;
        co.header.frame_id = "base_link";
        co.header.stamp = ros::Time::now();
        arm_navigation_msgs::Shape object;
        object.type = shape;  //0: sphere; 1: box; 2: cylinder; 3: mesh
        //arm_navigation_msgs::Shape::CYLINDER;
        object.dimensions = dims;
        co.shapes.push_back(object);
        co.poses.push_back(pose);

        arm_navigation_msgs::GetPlanningScene planning_scene_srv;
        planning_scene_srv.request.planning_scene_diff.collision_objects.push_back(co);
        if (!get_planning_scene_client_.call(planning_scene_srv))
        {
            ROS_ERROR("Could not connect to planning scene server.");
            return false;
        }
        else
        {
            ROS_INFO("Added collision object");
            std::vector<arm_navigation_msgs::CollisionObject> vco = planning_scene_srv.response.planning_scene.collision_objects;
            ROS_INFO("number of collision objects after = %zu", vco.size());
            return true;
        }
    }


    bool find_bbx(sensor_msgs::PointCloud2 cloud, geometry_msgs::PoseStamped& bbx_pose, geometry_msgs::Vector3 bbx_dims)
    {
        object_manipulation_msgs::FindClusterBoundingBox2 bbx_srv;
        bbx_srv.request.cluster = cloud;
        if (!bbx_client_.call(bbx_srv))
        {
            ROS_ERROR("Failed to call bounding box service.");
            return false;
        }
        else
        {
            if (bbx_srv.response.error_code == 1)
            {
                ROS_ERROR("Failed to find bounding box");
                return false;
            }
            else
            {
                bbx_pose = bbx_srv.response.pose;
                bbx_dims = bbx_srv.response.box_dims;
                return true;
            }
        }
    }
  */


int main(int argc, char **argv)
{
    //initialize the ROS node
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n;
    PickPlace pp(n);
    ros::spin();
    return(0);
}

/*
 if (argc < 3){
   ROS_ERROR("Usage: ./pick_place <scene_cloud.pcd> <target_cloud.pcd>");
   return -1;
 }
 */
