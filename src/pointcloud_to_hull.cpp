#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
//#include <tf_conversions/tf_eigen.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <visualization_msgs/Marker.h>

#include <tum_os/Clusters.h>

#include "../include/robot_arm.h"
#include "../include/van_der_corput.h"
#include "../include/grasp_planning.h"

extern "C" {
#include <gpcl/gpc.h>
}

#include <collision_testing.h>


#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>

bool data_from_bag = false;
bool data_to_bag = false;
std::string data_bag_name = "data.bag";


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mc_graspable/FindGraspablesAction.h>

#include <stdlib.h>

void start()
{
    int i = system("rosservice call gazebo/unpause_physics");
    std::cout << "system " << i << std::endl;
}

void finish()
{
    int i = system("rosservice call gazebo/pause_physics");
    std::cout << "system " << i << std::endl;
    exit(0);
}


int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues);

std::string fixed_frame_ = "tum_os_table";
//std::string fixed_frame_ = "head_mount_kinect_ir_link";//"map";
std::string mount_frame_ = "head_mount_link";
std::string rgb_optical_frame_ = "head_mount_kinect_ir_link";
std::string rgb_topic_ = "/head_mount_kinect/depth_registered/points";

std::string ik_frame_ = "/torso_lift_link";


#include <tf/transform_broadcaster.h>
tf::TransformBroadcaster *br = 0;


template <class T>
void pubCloud(const std::string &topic_name, const T &cloud, std::string frame_id = fixed_frame_);
//void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string frame_id = fixed_frame_);


void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( *cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

actionlib::SimpleActionClient<mc_graspable::FindGraspablesAction> *mcg_client_ = NULL;

void getGrasps(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<tf::Pose> &low, std::vector<tf::Pose> &high)
{

    if (!mcg_client_)
    {
        mcg_client_ = new actionlib::SimpleActionClient<mc_graspable::FindGraspablesAction> ("mc_graspable", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        mcg_client_->waitForServer(); //will wait for infinite time
    }

    tf::Vector3 pmin,pmax;
    minmax3d(pmin,pmax,cloud);

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    mc_graspable::FindGraspablesGoal goal;

    goal.cloud_topic = "/mc_graspable_query";

    goal.frame_id = "/map";

    goal.aabb_min.x = pmin.x();
    goal.aabb_min.y = pmin.y();
    goal.aabb_min.z = pmin.z();
    goal.aabb_max.x = pmax.x();
    goal.aabb_max.y = pmax.y();
    goal.aabb_max.z = pmax.z();
    goal.delta = 0.02;
    goal.scaling = 20;
    goal.pitch_limit = 0.4;
    goal.thickness = 0.04;
    mcg_client_->sendGoal(goal);



    ros::Rate rt(5);
    for (size_t k =0; k < 10; k ++)
    {
        std::cout << "Publishing cluster on " << goal.cloud_topic << std::endl;
        pubCloud(goal.cloud_topic, cloud, "/map");
        ros::spinOnce();
        rt.sleep();
    }


    //wait for the action to return
    bool finished_before_timeout = mcg_client_->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state =mcg_client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());

        mc_graspable::FindGraspablesResultConstPtr result = mcg_client_->getResult();
        for (std::vector<geometry_msgs::Pose>::const_iterator it = result->high.poses.begin(); it != result->high.poses.end(); ++it)
        {
            tf::Pose act;
            tf::poseMsgToTF(*it, act);
            high.push_back(act);
        }
        for (std::vector<geometry_msgs::Pose>::const_iterator it = result->low.poses.begin(); it != result->low.poses.end(); ++it)
        {
            tf::Pose act;
            tf::poseMsgToTF(*it, act);
            low.push_back(act);
        }

    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //return 0;
}

bool inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax)
{
    if ((point.x() > bbmin.x()) && (point.x() < bbmax.x()) &&
            (point.y() > bbmin.y()) && (point.y() < bbmax.y()) &&
            (point.z() > bbmin.z()) && (point.z() < bbmax.z()))
        return true;
    else
        return false;
}

tf::TransformListener*listener_ = 0L;

ros::Publisher *vis_pub_ = 0L;

ros::NodeHandle *nh_ = 0L;

ros::Publisher *pose_ary_pub_ = 0L;


double dist_to_sensor = 1;


void init()
{
    if (!listener_)
        nh_ = new ros::NodeHandle();
    if (!listener_)
        listener_ = new tf::TransformListener();
    if (!vis_pub_)
    {
        vis_pub_ = new ros::Publisher();
        *vis_pub_ = nh_->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    }
    if (!br)
        br = new tf::TransformBroadcaster();
}

gpc_polygon last;
bool have_last = false;

void pubPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = rgb_optical_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    gpc_polygon subject, result;
    subject.num_contours = 1;
    subject.hole = new int[1];
    subject.contour = new gpc_vertex_list[1];
    subject.contour[0].num_vertices = cloud_hull->points.size();
    subject.contour[0].vertex = new gpc_vertex[cloud_hull->points.size()];
    for (size_t i = 0; i < cloud_hull->points.size(); ++i)
    {
        subject.contour[0].vertex[i].x = cloud_hull->points[i].y * 10;
        subject.contour[0].vertex[i].y = cloud_hull->points[i].z * 10;
    }

    gpc_tristrip tristrip;

    if (have_last)
    {
        gpc_polygon_clip(GPC_INT, &subject, &last, &result);
        gpc_polygon_to_tristrip(&result,&tristrip);
    }
    else
    {
        gpc_polygon_to_tristrip(&subject,&tristrip);
    }

    have_last = true;
    last = subject;

    for (int i = 0; i < tristrip.num_strips; ++i)
    {
        for (int j = 3; j < tristrip.strip[i].num_vertices; ++j)
        {
            //std::cout << "tri" << tristrip.strip[i].vertex[j-2].x << " " << tristrip.strip[i].vertex[j-2].y << std::endl;
            //std::cout << "   " << tristrip.strip[i].vertex[j-1].x << " " << tristrip.strip[i].vertex[j-1].y << std::endl;
            //std::cout << "   " << tristrip.strip[i].vertex[j-0].x << " " << tristrip.strip[i].vertex[j-0].y << std::endl;
            geometry_msgs::Point pt[3];
            if (j % 2 == 0)
            {
                pt[0].x = dist_to_sensor;
                pt[0].y = tristrip.strip[i].vertex[j-2].x;
                pt[0].z = tristrip.strip[i].vertex[j-2].y;
                pt[1].x = dist_to_sensor;
                pt[1].y = tristrip.strip[i].vertex[j-1].x;
                pt[1].z = tristrip.strip[i].vertex[j-1].y;
                pt[2].x = dist_to_sensor;
                pt[2].y = tristrip.strip[i].vertex[j-0].x;
                pt[2].z = tristrip.strip[i].vertex[j-0].y;
            }
            else
            {
                pt[0].x = dist_to_sensor;
                pt[0].y = tristrip.strip[i].vertex[j-1].x;
                pt[0].z = tristrip.strip[i].vertex[j-1].y;
                pt[1].x = dist_to_sensor;
                pt[1].y = tristrip.strip[i].vertex[j-2].x;
                pt[1].z = tristrip.strip[i].vertex[j-2].y;
                pt[2].x = dist_to_sensor;
                pt[2].y = tristrip.strip[i].vertex[j-0].x;
                pt[2].z = tristrip.strip[i].vertex[j-0].y;
            }
            marker.points.push_back(pt[0]);
            marker.points.push_back(pt[1]);
            marker.points.push_back(pt[2]);
            std_msgs::ColorRGBA color;
            color.r = .7;
            color.g = .2;
            color.b = .1;
            color.a = .7;
            marker.colors.push_back(color);
            marker.colors.push_back(color);
            marker.colors.push_back(color);
            //std::cout << "tri" << tristrip.strip[i].vertex[j].x << " " << tristrip.strip[i].vertex[j].y << std::endl;
        }
    }

    vis_pub_->publish( marker );

}


tf::Stamped<tf::Pose> getPoseIn(const std::string target_frame, tf::Stamped<tf::Pose>src)
{

    if (src.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
    {
        ROS_ERROR("Frame not in TF: %s", src.frame_id_.c_str());
        tf::Stamped<tf::Pose> pose;
        return pose;
    }

    if (!listener_)
        listener_ = new tf::TransformListener(ros::Duration(30));

    tf::Stamped<tf::Pose> transform;
    //this shouldnt be here TODO
    //src.stamp_ = ros::Time(0);

    listener_->waitForTransform(src.frame_id_, target_frame,
                                ros::Time(0), ros::Duration(30.0));
    bool transformOk = false;
    while (!transformOk)
    {
        try
        {
            transformOk = true;
            listener_->transformPose(target_frame, src, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getPoseIn %s",ex.what());
            // dirty:
            src.stamp_ = ros::Time(0);
            transformOk = false;
        }
        ros::spinOnce();
    }
    return transform;
}

tf::Stamped<tf::Pose> getPose(const std::string target_frame,const std::string lookup_frame, ros::Time tm = ros::Time(0))
{

    init();
    ros::Rate rate(50.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    bool had_to_retry = false;

    //listener_->waitForTransform(target_frame, lookup_frame, tm, ros::Duration(0.5));
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(target_frame, lookup_frame,tm, transform);
        }
        catch (tf::TransformException ex)
        {
            std::string what = ex.what();
            what.resize(100);
            ROS_INFO("getPose: tf::TransformException ex.what()='%s', will retry",what.c_str());
            transformOk = false;
            had_to_retry = true;
            //listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
        }
        if (!transformOk)
            rate.sleep();
    }

    if (had_to_retry)
        ROS_INFO("Retry sucessful");

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

void getCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string frame_id, ros::Time after, ros::Time *tm = 0)
{

    sensor_msgs::PointCloud2 pc;
    bool found = false;
    while (!found)
    {
        pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(rgb_topic_));
        if ((after == ros::Time(0,0)) || (pc.header.stamp > after))
            found = true;
        else
        {
            //ROS_ERROR("getKinectCloudXYZ cloud too old : stamp %f , target time %f",pc.header.stamp.toSec(), after.toSec());
        }
    }
    if (tm)
        *tm = pc.header.stamp;

    tf::Stamped<tf::Pose> net_stamped = getPose(fixed_frame_.c_str(),pc.header.frame_id.c_str());
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud(frame_id.c_str(),net_transform,pc,pct);
    pct.header.frame_id = frame_id.c_str();

    geometry_msgs::Transform t_msg;
    tf::transformTFToMsg(net_transform, t_msg);

    //std::cout << "CLOUD transf " << pc.header.frame_id << " to " << pct.header.frame_id << " : " << t_msg << std::endl;

    pcl::fromROSMsg(pct, *cloud);
}

std::map<std::string, ros::Publisher*> cloud_publishers;

//void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string frame_id = "/map")
template <class T>
void pubCloud(const std::string &topic_name, const T &cloud, std::string frame_id)
{
    ros::Publisher *cloud_pub;

    if (cloud_publishers.find(topic_name) == cloud_publishers.end())
    {

        cloud_pub = new ros::Publisher();
        *cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>(topic_name,0,true);

        cloud_publishers.insert(std::pair<std::string, ros::Publisher*>(topic_name, cloud_pub ));
        //std::cout << "created new publisher" << cloud_pub << std::endl;
    }
    else
    {
        cloud_pub = cloud_publishers.find(topic_name)->second;
        //std::cout << "found pub on " << cloud_pub->getTopic() << ", reusing it" << std::endl;
    }

    sensor_msgs::PointCloud2 out; //in map frame

    pcl::toROSMsg(*cloud,out);
    out.header.frame_id = frame_id;
    out.header.stamp = ros::Time::now();
    cloud_pub->publish(out);

    //ROS_INFO("published frame %s %i x %i points on %s", out.header.frame_id.c_str(), out.height, out.width, topic_name.c_str());
}


void pubCloudWait(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string frame_id, ros::Duration waitTime)
{
    waitTime.sleep();
    return pubCloud(topic_name,cloud,frame_id);
}


void getPointsInBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inBox, const tf::Vector3 min, const tf::Vector3 max)
{
    Eigen::Vector4f min_pt, max_pt;

    min_pt = Eigen::Vector4f(std::min(min.x(), max.x()),std::min(min.y(), max.y()),std::min(min.z(), max.z()), 1);
    max_pt = Eigen::Vector4f(std::max(min.x(), max.x()),std::max(min.y(), max.y()),std::max(min.z(), max.z()), 1);

    //ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
    //ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);

    //ROS_INFO("cloud size : %zu", cloud->points.size());

    boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );

    pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

    //std::cout << "idx size" << indices->size() << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*inBox);

    pubCloud("debug_cloud", inBox);

    ROS_INFO("cloud size after box filtering: %zu = %i * %i", inBox->points.size(), inBox->width, inBox->height);
}


//orthogonal projection
void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected)
//(tf::Vector3 planeNormal, double planeDist,
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // construct a plane parallel to the camera sensor
    tf::Stamped<tf::Pose> planePoint;
    // should work with three points but strangely doesnt
    int numpoints = 10;
    for (int i = 0; i < numpoints ; ++i )
    {
        planePoint.frame_id_ = rgb_optical_frame_;
        //planePoint.frame_id_ = fixed_frame_;
        planePoint.stamp_ = ros::Time(0);
        //planePoint.setOrigin(tf::Vector3(dist_to_sensor,sin(i*(360 / numpoints )* M_PI / 180.0f),cos(i*(360 / numpoints )* M_PI / 180.0f)));
        planePoint.setOrigin(tf::Vector3(dist_to_sensor,sin(i*(360 / numpoints )* M_PI / 180.0f),cos(i*(360 / numpoints )* M_PI / 180.0f)));
        //planePoint.setOrigin(tf::Vector3(sin(i*(360 / numpoints )* M_PI / 180.0f),cos(i*(360 / numpoints )* M_PI / 180.0f),1.0));
        planePoint.setRotation(tf::Quaternion(0,0,0,1));
        planePoint = getPoseIn(fixed_frame_.c_str(), planePoint);

        pcl::PointXYZ planePt;
        planePt.x = planePoint.getOrigin().x();
        planePt.y = planePoint.getOrigin().y();
        planePt.z = planePoint.getOrigin().z();

        //std::cout << i<< planePt.x << " " << planePt.y << " " << planePt.z << std::endl;
        plane_cloud->points.push_back(planePt);
        inliers->indices.push_back(i);
    }

    //we abuse pcl plane model
    //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (plane_cloud);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
              << inliers->indices.size () << " inliers." << std::endl;


    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->points.size () << " data points." << std::endl;

    pubCloud("cloud_projected", cloud_projected);

}

void calcHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull)
{

    // Create a Concave Hull representation of the projected inliers
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);

    tf::Stamped<tf::Pose> net_stamped = getPose(rgb_optical_frame_.c_str(),fixed_frame_.c_str());
    tf::Transform fixed_to_sensor;
    fixed_to_sensor.setOrigin(net_stamped.getOrigin());
    fixed_to_sensor.setRotation(net_stamped.getRotation());

    net_stamped = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());
    tf::Transform sensor_to_fixed;
    sensor_to_fixed.setOrigin(net_stamped.getOrigin());
    sensor_to_fixed.setRotation(net_stamped.getRotation());

    // transforming point cloud back to sensor frame to get chull to recognize that its 2d
    // this will not be needed anmore with groove
    pcl_ros::transformPointCloud(*cloud,*cloud,fixed_to_sensor);

    for (size_t i = 0; i < cloud->points.size(); i++)
        cloud->points[i].x = 0;

    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.setAlpha (.01);
    chull.setAlpha (.1);
    chull.setKeepInformation(true);
    chull.reconstruct (*cloud_hull);

    std::cerr << "Concave hull " << chull.getDim() << " has: " << cloud_hull->points.size ()
              << " data points." << std::endl;

    if (chull.getDim() > 2)
        ROS_ERROR("CONCAVE HULL IS 3D!");

    pubPolygon(cloud_hull);

    for (size_t  i = 0; i < cloud_hull->points.size(); i++)
        cloud_hull->points[i].x = dist_to_sensor;

    pcl_ros::transformPointCloud(*cloud_hull,*cloud_hull,sensor_to_fixed);

    pubCloud("cloud_hull", cloud_hull);

    //pcl::PCDWriter writer;
    //writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
}


void test_hull_calc()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box_projected_hull (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in_box->width = 0;
    cloud_in_box->height = 0;

    ros::Time lookup_time;

    getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1), &lookup_time);

    {
        tf::Vector3 bb_min(-1.9,1.6,.84);
        tf::Vector3 bb_max(-1.6,2.1,1.2);

        getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);

        projectToPlane(cloud_in_box, cloud_in_box_projected);

        calcHull(cloud_in_box_projected, cloud_in_box_projected_hull);
    }

    {
        tf::Vector3 bb_min(-1.9,1.6,.75);
        tf::Vector3 bb_max(-1.6,2.1,.84);

        getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);

        projectToPlane(cloud_in_box, cloud_in_box_projected);

        calcHull(cloud_in_box_projected, cloud_in_box_projected_hull);
    }

}

#include <TableTopObject.h>

//#include <octomap_ros/OctomapROS.h>

using namespace octomap;

struct lex_compare
{
    bool operator() (const octomath::Vector3& lhs, const octomath::Vector3& rhs) const
    {
        if (lhs.x() < rhs.x())
            return true;
        if (lhs.y() < rhs.y())
            return true;
        if (lhs.z() < rhs.z())
            return true;
        return false;
    }
};

void insert_pointcloud(OcTreeROS *octoMap,tf::Point sensor_origin, pcl::PointCloud<pcl::PointXYZ>::Ptr inBox)
{
    geometry_msgs::Point point;
    tf::pointTFToMsg(sensor_origin, point);

    octoMap->insertScan(*inBox, point );
}


std::map<std::string, ros::Publisher*> belief_publishers;

void pub_belief(const std::string &topic_name,const std::vector<tf::Pose> poses, std::string frame_id = fixed_frame_)
{

    ros::Publisher *pose_ary_pub;

    if (belief_publishers.find(topic_name) == belief_publishers.end())
    {

        pose_ary_pub = new ros::Publisher();

        *pose_ary_pub = nh_->advertise<geometry_msgs::PoseArray>(topic_name,0,true);

        belief_publishers.insert(std::pair<std::string, ros::Publisher*>(topic_name, pose_ary_pub ));
        //std::cout << "created new publisher" << cloud_pub << std::endl;
    }
    else
    {
        pose_ary_pub = belief_publishers.find(topic_name)->second;
        //std::cout << "found pub on " << cloud_pub->getTopic() << ", reusing it" << std::endl;
    }

    geometry_msgs::PoseArray ps_ary;
    ps_ary.header.frame_id = frame_id;

    for (std::vector<tf::Pose>::const_iterator it = poses.begin(); it!=poses.end(); it++)
    {
        geometry_msgs::Pose pose_msg;
        tf::poseTFToMsg(*it, pose_msg);
        ps_ary.poses.push_back(pose_msg);
    }

    pose_ary_pub->publish(ps_ary);

}


boost::mutex transforms_from_planning_response_mutex;
std::vector<tf::StampedTransform> transforms_from_planning_response; // for tf publishing


bool compare_push(Push i,Push j)
{
    if (i.num_removed == j.num_removed)
        return (i.object_motion.length() > j.object_motion.length());
    else
        return (i.num_removed > j.num_removed);
}


double planning_precision = 0.01;

void generate_valid_pushes(std::vector<tf::Pose> &object_posterior_belief,
                           int max_idx,
                           int arm,
                           tf::Stamped<tf::Pose> &fixed_to_ik,
                           tf::Stamped<tf::Transform> &tum_os_table_to_odom_combined,
                           TableTopObject &obj,
                           TableTopObject &full_environment,
                           CollisionTesting &ct_full_env,
                           std::vector<CollisionTesting> &ct_obj_excluding,
                           TableTopObject &table,
                           std::vector<TableTopObject*> obj_only,
                           std::vector<Push> *pushes = 0L)
{
    GraspPlanning grasp;

    tf::Transform push_transform;
    push_transform.setOrigin(tf::Vector3(0,0,0));
    push_transform.setRotation(tf::Quaternion(0,0,0,1));

    size_t max_remaining = 0;
    for (std::vector<tf::Pose>::iterator jjt = object_posterior_belief.begin(); jjt!=object_posterior_belief.end(); jjt++)
    {
        if (obj.checkCoveredPointcloud(*jjt,push_transform,*obj_only[max_idx]))
            max_remaining++;
    }

    ROS_ERROR("MAX REMAINING CLUSTER %i gives %zu", max_idx, max_remaining);

    //!check for the best object to remove, if we have one

    //int arm =  1; // right arm
    std::cout << "Getting grasps for cluster #" << max_idx << std::endl;
    std::vector<tf::Pose> ik_checked,random,low,high,checked,filtered, reachable, collision_free;

    tf::Vector3 cluster_min, cluster_max;

    /*minmax3d(cluster_min, cluster_max, clusters[max_idx]);*/
    minmax3d(cluster_min, cluster_max, obj_only[max_idx]->cloud);
    //cluster_min -= tf::Vector3(.2,.2,.2);
    //cluster_max += tf::Vector3(.2,.2,.2);

    //!dirty setup dependent code TODO ERROR! only works in current sim environment
    cluster_min -= tf::Vector3(.2,.0,0);
    cluster_max += tf::Vector3(.2,.0,0);

    //! Dangerous, we're using tf now for a test live
    fixed_to_ik = getPose(fixed_frame_, ik_frame_);

    geometry_msgs::PoseStamped ps;
    tf::poseStampedTFToMsg(fixed_to_ik, ps);
    std::cout << "FIXED TO IK " << ps << std::endl;

    //finish();

    tf::Stamped<tf::Pose> odom_to_torso = getPose("odom_combined", ik_frame_);

    //! num grasps per cluster and arm
    for (int k =0; k < 2500; k++)
    {
        tf::Pose act = VanDerCorput::vdc_pose_bound(cluster_min,cluster_max,k);
        //act.setRotation(tf::Quaternion((k % 1 == 0) ? 0.65 : -.65,0,0,.65));
        //act.setRotation(tf::Quaternion( 0.0445028, 0.57906,0.089325, 0.809154));
        //act.getRotation() = act.getRotation().normalize();
        //std::cout << "Z " << act.getOrigin().z() << std::endl;
        //!dirty setup dependent code TODO ERROR! only works in current sim environment
        //act.getOrigin().setZ(.1);
        act.setRotation(tf::createQuaternionFromRPY(M_PI / 2.0f,0, k / 100.0f));
        //act.setRotation(tf::Quaternion( 0,0,0,1));
        random.push_back(act);
    }

    pub_belief("checked_grasps",random,fixed_frame_);

    //finish();

    ROS_INFO("tick");
    // should have points of cluster we want to grasp inside

    ROS_INFO("BEFORE CHECKING GRASPS");
    grasp.checkGraspsIK(arm,fixed_to_ik,random,ik_checked);
    ROS_INFO("checked for reachability, %zu reachable grasp candidates", ik_checked.size());
    grasp.checkGrasps(obj_only[max_idx]->cloud,ik_checked,filtered);

    std::cout << "number of filtered grasps : " << filtered.size() << " out of " << random.size() << std::endl;

    // check against general collision
    std::vector<tf::Vector3> normals, centers;
    std::vector<int> grasp_indices;
    grasp.checkGrasps(full_environment.cloud,filtered,reachable,&grasp_indices,&normals,&centers);

    ROS_INFO("AFTER CHECKING GRASPS");

    std::cout << "number of reachable grasps : " << reachable.size() << " out of " << ik_checked.size() << std::endl;

    pub_belief("checked_grasps",checked);

    pub_belief("reachable_grasps",reachable);

    /*
    {

        tf::Transform *relative_transform = &tum_os_table_to_odom_combined;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr odom_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < full_environment.cloud->points.size(); ++i)
        {
            tf::Vector3 tf_pt(full_environment.cloud->points[i].x,full_environment.cloud->points[i].y,full_environment.cloud->points[i].z);
            tf::Transform trans;
            trans.setOrigin(tf_pt);
            trans.setRotation(tf::Quaternion(0,0,0,1));

            if (relative_transform)
            {
                trans = *relative_transform * trans;
                tf_pt = trans.getOrigin();
            }

            pcl::PointXYZRGB pt;
            pt.x = tf_pt.x();
            pt.y = tf_pt.y();
            pt.z = tf_pt.z();
            pt.r = 0;
            pt.g = 0;
            pt.b = 1;

            odom_cloud->points.push_back(pt);

            //odom_cloud->points[i].x = tf_pt.x();
            //odom_cloud->points[i].y = tf_pt.y();
            //odom_cloud->points[i].z = tf_pt.z();

        }

        pubCloud("odom_cloud", odom_cloud, "odom_combined");
        pubCloud("tumos_cloud", full_environment.cloud, "tum_os_table");

    }*/


    tf::Transform wristy;
    wristy.setOrigin(tf::Vector3(0.18,0,0));
    wristy.setRotation(tf::Quaternion(0,0,0,1));

    std::vector<double> result;
    result.resize(7);
    std::fill( result.begin(), result.end(), 0 );

    std::vector<double> result_push;
    result_push.resize(7);
    std::fill( result_push.begin(), result_push.end(), 0 );

    int lowest_idx = -1;
    double lowest_z = 1000;
    //! checking cycle

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_normalvis(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<double> collision_free_pushfactor;
    //while (ros::ok())
    {

        collision_free.clear();

        //size_t min_remaining = object_posterior_belief.size() + 1;

        //for (std::vector<tf::Pose>::iterator it = reachable.begin(); (it!=reachable.end()) && ros::ok(); ++it)
        for (size_t sit = 0 ; sit < reachable.size() ; sit++)
        {
            tf::Pose *it = &reachable[sit];

            tf::Transform rel = grasp.grasps[grasp_indices[sit]].approach[0];
            //std::cout << "tick" << std::endl;
            // get this from grip model

            tf::Pose in_ik_frame = fixed_to_ik.inverseTimes(*it);

            tf::Pose in_ik_frame_push = in_ik_frame * rel;

            if ((!ct_full_env.inCollision(arm, in_ik_frame)) && (!ct_obj_excluding[max_idx].inCollision(arm,in_ik_frame_push)))
            {

                tf::Stamped<tf::Pose> actPose, approach, push;
                actPose.setData(in_ik_frame * wristy);
                actPose.frame_id_ = ik_frame_;
                //actPose = getPoseIn("base_link",actPose);
                //printf("\nbin/ias_drawer_executive -2 %i %f %f %f %f %f %f %f\n", 0 ,actPose.getOrigin().x(), actPose.getOrigin().y(), actPose.getOrigin().z(), actPose.getRotation().x(), actPose.getRotation().y(), actPose.getRotation().z(), actPose.getRotation().w());
                approach = getPoseIn("torso_lift_link",actPose);

                actPose.setData(in_ik_frame * wristy * rel);
                actPose.frame_id_ = ik_frame_;
                //actPose = getPoseIn("base_link",actPose);
                //printf("bin/ias_drawer_executive -2 %i %f %f %f %f %f %f %f\n", 0 ,actPose.getOrigin().x(), actPose.getOrigin().y(), actPose.getOrigin().z(), actPose.getRotation().x(), actPose.getRotation().y(), actPose.getRotation().z(), actPose.getRotation().w());
                push = getPoseIn("torso_lift_link",actPose);
                std::cout << push.getOrigin().z() << std::endl;

                if (push.getOrigin().z() < lowest_z)
                {
                    lowest_idx = collision_free.size();
                    lowest_z = push.getOrigin().z() ;
                }

                tf::Transform normal_pose;
                normal_pose.setOrigin(normals[sit]);
                normal_pose.setRotation(tf::Quaternion(0,0,0,1));
                normal_pose = fixed_to_ik.inverseTimes(normal_pose);
                tf::Vector3 normal = normal_pose.getOrigin();

                tf::Transform center_pose;
                center_pose.setOrigin(centers[sit]);
                center_pose.setRotation(tf::Quaternion(0,0,0,1));
                center_pose = fixed_to_ik.inverseTimes(center_pose);
                tf::Vector3 center = center_pose.getOrigin();

                normal = normal - center;

                //!check effect of pushing
                tf::Vector3 push_vector = push.getOrigin() - approach.getOrigin();
                std::cout << "PUSH REL idx " << grasp_indices[sit] << " vec "<< rel.getOrigin().x() << " "<< rel.getOrigin().y() << " "<< rel.getOrigin().z() << std::endl;
                std::cout << "PUSH VECTOR" << push_vector.x() << " "<< push_vector.y() << " "<< push_vector.z() << std::endl;

                std::cout << "PUSH normal                             " << normal.x() << " "<< normal.y() << " "<< normal.z() << " " << std::endl;

                double cos_angle = fabs(cos(push_vector.angle(normal)));

                std::cout << "cosangle" << cos_angle <<  " angle " << push_vector.angle(normal) << std::endl;

                // how far do we push until we touch the object ?


                double amt_step = .1; // 1 cm steps as push lenght is 10 cm
                double amt_free = 0;
                for (double amt = 0; amt <= 1.001; amt += amt_step)
                {

                    tf::Pose check_pose = in_ik_frame;
                    check_pose.getOrigin() = (in_ik_frame.getOrigin() * (1 - amt)) + (in_ik_frame_push.getOrigin() * amt);
                    bool inCo = ct_full_env.inCollision(arm,check_pose);

                    if (inCo)
                        amt = 100;
                    else
                        amt_free = amt;
                }

                //std::cout << amt << " inco " <<  (inCo ? "true " : "false ") << amt_free << std::endl;
                int cnt = 0;
                for (double amt = 0; amt <= 4; amt += amt_step * 2)
                {
                    // visualize normals
                    cnt++;
                    tf::Pose check_pose = in_ik_frame;
                    check_pose.getOrigin() = in_ik_frame.getOrigin() + push_vector * amt;

                    //(in_ik_frame.getOrigin() * (1 - amt)) + (in_ik_frame_push.getOrigin() * amt);

                    pcl::PointXYZRGB pt;
                    pt.x = check_pose.getOrigin().x();
                    pt.y = check_pose.getOrigin().y();
                    pt.z = check_pose.getOrigin().z();
                    pt.r = cos_angle * 255;
                    pt.g = 50;
                    pt.b = 50;
                    cloud_normalvis->points.push_back(pt);

                    if (cnt > 10)
                        cnt = 0;

                    if (cnt == 0)
                        for (double len = 0; len < 0.05; len+= 0.001)
                        {
                            pt.x = center.x() + normal.x() * len;
                            pt.y = center.y() + normal.y() * len;
                            pt.z = center.z() + normal.z() * len;
                            pt.r = 0;
                            pt.g = 0;
                            pt.b = 250;
                            cloud_normalvis->points.push_back(pt);
                        }

                }

                //std::cout <<  "AMT FRE" << amt_free << std::endl;

                // how far can we push the object?
                double end_step = .1;
                double end_free = 0;
                for (double end = 1; end <= 2.001; end += end_step)
                {

                    tf::Pose check_pose = in_ik_frame;
                    check_pose.getOrigin() = in_ik_frame.getOrigin() + end * (in_ik_frame_push.getOrigin() - in_ik_frame.getOrigin());

                    bool inCo = ct_obj_excluding[max_idx].inCollision(arm,check_pose);

                    //std::cout << amt << " inco " <<  (inCo ? "true " : "false ") << amt_free << std::endl;

                    if (inCo)
                        end = 100;
                    else
                        end_free = end;
                }

                //std::cout <<  "END FREE" << end_free << std::endl;
                //std::cout <<  "              gives us " << end_free - amt_free << std::endl;

                tf::Transform push_transform;
                push_transform.setOrigin(tf::Vector3(push_vector.x(),push_vector.y(),0));
                push_transform.setRotation(tf::Quaternion(0,0,0,1));

                //push_transform.setOrigin(push_transform.getOrigin() * (1-amt_free));

                push_transform.setOrigin(push_transform.getOrigin() * ( end_free - amt_free) * cos_angle);

                //std::cout << "Vector length" << push_transform.getOrigin().length() << std::endl;

                size_t num_remaining_inv = 0;
                for (std::vector<tf::Pose>::iterator jjt = object_posterior_belief.begin(); jjt!=object_posterior_belief.end(); jjt++)
                {
                    if (obj.checkCoveredPointcloud(*jjt,push_transform,*obj_only[max_idx]))
                        num_remaining_inv++;
                }
                //std::cout << "REMAINING " << num_remaining_inv << " of " << object_posterior_belief.size() << std::endl;

                //if (num_remaining_inv <= min_remaining)
                //{
                //  if (num_remaining_inv < min_remaining)
                //{
                //collision_free.clear();
                //collision_free_pushfactor.clear();
                //pushes->clear();

                //}

                //min_remaining = num_remaining_inv;
                Push current_push;
                current_push.arm = arm;
                current_push.cluster_index = max_idx;
                current_push.from = in_ik_frame;
                current_push.to = in_ik_frame;
                current_push.to.getOrigin() = in_ik_frame.getOrigin() + (end_free - 0.1) * (in_ik_frame_push.getOrigin() - in_ik_frame.getOrigin());
                current_push.grasp_index = grasp_indices[sit];
                current_push.num_removed = max_remaining - num_remaining_inv;
                current_push.object_motion = push_transform.getOrigin();
                if (current_push.num_removed > 0)
                    pushes->push_back(current_push);

                //collision_free.push_back(*it);
                //collision_free_pushfactor.push_back(end_free - amt_free);
                //}

                //ros::Duration(0.2).sleep();
                //RobotArm::getInstance(0)->move_arm_via_ik(approach);
                //RobotArm::getInstance(0)->move_arm_via_ik(push);
                //RobotArm::getInstance(0)->move_arm_via_ik(approach);
                //exit(0);
            }
        }

        std::cout << "number of collision free grasps : " << pushes->size() << " out of " << reachable.size() << std::endl;

    }

    pubCloud("normals", cloud_normalvis, "torso_lift_link");

    //finish();

    //take a random grasp for debugging
    std::sort (pushes->begin(), pushes->end(), compare_push);
}

int planStep(int arm, TableTopObject obj, std::vector<tf::Pose> apriori_belief, std::vector<tf::Pose> &object_posterior_belief, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,tf::Vector3 bb_min, tf::Vector3 bb_max, tf::Stamped<tf::Pose> fixed_to_ik, tf::Stamped<tf::Pose> sensor_in_fixed)
{
    //grasp.visualizeGrasp(0,"r_wrist_roll_link");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unknown (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr table (new pcl::PointCloud<pcl::PointXYZ>);
    for (double y = bb_min.y(); y < bb_max.y(); y += 0.01)
        for (double x = bb_min.x(); x < bb_max.x(); x += 0.01)
        {
            pcl::PointXYZ pt;
            pt.x = x;
            pt.y = y;
            pt.z = bb_min.z();
            table->points.push_back(pt);
        }


    TableTopObject table_object(sensor_in_fixed.getOrigin(), bb_min.z(), table);

    pubCloud("table", table , fixed_frame_);

    getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);

    //! get sensor to map
    //tf::Stamped<tf::Pose> sensor_in_fixed = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());

    TableTopObject full_environment(sensor_in_fixed.getOrigin(), bb_min.z(), cloud_in_box);

    pubCloud("cluster_volume", full_environment.getAsCloud() , fixed_frame_);

    tf::Transform identity;
    identity.setIdentity();

    //std::vector<tf::Pose> object_posterior_belief;
    for (std::vector<tf::Pose>::iterator it = apriori_belief.begin(); it!=apriori_belief.end(); it++)
    {
        if (obj.checkCoveredPointcloud(*it,identity,full_environment))
            object_posterior_belief.push_back(*it);
    }

    ROS_INFO("samples checked");

    std::cout << "size of object belief " << object_posterior_belief.size() << std::endl;

    pub_belief("belief_poses",object_posterior_belief);

    if (object_posterior_belief.size() == 0)
    {
        ROS_INFO("OBJECT A POSTERIORI BELIEF EMPTY");
        return -1;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
    std::vector<double> percentages;

    //! get clusters from euclidian clustering
    tum_os::Clusters clusters_msg;
    {
        boost::thread t(&pubCloudWait,"object_cloud", cloud_in_box, fixed_frame_.c_str(),ros::Duration(0.1));
        //! get the clusters
        std::cout << "Waiting for clusters." << std::endl;
        clusters_msg  = *(ros::topic::waitForMessage<tum_os::Clusters>("/clusters"));
        std::cout << "Clusters : " << clusters_msg.clusters.size() << std::endl;
        for (size_t i = 0; i < clusters_msg.clusters.size(); i++)
        {
            std::cout << "cluster " << i << clusters_msg.clusters[i].width << " * " << clusters_msg.clusters[i].height << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(clusters_msg.clusters[i], *cloud);
            clusters.push_back(cloud);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr percentage_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    //!create tabletop representation with one cluster missing at a time and check how many hypothesis it's covering
    int max_idx = -1;
    double max_perc = 0;
    std::vector<TableTopObject*> obj_excluding;
    std::vector<TableTopObject*> obj_only;
    percentages.resize(clusters.size());
    for (size_t i = 0; i < clusters.size(); i ++)
    {
        TableTopObject *act = new TableTopObject();

        TableTopObject *act_inv = new TableTopObject();
        for (size_t j = 0; j < clusters.size(); j ++)
        {
            if (j != i)
            {
                std::cout << "Cluster j size" << clusters[j]->points.size() << std::endl;
                act->addPointCloud(sensor_in_fixed.getOrigin(), bb_min.z(), clusters[j]);
            }
        }
        obj_excluding.push_back(act);

        size_t num_remaining = 0;
        for (std::vector<tf::Pose>::iterator it = object_posterior_belief.begin(); it!=object_posterior_belief.end(); it++)
        {
            if (obj.checkCoveredPointcloud(*it,identity,*act))
                num_remaining++;
        }

        std::cout << "REMAINIGN " << num_remaining << std::endl;

        {

            std::cout << "Cluster size" << clusters[i]->points.size() << std::endl;
            act_inv->addPointCloud(sensor_in_fixed.getOrigin(), bb_min.z(), clusters[i]);
            obj_only.push_back(act_inv);
            size_t num_remaining_inv = 0;
            for (std::vector<tf::Pose>::iterator it = object_posterior_belief.begin(); it!=object_posterior_belief.end(); it++)
            {
                if (obj.checkCoveredPointcloud(*it,identity,*act_inv))
                    num_remaining_inv++;
            }

            std::cout << "REMAINIGN INVERSE" << num_remaining_inv << std::endl;

        }


        double percentage = (object_posterior_belief.size() == 0 ? 1 : (object_posterior_belief.size() - num_remaining) / (double)object_posterior_belief.size());

        std::cout << "Removing Cluster " << i << " would reveal " << object_posterior_belief.size() - num_remaining << " of remaining hypotheses " <<
                  " that is "  << 100 * percentage << "%" << std::endl;


        if (percentage >  max_perc)
        {
            max_perc = percentage;
            max_idx = i;
        }

        percentages[i] = percentage;

        for (size_t j = 0; j < clusters[i]->points.size(); j++)
        {
            pcl::PointXYZRGB pt;
            pt.x = clusters[i]->points[j].x;
            pt.y = clusters[i]->points[j].y;
            pt.z = clusters[i]->points[j].z;
            pt.r = percentage * 255;
            pt.g = 0;
            pt.b = 255 - (percentage * 255);
            if (percentage == 0)
            {
                pt.b = 0;
                pt.g = 255;
            }

            percentage_cloud->points.push_back(pt);
        }
    }

    pubCloud("percentage", percentage_cloud, fixed_frame_.c_str());

    tf::Stamped<tf::Transform> tum_os_table_to_odom_combined;
    //tum_os_table_to_odom_combined.setOrigin(tf::Vector3(0.538, -0.053, 0.730));
    //tum_os_table_to_odom_combined.setRotation(tf::Quaternion(-0.005, 0.006, -0.774, 0.633));
    tum_os_table_to_odom_combined.setOrigin(tf::Vector3(0,0,0));
    tum_os_table_to_odom_combined.setRotation(tf::Quaternion(0,0,0,1));
    tum_os_table_to_odom_combined.frame_id_ = "tum_os_table";
    tum_os_table_to_odom_combined = getPoseIn("odom_combined",tum_os_table_to_odom_combined);

    //! get the pose from fixed to ik
    //tf::Stamped<tf::Pose> fixed_to_ik = getPose(fixed_frame_, ik_frame_);
    //tf::Stamped<tf::Pose> fixed_to_ik = getPose(fixed_frame_, ik_frame_);

    // full environment including shadows
    CollisionTesting ct_full_env(*nh_);
    ct_full_env.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);
    ct_full_env.setCollisionFrame("odom_combined");
    ct_full_env.addPointCloud( full_environment.getAsCloud(), planning_precision , &tum_os_table_to_odom_combined);
    ct_full_env.addPointCloud(table,planning_precision ,&tum_os_table_to_odom_combined);
    ct_full_env.updateCollisionModel();
    ct_full_env.kinematic_state->updateKinematicLinks();
    ct_full_env.publish_markers = true;

    std::vector<CollisionTesting> ct_obj_excluding;
    for (int k = 0; k < obj_excluding.size(); k++)
    {
        CollisionTesting act(*nh_);
        act.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);
        act.setCollisionFrame("odom_combined");
        act.addPointCloud( obj_excluding[k]->cloud, planning_precision , &tum_os_table_to_odom_combined);
        act.addPointCloud(table,planning_precision ,&tum_os_table_to_odom_combined);
        act.updateCollisionModel();
        act.kinematic_state->updateKinematicLinks();
        act.publish_markers = true;
        ct_obj_excluding.push_back(act);
    }

    std::vector<Push> pushes;

    for (int arm_i = 0; arm_i < 2; arm_i ++)
        for (int k = 0; k < obj_only.size(); k++)
            generate_valid_pushes(object_posterior_belief,
                                  k,
                                  arm_i ,
                                  fixed_to_ik,
                                  tum_os_table_to_odom_combined,
                                  obj,
                                  full_environment,
                                  ct_full_env,
                                  ct_obj_excluding,
                                  table_object,
                                  obj_only,
                                  &pushes);

    ROS_INFO("PUSHES SIZE %zu", pushes.size());

    for (int p =0 ; p < pushes.size(); p++)
    {
        Push &a = pushes[p];
        std::cout << "Push " << p << " removes " << a.num_removed << " arm " << a.arm << " clus " << a.cluster_index << " vec " << a.object_motion.x() << " "
                  << a.object_motion.y() << " " << a.object_motion.z() << " " << "len" << a.object_motion.length() << std::endl;
    }

    //finish();

    if (pushes.size() > 0)
    {

        bool success = false;

        for (int index = 0; (!success) && (index < pushes.size()); index++)
        {

            ROS_INFO("PUSHES INDEX %i", index);

            Push act_push = pushes[index];
            int arm = act_push.arm;

            std::vector<double> result;
            result.resize(7);
            std::fill( result.begin(), result.end(), 0 );

            std::vector<double> result_push;
            result_push.resize(7);
            std::fill( result_push.begin(), result_push.end(), 0 );

            int err = get_ik(arm, pushes[index].from, result);
            int err2 = get_ik(arm, pushes[index].to , result_push);

            ROS_INFO("ERROR CODES %i %i", err, err2);

            RobotArm::getInstance(arm)->open_gripper(0.01);

            int failure = RobotArm::getInstance(arm)->move_arm(pushes[index].from);
            if (failure == 0)
            {

                ROS_ERROR("MOVED THE ARM");
                RobotArm::getInstance(arm)->move_arm_joint(result_push,5);
                RobotArm::getInstance(arm)->open_gripper(0.02);
                RobotArm::getInstance(arm)->move_arm_joint(result,2);
                //try planning out of last position, if it fails, raise the arm to the max and then pull it back
                if (RobotArm::getInstance(arm)->home_arm() != 0)
                {
                    ROS_ERROR("Planning to home failed, trying alternative heuristic approach");
                    tf::Pose higher = pushes[index].from;
                    bool ik_good = true;
                    while (ik_good)
                    {
                        higher.getOrigin() += tf::Vector3(-.00,0,0.01);
                        ik_good = (get_ik(arm, higher, result) == 1);
                    }
                    higher.getOrigin() -= tf::Vector3(-.00,0,0.01);
                    RobotArm::getInstance(arm)->move_arm_via_ik(higher);
                }
                RobotArm::getInstance(arm)->open_gripper(0.001);
                success = true;
            }
        }

    }

    /*if (lowest_idx != -1)
    {

        int index = ((size_t)ros::Time::now().toSec()) % collision_free.size();

        std::cout <<" TIME " <<  ((size_t)ros::Time::now().toSec()) << std::endl;

        std::cout <<" GRASP INDEX " <<  grasp_indices[index] << std::endl;

        tf::Transform rel = grasp.grasps[grasp_indices[index]].approach[0];

        std::cout <<" GRASP PUSH " <<  rel.getOrigin().x() << " " <<  rel.getOrigin().y() << " " <<  rel.getOrigin().z() << " " << std::endl;

        double factor = collision_free_pushfactor[index];
        tf::Pose in_ik_frame = fixed_to_ik.inverseTimes(collision_free[index]);
        tf::Pose in_ik_frame_push = in_ik_frame * rel;

        in_ik_frame_push.getOrigin() = in_ik_frame.getOrigin() + factor * (in_ik_frame_push.getOrigin() - in_ik_frame.getOrigin());

        int err = get_ik(arm, in_ik_frame, result);
        int err2 = get_ik(arm, in_ik_frame_push, result_push);

        ROS_INFO("ERROR CODES %i %i", err, err2);

        RobotArm::getInstance(arm)->open_gripper(0.01);

        int failure = RobotArm::getInstance(arm)->move_arm(in_ik_frame);
        if (failure == 0)
        {

            ROS_ERROR("MOVED THE ARM");
            RobotArm::getInstance(arm)->move_arm_joint(result_push,5);
            RobotArm::getInstance(arm)->open_gripper(0.02);
            RobotArm::getInstance(arm)->move_arm_joint(result,2);
            RobotArm::getInstance(arm)->open_gripper(0.01);
            RobotArm::getInstance(arm)->home_arm();
            RobotArm::getInstance(arm)->open_gripper(0.001);
        }


        //tf::Stamped<tf::Pose> actPose, approach, push;
        //actPose.setData(in_ik_frame * wristy);
        //actPose.frame_id_ = ik_frame_;
        //approach = getPoseIn("torso_lift_link",actPose);

        //actPose.setData(in_ik_frame * wristy * rel);
        //actPose.frame_id_ = ik_frame_;
        //push = getPoseIn("torso_lift_link",actPose);

    }*/

    return 0;

}

//return collision_free.size();
//return 0;

/*
   //! show object point cloud at different remaining hypothetical positions
   ros::Rate rt(5);
   size_t idx = 0;
   if (0)
       while (ros::ok())
       {
           idx ++;
           if (idx == object_posterior_belief.size())
               idx = 0;

           pcl::PointCloud<pcl::PointXYZRGB>::Ptr hypo_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
           //std::cout << "k " << idx<< " size " << obj.cloud->points.size() << std::endl;
           for (size_t i = 0; i < obj.cloud->points.size(); ++i)
           {
               tf::Vector3 vec(obj.cloud->points[i].x, obj.cloud->points[i].y, obj.cloud->points[i].z);
               vec = object_posterior_belief[idx] * vec;
               pcl::PointXYZRGB pt;
               pt.x = vec.x();
               pt.y = vec.y();
               pt.z = vec.z();
               pt.r = 0;
               pt.g = 0;
               pt.b = 1;
               hypo_cloud->points.push_back(pt);
           }

           pubCloud("hypothesis", hypo_cloud , fixed_frame_.c_str());

           //rt.sleep();
       }*/


//}

void testOctomap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,tf::Stamped<tf::Pose> fixed_to_ik, tf::Stamped<tf::Pose> sensor_in_fixed)
{

    int n = 0;

    //generate object we search as a pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i < 100; i++)
    {
        tf::Pose act = VanDerCorput::vdc_pose(n++);
        act.setOrigin(tf::Vector3(0,0,0));
        tf::Pose rel;
        rel.setOrigin(tf::Vector3(.0125,0,0));
        //rel.setOrigin(tf::Vector3(.05,0,0));
        rel.setRotation(tf::Quaternion(0,0,0,1));

        act = act * rel;
        pcl::PointXYZ pt;
        pt.x = act.getOrigin().x();
        pt.y = act.getOrigin().y();
        pt.z = act.getOrigin().z();

        object_cloud->points.push_back(pt);

        //geometry_msgs::Pose pose_msg;
        //tf::poseTFToMsg(act, pose_msg);
        //std::cout << pose_msg << std::endl;
        //ps_ary.poses.push_back(pose_msg);
    }

    //generate a tabletopobject representing the object we search
    TableTopObject obj(object_cloud);

    ros::Time lookup_time;

    //! get the pointcloud
    //getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1), &lookup_time);

    ros::Time before = ros::Time::now();

    double m_res = 0.01;
    //double m_treeDepth;
    double m_probHit = 0.7;
    double m_probMiss = 0.4;
    double m_thresMin = 0.12;
    double m_thresMax = 0.97;

    OcTreeROS *m_octoMap = new OcTreeROS(m_res);
    m_octoMap->octree.setProbHit(m_probHit);
    m_octoMap->octree.setProbMiss(m_probMiss);
    m_octoMap->octree.setClampingThresMin(m_thresMin);
    m_octoMap->octree.setClampingThresMax(m_thresMax);
    //m_treeDepth = m_octoMap->octree.getTreeDepth();

    OcTreeROS *m_octoMap_b = new OcTreeROS(m_res);
    m_octoMap_b->octree.setProbHit(m_probHit);
    m_octoMap_b->octree.setProbMiss(m_probMiss);
    m_octoMap_b->octree.setClampingThresMin(m_thresMin);
    m_octoMap_b->octree.setClampingThresMax(m_thresMax);
    //m_treeDepth = m_octoMap_b->octree.getTreeDepth();

    tf::Vector3 bb_min(0,0,0.01);
    tf::Vector3 bb_max(.75,.4,.4);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;


    ROS_INFO("before creating samples");
    std::vector<tf::Pose> apriori_belief;
    std::vector<tf::Pose> aposteriori_belief;
    for (int k =0; k < 10000; k++)
    {
        apriori_belief.push_back(VanDerCorput::vdc_pose_bound(bb_min,bb_max,k));
    }
    ROS_INFO("samples created");

    int arm = 0;

    while (ros::ok())
    {

        if (!data_from_bag)
            getCloud(cloud, fixed_frame_, ros::Time::now());

        planStep(arm, obj, apriori_belief, aposteriori_belief, cloud, bb_min, bb_max, fixed_to_ik, sensor_in_fixed);

        if (arm == 0)
            arm = 1;
        else
            arm = 0;

        ROS_ERROR("PRE %zu POST %zu", apriori_belief.size(), aposteriori_belief.size());

        if (aposteriori_belief.size() == 0)
            finish();

        apriori_belief = aposteriori_belief;
        aposteriori_belief.clear();

        RobotArm::reset_arms();

    }

}


tf::Stamped<tf::Pose> fixed_to_ik;
tf::StampedTransform sensor_in_ik;

bool map_to_odom_set = false;
tf::Stamped<tf::Pose> map_to_odom;


sensor_msgs::JointState joint_state;

void tf_broadcaster_thread()
{
    ros::Publisher *joint_state_pub = new ros::Publisher();
    *joint_state_pub = nh_->advertise<sensor_msgs::JointState>( "joint_states", 1 );

    ros::Rate rt(2);
    while (ros::ok())
    {

        transforms_from_planning_response_mutex.lock();
        for (std::vector<tf::StampedTransform>::iterator it = transforms_from_planning_response.begin(); it!=transforms_from_planning_response.end(); ++it)
        {
            it->stamp_ = ros::Time::now();
            br->sendTransform(*it);
        }
        transforms_from_planning_response_mutex.unlock();
        rt.sleep();

        joint_state.header.stamp = ros::Time::now();
        joint_state_pub->publish(joint_state);
        //sensor_in_ik.stamp_ = ros::Time::now();
        //br->sendTransform(sensor_in_ik);
        rt.sleep();
    }
}


int main(int argc,char **argv)
{

    ros::init(argc, argv, "pointcloud_to_hull");

    init();

    start();

    std::cout << argc << std::endl;

    if ((argc>1) && (atoi(argv[1])==1))
    {
        std::cout << "from bag" << std::endl;
        data_from_bag = true;
    }

    if ((argc>1) && (atoi(argv[1])==2))
    {
        data_to_bag = true;
        std::cout << "to bag" << std::endl;
    }

    if (argc>2)
    {
        data_bag_name = argv[2];
    }


    // test arm motion

    if ((argc>1) && (atoi(argv[1])==0))
    {
        RobotArm::reset_arms();
        exit(0);
    }

    ros::Rate rt(1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //tf::Stamped<tf::Pose> fixed_to_ik;
    tf::Stamped<tf::Pose> sensor_in_fixed;


    if (!data_from_bag)
    {
        //getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1));
        getCloud(cloud, fixed_frame_, ros::Time::now());
        fixed_to_ik = getPose(fixed_frame_, ik_frame_);
        sensor_in_fixed = getPose(fixed_frame_,rgb_optical_frame_);

        if (data_to_bag)
        {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud,cloud_msg);
            geometry_msgs::PoseStamped fixed_to_ik_msg;
            tf::poseStampedTFToMsg(fixed_to_ik,fixed_to_ik_msg);
            geometry_msgs::PoseStamped sensor_in_fixed_msg;
            tf::poseStampedTFToMsg(sensor_in_fixed,sensor_in_fixed_msg);
            rosbag::Bag bag;
            bag.open(data_bag_name, rosbag::bagmode::Write);
            bag.write("/cloud", ros::Time::now(), cloud_msg);
            bag.write("/fixed_to_ik", ros::Time::now(), fixed_to_ik_msg);
            bag.write("/sensor_in_map", ros::Time::now(), sensor_in_fixed_msg);
            bag.close();
        }

    }
    else
    {

        rosbag::Bag bag;
        bag.open(data_bag_name, rosbag::bagmode::Read);

        rosbag::View view(bag);

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            if (m.getTopic() == "/cloud")
            {
                sensor_msgs::PointCloud2 ::ConstPtr msg_in = m.instantiate<sensor_msgs::PointCloud2 >();
                pcl::fromROSMsg(*msg_in, *cloud);
            }
            if (m.getTopic() == "/fixed_to_ik")
            {
                geometry_msgs::PoseStamped::ConstPtr msg_in = m.instantiate<geometry_msgs::PoseStamped>();
                tf::poseStampedMsgToTF(*msg_in, fixed_to_ik);
            }
            if (m.getTopic() == "/sensor_in_map")
            {
                geometry_msgs::PoseStamped::ConstPtr msg_in = m.instantiate<geometry_msgs::PoseStamped>();
                tf::poseStampedMsgToTF(*msg_in, sensor_in_fixed);
            }
        }

        fixed_to_ik.setOrigin(tf::Vector3(0.364, -0.676, 0.341));
        fixed_to_ik.setRotation(tf::Quaternion(0.000, -0.001, 0.751, 0.660));
        //base_footprint to torso_lift_link

        sensor_in_ik.setData(fixed_to_ik.inverseTimes(sensor_in_fixed));
        sensor_in_ik.child_frame_id_= rgb_optical_frame_;
        sensor_in_ik.frame_id_ = fixed_frame_;

        //br->sendTransform(tf::StampedTransform(fixed_to_ik, ros::Time::now(), ik_frame_, fixed_frame_));

        bag.close();

        {
            CollisionTesting collision_testing(*nh_);
            collision_testing.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);

            //exit(0);

            //arm_navigation_msgs::PlanningScene ps = collision_testing.planning_scene_res.planning_scene;

            std::cout << "Planning scene :" << collision_testing.planning_scene << std::endl;

            //std::cout << "Joint State : " << collision_testing.planning_scene_res.planning_scene.robot_state.joint_state.name.size() << std::endl;
            //for (size_t i = 0; i < collision_testing.planning_scene_res.planning_scene.robot_state.joint_state.name.size(); ++i)
            //  std::cout << "Joint: " << collision_testing.planning_scene_res.planning_scene.robot_state.joint_state.name[i] << std::endl;

            joint_state = collision_testing.planning_scene.robot_state.joint_state;

            //size_t hit = -1;

            for (size_t i = 0; i < collision_testing.planning_scene.fixed_frame_transforms.size(); ++i)
            {
                std::cout << "fixed frame " << collision_testing.planning_scene.fixed_frame_transforms[i] << std::endl;

                tf::StampedTransform act;
                tf::transformStampedMsgToTF(collision_testing.planning_scene.fixed_frame_transforms[i], act);

            }

            //for (size_t j = 0; j < collision_testing.fixed_frame_transforms.size(); j++)
            //transforms_from_planning_response.push_back(collision_testing.fixed_frame_transforms[j]);

            collision_testing.updateCollisionModel();

            //tf::Transform root = collision_testing.kinematic_state->getRootTransform();

        }

        boost::thread ty(&tf_broadcaster_thread);

    }



    testOctomap(cloud, fixed_to_ik, sensor_in_fixed);

    finish();

}
