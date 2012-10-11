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

extern "C" {
#include <gpcl/gpc.h>
}

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mc_graspable/FindGraspablesAction.h>

void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id = "/map");

void getGrasps()
{

    actionlib::SimpleActionClient<mc_graspable::FindGraspablesAction> ac("mc_graspable", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    mc_graspable::FindGraspablesGoal goal;

    goal.cloud_topic = "/head_mount_kinect/depth_registered/points";

    goal.frame_id = "/map";
    goal.aabb_min.x = -2.1;//atof(argv[1]);
    goal.aabb_min.y = 1.54;//atof(argv[2]);
    goal.aabb_min.z = .5;
    goal.aabb_max.x = -1.59;//atof(argv[3]);
    goal.aabb_max.y = 2,2;//atof(argv[4]);
    goal.aabb_max.z = 1.5;
    goal.delta = 0.02;
    goal.scaling = 20;
    goal.pitch_limit = 0.4;
    goal.thickness = 0.04;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");


    //return 0;
}

void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( 	*cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

actionlib::SimpleActionClient<mc_graspable::FindGraspablesAction> *mcg_client_ = NULL;

void getGrasps(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{

    if (!mcg_client_) {
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
    for (size_t k =0; k < 10; k ++) {
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
    }
    else
        ROS_INFO("Action did not finish before the time out.");


    //return 0;
}




std::string fixed_frame_ = "map";
//std::string fixed_frame_ = "head_mount_kinect_ir_link";//"map";
std::string mount_frame_ = "head_mount_link";
std::string rgb_optical_frame_ = "head_mount_kinect_ir_link";
std::string rgb_topic_ = "/head_mount_kinect/depth_registered/points";

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
    if (!pose_ary_pub_)
    {
        pose_ary_pub_ = new ros::Publisher();
        *pose_ary_pub_ = nh_->advertise<geometry_msgs::PoseArray>("vdc_poses",0,true);
    }
}

gpc_polygon last;
bool have_last = false;

void pubPolygon(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull)
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
    ros::Rate rate(100.0);

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


void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id, ros::Time after, ros::Time *tm)
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

//void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id = "/map")
void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id)
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

void getPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox, const tf::Vector3 min, const tf::Vector3 max)
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

    pcl::ExtractIndices<pcl::PointXYZRGB> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*inBox);

    pubCloud("debug_cloud", inBox);

    ROS_INFO("cloud size after box filtering: %zu = %i * %i", inBox->points.size(), inBox->width, inBox->height);
}


//orthogonal projection
void projectToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_projected)
//(tf::Vector3 planeNormal, double planeDist,
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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

        pcl::PointXYZRGB planePt;
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
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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

    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->points.size () << " data points." << std::endl;

    pubCloud("cloud_projected", cloud_projected);

}

void calcHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hull)
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

    pcl::ConcaveHull<pcl::PointXYZRGB> chull;
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box_projected_hull (new pcl::PointCloud<pcl::PointXYZRGB>);

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


#include <cmath>
#include <iostream>

//van der corput sequence
double vdc(int n, double base = 2)
{
    double vdc = 0, denom = 1;
    while (n)
    {
        vdc += fmod(n, base) / (denom *= base);
        n /= base; // note: conversion from 'double' to 'int'
    }
    return vdc;
}

int bases[] = {2,3,5,7,11,13,17,19,23,29};

//get the nth element of a dense sequence of poses filling 0..1 in x, y, z and the SO(3) for rotation
// based on van der corput sequence with relatively prime bases
tf::Pose vdc_pose(int n)
{
    tf::Pose ret;
    ret.setOrigin(tf::Vector3(vdc(n,bases[0]),vdc(n,bases[1]),vdc(n,bases[2])));
    double u[3];
    for (int i= 0; i<3; i++)
        u[i] = vdc(n,bases[i+3]);
    double q[4];
    q[0] = sqrt(1 - u[0]) * sin(2 * M_PI * u[1]);
    q[1] = sqrt(1 - u[0]) * cos(2 * M_PI * u[1]);
    q[2] = sqrt(u[0]) * sin(2 * M_PI * u[2]);
    q[3] = sqrt(u[0]) * cos(2 * M_PI * u[2]);
    ret.setRotation(tf::Quaternion(q[0],q[1],q[2],q[3]));
    return ret;
}

tf::Pose vdc_pose_bound(tf::Vector3 min, tf::Vector3 max, int n)
{
    tf::Pose ret = vdc_pose(n);
    ret.getOrigin() = tf::Vector3( min.x() + (ret.getOrigin().x() * (max.x() - min.x())),
                                   min.y() + (ret.getOrigin().y() * (max.y() - min.y())),
                                   min.z() + (ret.getOrigin().z() * (max.z() - min.z())));
    return ret;
}

//octomap

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

void insert_pointcloud(OcTreeROS *octoMap,tf::Point sensor_origin, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox)
{
    geometry_msgs::Point point;
    tf::pointTFToMsg(sensor_origin, point);

    octoMap->insertScan(*inBox, point );
}

void pub_belief(std::vector<tf::Pose> poses)
{

    geometry_msgs::PoseArray ps_ary;
    ps_ary.header.frame_id = "/map";

    for (std::vector<tf::Pose>::iterator it = poses.begin(); it!=poses.end(); it++)
    {
        geometry_msgs::Pose pose_msg;
        tf::poseTFToMsg(*it, pose_msg);
        ps_ary.poses.push_back(pose_msg);
    }

    pose_ary_pub_->publish(ps_ary);

}


void testOctomap()//const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hull)
{


    int n = 0;

    //generate object we search as a pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < 100; i++)
    {
        tf::Pose act = vdc_pose(n++);
        act.setOrigin(tf::Vector3(0,0,0));
        tf::Pose rel;
        rel.setOrigin(tf::Vector3(.0125,0,0));
        //rel.setOrigin(tf::Vector3(.05,0,0));
        rel.setRotation(tf::Quaternion(0,0,0,1));

        act = act * rel;
        pcl::PointXYZRGB pt;
        pt.x = act.getOrigin().x();
        pt.y = act.getOrigin().y();
        pt.z = act.getOrigin().z() * 4;

        object_cloud->points.push_back(pt);

        //geometry_msgs::Pose pose_msg;
        //tf::poseTFToMsg(act, pose_msg);
        //std::cout << pose_msg << std::endl;
        //ps_ary.poses.push_back(pose_msg);
    }

    //generate a tabletopobject representing the object we search
    TableTopObject obj(object_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_unknown (new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Time lookup_time;

    getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1), &lookup_time);

    ros::Time before = ros::Time::now();

// {
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

    //tf::Vector3 bb_min(-1.9,1.6,.875);
    //tf::Vector3 bb_max(-1.6,2.1,1.2);
    //tf::Vector3 bb_min(-2.1,1.6,.875);
    tf::Vector3 bb_min(-2.1,1.6,.89);
    tf::Vector3 bb_max(-1.6,2.1,1.2);

    getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);

    tf::Stamped<tf::Pose> sensorsInMap = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());

    TableTopObject myCluster(sensorsInMap.getOrigin(), bb_min.z(), cloud_in_box);

    //pubCloud("cluster_volume", myCluster.getAsCloud() , "/map");

    ROS_INFO("before creating samples");
    std::vector<tf::Pose> object_belief;
    for (int k =0; k < 1000000; k++)
    {
        object_belief.push_back(vdc_pose_bound(bb_min,bb_max,k));
    }
    ROS_INFO("samples created");

    //pub_belief(object_belief);

    tf::Transform identity;
    identity.setIdentity();

    std::vector<tf::Pose> object_posterior_belief;
    for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it!=object_belief.end(); it++)
    {
        if (obj.checkCoveredPointcloud(*it,identity,myCluster))
            object_posterior_belief.push_back(*it);
    }

    ROS_INFO("samples checked");

    std::cout << "size of object belief " << object_posterior_belief.size() << std::endl;

    pub_belief(object_posterior_belief);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters;

    tum_os::Clusters clusters_msg;
    {
        pubCloud("object_cloud", cloud_in_box, "/map");
        clusters_msg  = *(ros::topic::waitForMessage<tum_os::Clusters>("/clusters"));
        std::cout << "Clusters : " << clusters_msg.clusters.size() << std::endl;
        for (size_t i = 0; i < clusters_msg.clusters.size(); i++)
        {
            std::cout << "cluster " << i << clusters_msg.clusters[i].width << " * " << clusters_msg.clusters[i].height << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(clusters_msg.clusters[i], *cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (size_t j = 0; j < cloud->points.size(); j++)
            {
                pcl::PointXYZRGB pt;
                pt.x = cloud->points[j].x;
                pt.y = cloud->points[j].y;
                pt.z = cloud->points[j].z;
                pt.r = 0;
                pt.g = 0;
                pt.b = 0;
                cloudrgb->points.push_back(pt);
            }
            clusters.push_back(cloudrgb);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr percentage_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    //create tabletop representation with one cluster missing at a time
    int max_idx = -1;
    double max_perc = 0;
    std::vector<TableTopObject*> obj_excluding;
    for (size_t i = 0; i < clusters.size(); i ++)
    {
        TableTopObject *act = new TableTopObject();
        for (size_t j = 0; j < clusters.size(); j ++)
        {
            if (j != i)
                act->addPointCloud(sensorsInMap.getOrigin(), bb_min.z(), clusters[j]);
        }
        obj_excluding.push_back(act);
        size_t num_remaining = 0;
        for (std::vector<tf::Pose>::iterator it = object_posterior_belief.begin(); it!=object_posterior_belief.end(); it++)
        {
            if (obj.checkCoveredPointcloud(*it,identity,*act))
                num_remaining++;
        }
        std::cout << "Removing Cluster " << i << " would reveal " << object_posterior_belief.size() - num_remaining << " of remaining hypotheses " <<
                  " that is "  << 100 * (object_posterior_belief.size() - num_remaining) / object_posterior_belief.size()  << "%" << std::endl;

        double percentage = (object_posterior_belief.size() - num_remaining) / (double)object_posterior_belief.size();

        if (percentage >  max_perc) {
            max_perc = percentage;
            max_idx = i;
        }

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

    if (max_idx >= 0)
    {
        std::cout << "Getting grasps for cluster #" << max_idx << std::endl;
        getGrasps(clusters[max_idx]);
    }

    pubCloud("percentage", percentage_cloud, "/map");


    ros::Rate rt(5);
    size_t idx = 0;
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

        pubCloud("hypothesis", hypo_cloud , "/map");

        //rt.sleep();
    }

    /*
    if (0)
    for (double xadd = 0; xadd < 0.5; xadd+= 0.01)
    {
        tf::Transform ownTransform;
        tf::Transform otherTransform;

        ownTransform.setIdentity();
        otherTransform.setIdentity();

        otherTransform.getOrigin().setX(xadd);

        bool collides = myCluster.checkCollision(ownTransform, otherTransform, myCluster);

        std::cout << "Cluster are " << (collides ? "" : "not ") << " in collision at " << xadd << " cm shift" << std::endl;
    }
    */

    //tf::Stamped<tf::Pose> sensorsInMap = getPose(rgb_optical_frame_,rgb_optical_frame_);
    /*
        projectToPlanePerspective(sensorsInMap.getOrigin(), bb_min.z() ,cloud_in_box, cloud_in_box_projected);

        pubCloud("cluster", cloud_in_box, fixed_frame_);

        pubCloud("projected", cloud_in_box_projected, fixed_frame_);

        std::cout << cloud_in_box->points.size() << " to pts " << cloud_in_box_projected->points.size()  << std::endl;

        octomap::KeySet occupied_cells;

        for (size_t i = 0; i < cloud_in_box->points.size(); ++i)
        {
            octomap::KeyRay ray;
            octomath::Vector3 start;
            start.x() = cloud_in_box->points[i].x;
            start.y() = cloud_in_box->points[i].y;
            start.z() = cloud_in_box->points[i].z;
            octomath::Vector3 end;
            end.x() = cloud_in_box_projected->points[i].x;
            end.y() = cloud_in_box_projected->points[i].y;
            end.z() = cloud_in_box_projected->points[i].z;
            m_octoMap->octree.computeRayKeys (start, end, ray);
            for (octomap::KeyRay::iterator it = ray.begin(); it != ray.end(); it++)
            {
                occupied_cells.insert(*it);
            }
        }

        for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
        {
            m_octoMap->octree.updateNode(*it, true, false);
        }

        size_t numpt = 0;
        size_t numpt_not = 0;

        int depth;
        nh_->param<int>("oct_depth", depth, 16);



        for (OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(depth),
                end = m_octoMap->octree.end(); it != end; ++it)
        {
            if (m_octoMap->octree.isNodeOccupied(*it))
            {

                pcl::PointXYZRGB pt;

                pt.x = it.getX();
                pt.y = it.getY();
                pt.z = it.getZ();

                cloud_unknown->points.push_back(pt);

                numpt++;
            }
            else
            {
                numpt_not++;
            }

        }


        ros::Time after = ros::Time::now();

        std::cout << "time taken for octomap construction " << (after - before).toSec() << " s." <<std::endl;

        pubCloud("cloud_unknown", cloud_unknown, "/map");

        std::cout << "octomap filled nodes " << numpt << " = " << cloud_unknown->points.size() << " not filled " << numpt_not << std::endl;

        std::cout << "octomap volume " << m_octoMap->octree.volume() << std::endl;

        before = ros::Time::now();

        for (double xadd = 0; xadd < 0.5; xadd+= 0.01)
            for (OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(16),
                    end = m_octoMap->octree.end(); it != end; ++it)
            {


                //octomap::OcTreeNode *node = m_octoMap->octree.search(*it);
                //point3d 	keyToCoord (const OcTreeKey &key) const
                octomath::Vector3 coord;
                //m_octoMap->octree.genCoords(*it, 16, coord);
                //it->getCoords(coord);

                coord.x() = it.getX();
                coord.y() = it.getY();
                coord.z() = it.getZ();

                //todo : transform the object to its new coordinates
                coord.x() += xadd;

                octomap::OcTreeKey key;

                if (!m_octoMap->octree.genKey(coord, key))
                    continue;

                octomap::OcTreeNode *node = m_octoMap->octree.search(key);

                if (node && m_octoMap->octree.isNodeOccupied(node))
                {
                    ROS_INFO("XADD %f collision", xadd);
                    break;
                }

            }



        after = ros::Time::now();

        std::cout << " now filled " << numpt_not << std::endl;
        std::cout << "time taken for collision check" << (after - before).toSec() << " s." <<std::endl;
        */




    // lookup tests:



    /*

    // difference of unknown space in octree

    insert_pointcloud(m_octoMap, sensorsInMap.getOrigin(), cloud_in_box);

    std::list<octomath::Vector3> node_centers;
    m_octoMap->octree.getUnknownLeafCenters(node_centers, octomath::Vector3(bb_min.x(),bb_min.y(),bb_min.z()), octomath::Vector3(bb_max.x(),bb_max.y(),bb_max.z()) );
    std::cout << "a Number of free nodes: " << node_centers.size() << std::endl;

    bb_min = tf::Vector3(-1.9,1.6,.84);
    bb_max = tf::Vector3(-1.6,2.1,.86);

    //getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);
    for (float y = bb_min.y() - 2; y <= bb_max.y() + 2; y+= 0.01)
        for (float x = bb_min.x() - 2; x <= bb_max.x() + 2; x+= 0.01)
            {
                pcl::PointXYZRGB pnt;
                pnt.x = x;
                pnt.y = y;
                pnt.z = bb_min.z();
                cloud_table->points.push_back(pnt);
            }

    insert_pointcloud(m_octoMap_b, sensorsInMap.getOrigin(), cloud_table);

    std::list<octomath::Vector3> node_centers_b;

    bb_min = tf::Vector3(-1.9,1.6,.84);
    bb_max = tf::Vector3(-1.6,2.1,1.2);

    m_octoMap_b->octree.getUnknownLeafCenters(node_centers_b, octomath::Vector3(bb_min.x(),bb_min.y(),bb_min.z()), octomath::Vector3(bb_max.x(),bb_max.y(),bb_max.z()) );
    std::cout << "b Number of free nodes: " << node_centers_b.size() << std::endl;

    std::set<octomath::Vector3,lex_compare> node_centers_set;
    node_centers_set.insert(node_centers_b.begin(),node_centers_b.end());

    for (std::list<octomath::Vector3>::iterator it = node_centers.begin(); it != node_centers.end(); it++)
    //for (std::list<octomath::Vector3>::iterator it = node_centers_b.begin(); it != node_centers_b.end(); it++)
    {
        //if (node_centers_set.find(*it) != node_centers_set.end())
        {
            pcl::PointXYZRGB pnt;
            pnt.x = it->x();
            pnt.y = it->y();
            pnt.z = it->z();
            cloud_unknown->points.push_back(pnt);
        }
    }

    pubCloud("cloud_unknown", cloud_unknown);


    //( 	point3d_list &  	node_centers,		point3d  	pmin,		point3d  	pmax ) 		const
    }
    */
}


void  test_vdc()
{
    /*for (int base = 0; base < 6; ++base)
    {
        std::cout << "Base " << base << " : " << bases[base] << "\n";
        for (int n = 0; n <= bases[base] * 2 + 1 ; ++n)
        {
            std::cout << vdc(n, (float) bases[base]) << " ";
        }
        std::cout << "\n\n";
    }*/

    ros::Publisher pose_ary_pub = nh_->advertise<geometry_msgs::PoseArray>("vdc_poses",0,true);

    geometry_msgs::PoseArray ps_ary;
    ps_ary.header.frame_id = "/map";

    int n = 0;

    ros::Rate rt(5);

    while (ros::ok())
    {

        for (int i=0; i < 1000; i++)
        {
            tf::Pose act = vdc_pose(n++);
            geometry_msgs::Pose pose_msg;
            tf::poseTFToMsg(act, pose_msg);
            //std::cout << pose_msg << std::endl;
            ps_ary.poses.push_back(pose_msg);
        }

        pose_ary_pub.publish(ps_ary);

        rt.sleep();

        ros::spinOnce();
    }

}
//2 3 5 7 11 13 17

void test_search()
{

}


int main(int argc,char **argv)
{

    ros::init(argc, argv, "pointcloud_to_hull");

    init();

    //test_vdc();

    test_search();

    ros::Rate rt(5);

    while (ros::ok())
    {
        rt.sleep();
        //test_hull_calc();
        testOctomap();
    }

}
