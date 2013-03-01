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
void pubCloud(const std::string &topic_name, T &cloud, std::string frame_id = fixed_frame_);

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

class BoxSet
{
public:

    //void create_marker

    std::vector<tf::Vector3> bb_min;
    std::vector<tf::Vector3> bb_max;
    std::vector<bool> bb_full;
};


void checkGrasps(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked)
{
    // min coordinates of aabb
    //std::vector<tf::Vector3> bb_min;
    // max coordinates of aabb
    //std::vector<tf::Vector3> bb_max;
    // should this bounding box be empty => true or should contain a point => false
    //std::vector<bool> bb_full;

    /*
    double xShift = .18; // distance toolframe to wrist, we work in wrist later for ik etc

    // we want to see some points centered between the grippers
    bb_min.push_back(tf::Vector3(xShift + 0.03,-0.02,-.02));
    bb_max.push_back(tf::Vector3(xShift + 0.04, 0.02, .02));
    bb_full.push_back(true);

    // we want to see some points centered between the grippers
    bb_min.push_back(tf::Vector3(xShift + 0.04,-0.02,-.02));
    bb_max.push_back(tf::Vector3(xShift + 0.05, 0.02, .02));
    bb_full.push_back(true);

    //coarsest of approximation for gripper fingers when gripper is open
    bb_min.push_back(tf::Vector3(xShift + 0.00,0.03,-.02));
    bb_max.push_back(tf::Vector3(xShift + 0.05,0.09, .02));
    bb_full.push_back(false);

    bb_min.push_back(tf::Vector3(xShift + 0.00,-0.09,-.02));
    bb_max.push_back(tf::Vector3(xShift + 0.05,-0.03, .02));
    bb_full.push_back(false);

    // we want to be able to approach from far away, so check the space we sweep when approaching and grasping
    bb_min.push_back(tf::Vector3(xShift - 0.2 ,-0.09,-.03));
    bb_max.push_back(tf::Vector3(xShift + 0.00, 0.09, .03));
    bb_full.push_back(false);
    */

    BoxSet act;

    if(0)
    {
        //push forward open grip
        act.bb_min.push_back(tf::Vector3(0.23,-0.05,-0.02));
        act.bb_max.push_back(tf::Vector3(0.24,-0.03,0.02));
        act.bb_full.push_back(true);

        act.bb_min.push_back(tf::Vector3(0.23,0.03,-0.02));
        act.bb_max.push_back(tf::Vector3(0.24,0.05,0.02));
        act.bb_full.push_back(true);

        act.bb_min.push_back(tf::Vector3(0.18,0.03,-0.02));
        act.bb_max.push_back(tf::Vector3(0.23,0.09,0.02));
        act.bb_full.push_back(false);

        act.bb_min.push_back(tf::Vector3(0.18,-0.09,-0.02));
        act.bb_max.push_back(tf::Vector3(0.23,-0.03,0.02));
        act.bb_full.push_back(false);

        act.bb_min.push_back(tf::Vector3(-0.02,-0.09,-0.03));
        act.bb_max.push_back(tf::Vector3(0.18,0.09,0.03));
        act.bb_full.push_back(false);
    }


    if (1)
    {
        //push forward closed gripper
        act.bb_min.push_back(tf::Vector3(0.23,-0.02,-0.02));
        act.bb_max.push_back(tf::Vector3(0.24,3.46945e-18,0.02));
        act.bb_full.push_back(true);

        act.bb_min.push_back(tf::Vector3(0.23,-3.46945e-18,-0.02));
        act.bb_max.push_back(tf::Vector3(0.24,0.02,0.02));
        act.bb_full.push_back(true);

        act.bb_min.push_back(tf::Vector3(0.18,-3.46945e-18,-0.02));
        act.bb_max.push_back(tf::Vector3(0.23,0.04,0.02));
        act.bb_full.push_back(false);

        act.bb_min.push_back(tf::Vector3(0.18,-0.04,-0.02));
        act.bb_max.push_back(tf::Vector3(0.23,3.46945e-18,0.02));
        act.bb_full.push_back(false);

        act.bb_min.push_back(tf::Vector3(-0.02,-0.06,-0.03));
        act.bb_max.push_back(tf::Vector3(0.18,0.06,0.03));
        act.bb_full.push_back(false);
    }


    tf::Pose push;
    push.setOrigin(tf::Vector3(0.01,0,0));
    push.setRotation(tf::Quaternion(0,0,0,1));
    //act.approach.push_back(push);

    //act.name = "push_forward";

    std::vector<size_t> bb_cnt;
    bb_cnt.resize(act.bb_min.size());

    // for each grasp
    for (std::vector<tf::Pose>::iterator it = unchecked.begin(); it!=unchecked.end(); ++it)
    {
        std::fill( bb_cnt.begin(), bb_cnt.end(), 0 );

        bool good = true;

        //for each point, points first so that we transform only once, do not transform full pointcloud as we might get lucky and hit a point early
        // and thus not need to transform all of them
        //for (int i = 0; (i < cloud->points.size()) && good; ++i)
        for (size_t i = 0; (i < cloud->points.size()) && good; ++i)
        {
            tf::Vector3 curr(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            // project point to gripper coordinates
            curr = (*it).inverse() * curr;

            // check each defined bounding box
            //for (int k = 0; (k < bb_min.size()) && good; ++k)
            for (size_t k = 0; (k < act.bb_min.size()); ++k)
            {
                if (inside(curr, act.bb_min[k], act.bb_max[k]))
                {
                    bb_cnt[k]++;
                    if (!act.bb_full[k])
                        good = false;
                }

            }
        }

        //std::cout << std::endl;
        for (size_t j = 0; j < act.bb_min.size(); j++)
        {
            if (act.bb_full[j] && (bb_cnt[j] < 10))
                good = false;
        }

        if (good)
        {
            //for (int j = 0; j < bb_min.size(); j++)
            //std::cout << "bb_cnt" << j << " : " << bb_cnt[j] << std::endl;
            checked.push_back(*it);
        }
    }

}

void checkGraspsIK(int arm, tf::Stamped<tf::Pose> fixed_to_ik, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked)
{
    std::vector<double> result;
    result.resize(7);
    std::fill( result.begin(), result.end(), 0 );

    for (std::vector<tf::Pose>::iterator it = unchecked.begin(); it!=unchecked.end(); ++it)
    {
        tf::Pose in_ik_frame = fixed_to_ik.inverseTimes(*it);
        if (get_ik(arm, in_ik_frame, result) == 1)
            checked.push_back(*it);
    }

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
    //if (!pose_ary_pub_)
    //{
    //pose_ary_pub_ = new ros::Publisher();
    //>*pose_ary_pub_ = nh_->advertise<geometry_msgs::PoseArray>("vdc_poses",0,true);
    //}
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
void pubCloud(const std::string &topic_name, T &cloud, std::string frame_id)
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

void insert_pointcloud(OcTreeROS *octoMap,tf::Point sensor_origin, pcl::PointCloud<pcl::PointXYZ>::Ptr inBox)
{
    geometry_msgs::Point point;
    tf::pointTFToMsg(sensor_origin, point);

    octoMap->insertScan(*inBox, point );
}


std::map<std::string, ros::Publisher*> belief_publishers;

void pub_belief(const std::string &topic_name,const std::vector<tf::Pose> poses)
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
    ps_ary.header.frame_id = fixed_frame_;

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


void testOctomap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,tf::Stamped<tf::Pose> fixed_to_ik, tf::Stamped<tf::Pose> sensor_in_fixed)

//void testOctomap()//const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull)
{


    int n = 0;

    //generate object we search as a pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i < 100; i++)
    {
        tf::Pose act = vdc_pose(n++);
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

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unknown (new pcl::PointCloud<pcl::PointXYZ>);

    ros::Time lookup_time;

    //! get the pointcloud
    //getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1), &lookup_time);

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
    //tf::Vector3 bb_min(-2.1,1.6,.89);
    //tf::Vector3 bb_max(-1.6,2.1,1.2);
    //tf::Vector3 bb_min(-1.3,-7.15,.745);
    //tf::Vector3 bb_max(-.82,-6.7,1.2);

    tf::Vector3 bb_min(0,0,0.03);
    tf::Vector3 bb_max(.75,.75,.4);

    getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);


    //! get sensor to map
    //tf::Stamped<tf::Pose> sensor_in_fixed = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());


    TableTopObject myCluster(sensor_in_fixed.getOrigin(), bb_min.z(), cloud_in_box);

    //pubCloud("cluster_volume", myCluster.getAsCloud() , "/map");

    ROS_INFO("before creating samples");
    std::vector<tf::Pose> object_belief;
    for (int k =0; k < 10000; k++)
    {
        object_belief.push_back(vdc_pose_bound(bb_min,bb_max,k));
    }
    ROS_INFO("samples created");

    //pub_belief(object_belief);

    // randomly calculate grasps. fun!
    if (0)
    {
        std::vector<tf::Pose> checked;
        checkGrasps(cloud_in_box,object_belief,checked);
        std::cout << "number of good looking grasps : " << checked.size() << std::endl;
        pub_belief("vdc_poses",checked);
        ros::Rate rt(10);
        while (ros::ok())
        {
            rt.sleep();
        }
    }

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

    pub_belief("vdc_poses",object_posterior_belief);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;

    tum_os::Clusters clusters_msg;
    {
        pubCloud("object_cloud", cloud_in_box, fixed_frame_.c_str());
        //! get the clusters
        std::cout << "Waiting for clusters." << std::endl;
        clusters_msg  = *(ros::topic::waitForMessage<tum_os::Clusters>("/clusters"));
        std::cout << "Clusters : " << clusters_msg.clusters.size() << std::endl;
        for (size_t i = 0; i < clusters_msg.clusters.size(); i++)
        {
            std::cout << "cluster " << i << clusters_msg.clusters[i].width << " * " << clusters_msg.clusters[i].height << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(clusters_msg.clusters[i], *cloud);
            /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZ>);
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
            */
            clusters.push_back(cloud);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr percentage_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    //!create tabletop representation with one cluster missing at a time
    int max_idx = -1;
    double max_perc = 0;
    std::vector<TableTopObject*> obj_excluding;
    for (size_t i = 0; i < clusters.size(); i ++)
    {
        TableTopObject *act = new TableTopObject();
        for (size_t j = 0; j < clusters.size(); j ++)
        {
            if (j != i)
                act->addPointCloud(sensor_in_fixed.getOrigin(), bb_min.z(), clusters[j]);
        }
        obj_excluding.push_back(act);
        size_t num_remaining = 0;
        for (std::vector<tf::Pose>::iterator it = object_posterior_belief.begin(); it!=object_posterior_belief.end(); it++)
        {
            if (obj.checkCoveredPointcloud(*it,identity,*act))
                num_remaining++;
        }

        double percentage = (object_posterior_belief.size() == 0 ? 1 : (object_posterior_belief.size() - num_remaining) / (double)object_posterior_belief.size());

        std::cout << "Removing Cluster " << i << " would reveal " << object_posterior_belief.size() - num_remaining << " of remaining hypotheses " <<
                  " that is "  << 100 * percentage << "%" << std::endl;


        if (percentage >  max_perc)
        {
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

    //! get the pose from fixed to ik
    //tf::Stamped<tf::Pose> fixed_to_ik = getPose(fixed_frame_, ik_frame_);
    //tf::Stamped<tf::Pose> fixed_to_ik = getPose(fixed_frame_, ik_frame_);

    //!check for the best object to remove, if we have one
    if (max_idx >= 0)
    {
        // init collision testing here so that we have robot model up to date
        CollisionTesting collision_testing(*nh_);
        collision_testing.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);

        transforms_from_planning_response_mutex.lock();
        transforms_from_planning_response.clear();

        for (size_t j = 0; j < collision_testing.fixed_frame_transforms.size(); j++)
            transforms_from_planning_response.push_back(collision_testing.fixed_frame_transforms[j]);

        transforms_from_planning_response_mutex.unlock();

        int arm = 0; // right arm
        std::cout << "Getting grasps for cluster #" << max_idx << std::endl;
        std::vector<tf::Pose> ik_checked,random,low,high,checked,filtered, reachable, collision_free;

        //getGrasps(clusters[max_idx], low, high);

        //checkGrasps(cloud_in_box,low,checked);
        //checkGrasps(cloud_in_box,high,checked);
        tf::Vector3 cluster_min, cluster_max;

        minmax3d(cluster_min, cluster_max, clusters[max_idx]);
        cluster_min -= tf::Vector3(.1,.1,.1);
        cluster_max += tf::Vector3(.1,.1,.1);

        //! Dangerous, we're using tf now for a test live
        fixed_to_ik = getPose(fixed_frame_, ik_frame_);

        for (int k =0; k < 100000; k++)
        {
            random.push_back(vdc_pose_bound(bb_min,bb_max,k));
        }

        ROS_INFO("tick");
        // should have points of cluster we want to grasp inside

        //check ik first
        if (1)
        {
            checkGraspsIK(arm,fixed_to_ik,random,ik_checked);
            ROS_INFO("checked for reachability, %zu reachable grasp candidates", ik_checked.size());
            checkGrasps(clusters[max_idx],ik_checked,filtered);
        }
        else
        {
            checkGrasps(clusters[max_idx],random,filtered);
        }
        //checkGrasps(clusters[max_idx],random,filtered);
        std::cout << "number of filtered grasps : " << filtered.size() << " out of " << random.size() << std::endl;
        // should not collide with other points either
        checkGrasps(cloud_in_box,filtered,checked);
        std::cout << "number of checked grasps : " << checked.size() << " out of " << filtered.size() << std::endl;
        ROS_INFO("tock");
        //checkGrasps(cloud_in_box,random,checked);

        checkGraspsIK(arm,fixed_to_ik,checked,reachable);

        std::cout << "number of reachable grasps : " << reachable.size() << " out of " << checked.size() << std::endl;

        pub_belief("checked_grasps",checked);

        pub_belief("reachable_grasps",reachable);


        size_t hit = -1;
        /*
        for (size_t i = 0; i < collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms.size(); ++i)
        {
            std::cout << "fixed frame " << collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i] << std::endl;
            if (collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].child_frame_id == "tum_os_table")
            {
                hit = i;
                std::cout << "HIT" << i << std::endl;
                geometry_msgs::Transform odom_to_tumos;
                tf::transformTFToMsg(fixed_to_ik,odom_to_tumos);
                odom_to_tumos.translation.z = -.74;

                collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].transform = odom_to_tumos;

                std::cout << "fixed frame modified " << collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i] << std::endl;
            }
            //else
            //{
                //collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].child_frame_id = "undefined";
            //}
        }
        */

        /*
        ruehr@nono:~/Dropbox/ros/tum_os$ rosrun tf tf_echo  tum_os_table odom_combined
        At time 1360074867.314
        - Translation: [0.055, -0.527, -0.738]
        - Rotation: in Quaternion [0.005, -0.006, 0.774, 0.633]
            in RPY [-0.003, -0.015, 1.772]

        odom_combined tum_os_table
        - Translation: [0.538, -0.053, 0.730]
        - Rotation: in Quaternion [-0.005, 0.006, -0.774, 0.633]
            in RPY [-0.015, -0.000, -1.772]
            */

        tf::Stamped<tf::Transform> tum_os_table_to_odom_combined;
        //tum_os_table_to_odom_combined.setOrigin(tf::Vector3(0.538, -0.053, 0.730));
        //tum_os_table_to_odom_combined.setRotation(tf::Quaternion(-0.005, 0.006, -0.774, 0.633));
        tum_os_table_to_odom_combined.setOrigin(tf::Vector3(0,0,0));
        tum_os_table_to_odom_combined.setRotation(tf::Quaternion(0,0,0,1));
        tum_os_table_to_odom_combined.frame_id_ = "tum_os_table";
        tum_os_table_to_odom_combined = getPoseIn("odom_combined",tum_os_table_to_odom_combined);
        //!TODO

        collision_testing.setCollisionFrame("odom_combined");

        {

            tf::Transform *relative_transform = &tum_os_table_to_odom_combined;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr odom_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            for (size_t i = 0; i < cloud_in_box->points.size(); ++i)
            {
                tf::Vector3 tf_pt(cloud_in_box->points[i].x,cloud_in_box->points[i].y,cloud_in_box->points[i].z);
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
            pubCloud("tumos_cloud", cloud_in_box, "tum_os_table");

        }


        //add the pointclouds of the clusters we don't want to touch to the obstacles for collision checking
        //if (0)

        for (size_t i = 0; i < clusters.size(); i ++)
        {
            if (i != max_idx)
            {
                collision_testing.addPointCloud(clusters[i], .01,&tum_os_table_to_odom_combined);
                std::cout << " adding cluster " <<  i << "to obstacles" << std::endl;
            }
        }


        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_torso (new pcl::PointCloud<pcl::PointXYZ>);

        ros::Rate rt(1);
        //tf::Stamped<tf::Pose> fixed_to_ik_variable = fixed_to_ik;


        //collision_testing.addPointCloud(cloud,0.01);
        collision_testing.updateCollisionModel();


        while (ros::ok())
        {


            /*double shifterx = 0;
            nh_->param<double>("shifterx", shifterx, 0);
            double shiftery = 0;
            nh_->param<double>("shiftery", shiftery, 0);
            double shifterz = 0;
            nh_->param<double>("shifterz", shifterz, 0);*/


            //fixed_to_ik_variable.getOrigin() = fixed_to_ik.getOrigin() + tf::Vector3(shifterx,shiftery,shifterz);

            //pcl_ros::transformPointCloud(*cloud,*cloud_torso,fixed_to_ik_variable);

            //! take in the full pointcloud as obstacles for now
            //collision_testing.addPointCloud(cloud_torso);

            //pubCloud("cloud", cloud, fixed_frame_.c_str());
            //pubCloud("collision", cloud_torso, ik_frame_.c_str());
            pubCloud("collision", cloud, fixed_frame_);

            std::cout << "updating collision mode with pc" << std::endl;

            tf::Transform root = collision_testing.kinematic_state->getRootTransform();

            geometry_msgs::Transform root_msg;

            //! shift robot in planning env, does shift points in odom_combined or torso with it!
            //root.setOrigin(tf::Vector3(shifterx,shiftery,shifterz));

            tf::transformTFToMsg(root, root_msg);

            std::cout << " ROOT TRANSFORM " << root_msg << std::endl;

            //collision_testing.kinematic_state->getJointState("world_joint")->setJointStateValues(root);

            std::cout << "collision_testing.kinematic_state->updateKinematicLinks();" << std::endl;

            collision_testing.kinematic_state->updateKinematicLinks();

            collision_testing.publish_markers = true;

            std::vector<double> result;
            result.resize(7);
            std::fill( result.begin(), result.end(), 0 );

            collision_free.clear();

            ROS_INFO("REACHABLE %zu", reachable.size());
            for (std::vector<tf::Pose>::iterator it = reachable.begin(); (it!=reachable.end()) && ros::ok(); ++it)
            {
                std::cout << "tick" << std::endl;
                tf::Pose in_ik_frame = fixed_to_ik.inverseTimes(*it);
                if (get_ik(arm, in_ik_frame, result) == 1)
                {
                    bool inCollision = collision_testing.inCollision(arm, result);
                    if (!inCollision)
                        collision_free.push_back(*it);
                }
            }

            std::cout << "number of collision free grasps : " << collision_free.size() << " out of " << reachable.size() << std::endl;

            rt.sleep();
        }

    }

    pubCloud("percentage", percentage_cloud, fixed_frame_.c_str());

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
        }
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
    ps_ary.header.frame_id = fixed_frame_;

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

    /*
       ros::Rate rt(1);

       tf::Stamped<tf::Pose> torso_in_odom;
       torso_in_odom.frame_id_ = "odom_combined";
       //torso_in_odom.setOrigin(tf::Vector3(0,0,0.33));
       torso_in_odom.setOrigin(tf::Vector3(0,0,.33));
       torso_in_odom.setRotation(tf::Quaternion(0,0,0,1));

       geometry_msgs::PoseStamped fixed_to_ik_msg;
       tf::Stamped<tf::Pose> inverted = fixed_to_ik;
       inverted.setData(inverted.inverse());
       tf::poseStampedTFToMsg(fixed_to_ik,fixed_to_ik_msg);

       std::cout << "fixed_to_ik in bagfile " << fixed_to_ik_msg << std::endl;

       tf::poseStampedTFToMsg(inverted,fixed_to_ik_msg);

       std::cout << "fixed_to_ik in bagfile inverted " << fixed_to_ik_msg << std::endl;

       while (ros::ok())
       {
           // torso_lift_link to tum_os_table
           br->sendTransform(tf::StampedTransform(fixed_to_ik, ros::Time::now(), ik_frame_, fixed_frame_));
           rt.sleep();
           // odom_combined to torso_lift_link
           br->sendTransform(tf::StampedTransform(torso_in_odom, ros::Time::now(), "odom_combined", ik_frame_));
           rt.sleep();

           if (map_to_odom_set)
           {
               br->sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), "odom_combined", "map"));
               rt.sleep();
           }

           //std::cout << "blip" << std::endl;
       }
       */
}



int main(int argc,char **argv)
{

    ros::init(argc, argv, "pointcloud_to_hull");

    init();

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


    //test_vdc();

    //test_search();

    ros::Rate rt(1);

    //while (ros::ok())
    //{
    //  rt.sleep();
    //test_hull_calc();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //tf::Stamped<tf::Pose> fixed_to_ik;
    tf::Stamped<tf::Pose> sensor_in_fixed;


    /*if (data_from_bag)
    {
        arm_navigation_msgs::GetPlanningScene::Response *planning_scene_res;
        planning_scene_res = new arm_navigation_msgs::GetPlanningScene::Response ();\
        std::string filename = "planning_scene_res.bag";

        rosbag::Bag bag;
        bag.open(filename, rosbag::bagmode::Read);

        rosbag::View view(bag);

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            std::cout << "m.getTopic" << m.getTopic() << std::endl;
            if (m.getTopic() == "planning_scene_res")
            {
                std::cout << "Got the planning scene response from bagfile " << filename << std::endl;
                arm_navigation_msgs::GetPlanningSceneResponse::ConstPtr msg_in = m.instantiate<arm_navigation_msgs::GetPlanningSceneResponse>();
                *planning_scene_res = *msg_in;
                planning_scene_res->planning_scene.collision_map.boxes.clear();
                //std::cout << *planning_scene_res << std::endl;
                //pcl::fromROSMsg(*msg_in, *cloud);
            }
        }
    }*/

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

        /*

        fixed frame header:
          seq: 0
          stamp: 1352303530.162666972
          frame_id: odom_combined
        child_frame_id: tum_os_table
        transform:
          translation:
            x: 0.340477
            y: -0.523673
            z: -0.739989
          rotation:
            x: 0.00536571
            y: -0.00617949
            z: 0.697902
            w: 0.716146

        fixed_to_ik in bagfile header:
          seq: 0
          stamp: 1353333899.758390934
          frame_id: /tum_os_table
        pose:
          position:
            x: 0.201368
            y: -0.463186
            z: 0.333089
          orientation:
            x: -0.000498488
            y: 0.00269517
            z: 0.693091
            w: 0.720845



        */

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
        //- Translation: [-0.050, 0.000, 1.081]
        //- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
        //      in RPY [0.000, -0.000, 0.000]

//- Translation: [0.364, -0.676, 0.341]
//- Rotation: in Quaternion [0.000, -0.001, 0.751, 0.660]
        //          in RPY [-0.001, -0.001, 1.700]



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

            size_t hit = -1;

            for (size_t i = 0; i < collision_testing.planning_scene.fixed_frame_transforms.size(); ++i)
            {
                std::cout << "fixed frame " << collision_testing.planning_scene.fixed_frame_transforms[i] << std::endl;

                tf::StampedTransform act;
                tf::transformStampedMsgToTF(collision_testing.planning_scene.fixed_frame_transforms[i], act);
                //geometry_msgs/TransformStamped[]
                //std::string tmp = act.frame_id_;
                //act.frame_id_ = act.child_frame_id_;
                //act.child_frame_id_ = tmp;
                //act.setData(act.inverse());
                //!transforms_from_planning_response.push_back(act);

                //if (collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].child_frame_id == "map")
                //if (collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].header.frame_id== "odom_combined")
                //{
                //ROS_ERROR("NOT AN ERROR");
                //tf::transformMsgToTF(collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].transform, map_to_odom);
                //map_to_odom_set = true;
                //}

                /*if (0)
                if (collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].child_frame_id == "tum_os_table")
                {
                    hit = i;
                    std::cout << "HIT" << i << std::endl;
                    geometry_msgs::Transform odom_to_tumos;
                    tf::transformTFToMsg(fixed_to_ik,odom_to_tumos);
                    odom_to_tumos.translation.z = -.74;
                    collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i].transform = odom_to_tumos;

                    std::cout << "fixed frame modified " << collision_testing.planning_scene_res.planning_scene.fixed_frame_transforms[i] << std::endl;
                }*/
            }

            //for (size_t j = 0; j < collision_testing.fixed_frame_transforms.size(); j++)
            //transforms_from_planning_response.push_back(collision_testing.fixed_frame_transforms[j]);

            collision_testing.updateCollisionModel();

            tf::Transform root = collision_testing.kinematic_state->getRootTransform();

        }

        boost::thread ty(&tf_broadcaster_thread);

    }



    testOctomap(cloud, fixed_to_ik, sensor_in_fixed);
    //}

    while (ros::ok())
    {
        rt.sleep();
        ros::spinOnce();
    }


}
