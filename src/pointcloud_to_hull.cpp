#include <algorithm>
#include <set>
#include <iterator>

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

#include <std_srvs/Empty.h>

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

// do not publish anything in inner planner loops
bool silent = true;


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mc_graspable/FindGraspablesAction.h>

#include <stdlib.h>

void start_simulator()
{
    int i = system("rosservice call gazebo/unpause_physics");
    std::cout << "system " << i << std::endl;
}

void pause_simulator()
{
    int i = system("rosservice call gazebo/pause_physics");
    std::cout << "system " << i << std::endl;
}

void finish()
{
    pause_simulator();
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


#include "util.cpp"

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
                           bool test_full_env,
                           CollisionTesting &ct_full_env,
                           bool test_obj_excluding,
                           std::vector<CollisionTesting> &ct_obj_excluding,
                           CollisionTesting &ct_table,
                           std::vector<TableTopObject*> obj_only,
                           std::vector<Push> *pushes = 0L)
{
    GraspPlanning grasp;

    tf::Transform push_transform;
    push_transform.setIdentity();

    size_t max_remaining = 0;
    for (std::vector<tf::Pose>::iterator jjt = object_posterior_belief.begin(); jjt!=object_posterior_belief.end(); jjt++)
    {
        if (obj.checkCoveredPointcloud(*jjt,push_transform,*obj_only[max_idx]))
            max_remaining++;
    }

    //ROS_ERROR("MAX REMAINING CLUSTER %i gives %zu", max_idx, max_remaining);

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
    cluster_min -= tf::Vector3(.2,.2,0);
    cluster_max += tf::Vector3(.2,.2,.05);

    //! Dangerous, we're using tf now for a test live
    //fixed_to_ik = getPose(fixed_frame_, ik_frame_);

    //geometry_msgs::PoseStamped ps;
    //tf::poseStampedTFToMsg(fixed_to_ik, ps);
    //std::cout << "FIXED TO IK " << ps << std::endl;

    //finish();

    //tf::Stamped<tf::Pose> odom_to_torso = getPose("odom_combined", ik_frame_);

    //! num grasps per cluster and arm
    // 0.02 sec for 12.5k
    for (int k =0; k < 25000; k++)
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

    //pub_belief("checked_grasps",random,fixed_frame_);

    //finish();

    //ROS_INFO("tick");
    // should have points of cluster we want to grasp inside

    ROS_INFO("BEFORE CHECKING GRASPS");
    grasp.checkGraspsIK(arm,fixed_to_ik,random,ik_checked);
    ROS_INFO("checked for reachability, %zu reachable grasp candidates", ik_checked.size());
    grasp.checkGrasps(obj_only[max_idx]->cloud,ik_checked,filtered);

    //std::cout << "number of filtered grasps : " << filtered.size() << " out of " << random.size() << std::endl;

    // check against general collision
    std::vector<tf::Vector3> normals, centers;
    std::vector<int> grasp_indices;
    grasp.checkGrasps(full_environment.cloud,filtered,reachable,&grasp_indices,&normals,&centers);

    //ROS_INFO("AFTER CHECKING GRASPS");

    ROS_INFO ("number of reachable grasps : %zu out of %zu", reachable.size() , ik_checked.size() );

    if (!silent)
        pub_belief("checked_grasps",checked);

    if (!silent)
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

            tf::Pose in_ik_frame = fixed_to_ik.inverseTimes(*it);

            tf::Pose in_ik_frame_push = in_ik_frame * rel;

            if (ct_table.inCollision(arm, in_ik_frame))
                continue;

            if (ct_table.inCollision(arm, in_ik_frame_push))
                continue;

            if (test_full_env && ct_full_env.inCollision(arm, in_ik_frame))
                continue;

            if (test_obj_excluding && ct_obj_excluding[max_idx].inCollision(arm,in_ik_frame_push))
                continue;

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
            //std::cout << push.getOrigin().z() << std::endl;

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
            //std::cout << "PUSH REL idx " << grasp_indices[sit] << " vec "<< rel.getOrigin().x() << " "<< rel.getOrigin().y() << " "<< rel.getOrigin().z() << std::endl;
            //std::cout << "PUSH VECTOR" << push_vector.x() << " "<< push_vector.y() << " "<< push_vector.z() << std::endl;
            //std::cout << "PUSH normal                             " << normal.x() << " "<< normal.y() << " "<< normal.z() << " " << std::endl;

            double cos_angle = fabs(cos(push_vector.angle(normal)));

            //std::cout << "cosangle" << cos_angle <<  " angle " << push_vector.angle(normal) << std::endl;

            // how far do we push until we touch the object ?
            double amt_step = .1; // 1 cm steps as push lenght is 10 cm
            double amt_free = 0;
            for (double amt = 0; amt <= 1.001; amt += amt_step)
            {

                tf::Pose check_pose = in_ik_frame;
                check_pose.getOrigin() = (in_ik_frame.getOrigin() * (1 - amt)) + (in_ik_frame_push.getOrigin() * amt);
                bool inCo = ct_full_env.inCollision(arm,check_pose);
                inCo |= ct_table.inCollision(arm,check_pose);

                if (inCo)
                    amt = 100;
                else
                    amt_free = amt;
            }

            //std::cout << amt << " inco " <<  (inCo ? "true " : "false ") << amt_free << std::endl;
            /*int cnt = 0;
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

            }*/

            //std::cout <<  "AMT FRE" << amt_free << std::endl;

            // how far can we push the object before collision
            double end_free = 1;

            double end_step = .1;
            for (double end = 1; end <= 2.001; end += end_step)
            {

                tf::Pose check_pose = in_ik_frame;
                check_pose.getOrigin() = in_ik_frame.getOrigin() + end * (in_ik_frame_push.getOrigin() - in_ik_frame.getOrigin());

                bool inCo = test_obj_excluding && ct_obj_excluding[max_idx].inCollision(arm,check_pose);
                inCo |= ct_table.inCollision(arm,check_pose);

                //std::cout << amt << " inco " <<  (inCo ? "true " : "false ") << amt_free << std::endl;

                if (inCo)
                    end = 100;
                else
                    end_free = end;
            }

            //std::cout <<  "END FREE" << end_free << std::endl;
            //std::cout <<  "              gives us " << end_free - amt_free << std::endl;

            tf::Transform push_transform;
            //push_transform.setOrigin(tf::Vector3(push_vector.x(),push_vector.y(),0));
            //! HACK: only sidewards pushes? ERROR TODO
            push_transform.setOrigin(tf::Vector3(0,push_vector.y(),0));
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

            //if (current_push.num_removed > 0)
            //if (current_push.num_removed > 0.6 * max_remaining)
                pushes->push_back(current_push);

        }

        std::cout << "number of collision free grasps : " << pushes->size() << " out of " << reachable.size() << std::endl;

    }

    if (!silent)
        pubCloud("normals", cloud_normalvis, "torso_lift_link");

    //finish();

    //take a random grasp for debugging
    std::sort (pushes->begin(), pushes->end(), compare_push);
}


// abstract plan, only reflects order of object removals but not by which manipulation primitive
struct plan
{
    std::vector<int> remove_order;
    std::set<int> moved_objects;
    std::vector<double> percentage;
};

struct manipulation_plan
{
    size_t abstract_plan_index;
    std::vector<int> moved_objects;
    std::vector<int> primitive;
    std::vector<double> percentage;
    double expected_time;
    bool executable;
};

void print_manip_plan(manipulation_plan &manip)
{
    std::cout << "plan idx " << manip.abstract_plan_index;
    for (std::vector<int>::iterator it = manip.moved_objects.begin(); it!=manip.moved_objects.end(); it++)
        std::cout << " rem " << *it;
    for (std::vector<int>::iterator it = manip.primitive.begin(); it!=manip.primitive.end(); it++)
        std::cout << " prtv " << *it;
    for (std::vector<double>::iterator it = manip.percentage.begin(); it!=manip.percentage.end(); it++)
        std::cout << " % " << *it;
    std::cout << std::endl;
}


bool compare_manipulation_plan_expected(const manipulation_plan &i,const manipulation_plan &j)
{
    return (i.expected_time < j.expected_time);
}


//void fill_plan(plan abstract_plan, manipulation_plan *manip_plan, std::vector<std::vector<int> &indices, int depth = 0)
//{
//  manipul
//}

void print_push(const Push& a)
{
    std::cout << "Push " << " removes " << a.num_removed << " arm " << a.arm << " clus " << a.cluster_index << " vec " << a.object_motion.x() << " "
              << a.object_motion.y() << " " << a.object_motion.z() << " \t" << "len" << a.object_motion.length();// << std::endl;
    std::cout << std::endl;
}


//! check if all objects that were blocking this one were already removed in this abstract plan
bool still_blocked(int j,std::set<int> moved_objects, std::set<int> blocked)
{
    //! this will not scale well, sorting the remove_order each time!
    //std::set<int> remove_order_set(remove_order.begin(), remove_order.end());

    std::set<int> result;
    std::set_difference(blocked.begin(), blocked.end(), moved_objects.begin(), moved_objects.end(), std::inserter(result,result.begin()));
    //std::set_difference(blocked.begin(), blocked.end(), remove_order_set.begin(), remove_order_set.end(), std::inserter(result,result.begin()));


    //std::cout << "still blocked remove_order";
    //for (std::vector<int>::iterator it = remove_order.begin(); it!=remove_order.end(); it++)
    //std::cout << " " << *it;

    //std::cout << std::endl << "blocked";

    //for (std::set<int>::iterator it = blocked.begin(); it!=blocked.end(); it++)
    //  std::cout << " " << *it;

    //std::cout << std::endl;

    //std::cout << " result ";
    //for (std::set<int>::iterator it = result.begin(); it!=result.end(); it++)
    //std::cout << " " << *it;
    //std::cout << std::endl;

    return (result.size() > 0);
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

    if (cloud_in_box->points.size() == 0)
    {
        ROS_ERROR("CLOUD EMPTY IN SEARCH AREA, FALLING BACK");
        finish();
    }

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
    //int max_idx = -1;
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
            //max_idx = i;
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
    //ct_full_env.addPointCloud(table,planning_precision ,&tum_os_table_to_odom_combined);
    ct_full_env.updateCollisionModel();
    ct_full_env.kinematic_state->updateKinematicLinks();
    ct_full_env.publish_markers = !silent;

    //table ct
    CollisionTesting ct_table(*nh_);
    ct_table.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);
    ct_table.setCollisionFrame("odom_combined");
    ct_table.addPointCloud(table,planning_precision ,&tum_os_table_to_odom_combined);
    ct_table.updateCollisionModel();
    ct_table.kinematic_state->updateKinematicLinks();
    ct_table.publish_markers = !silent;

    std::vector<CollisionTesting> ct_obj_excluding;
    for (size_t k = 0; k < obj_excluding.size(); k++)
    {
        CollisionTesting act(*nh_);
        act.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);
        act.setCollisionFrame("odom_combined");
        act.addPointCloud( obj_excluding[k]->cloud, planning_precision , &tum_os_table_to_odom_combined);
        //act.addPointCloud(table,planning_precision ,&tum_os_table_to_odom_combined);
        act.updateCollisionModel();
        act.kinematic_state->updateKinematicLinks();
        act.publish_markers = !silent;
        ct_obj_excluding.push_back(act);
    }



    std::vector<CollisionTesting> ct_obj_only;
    for (size_t k = 0; k < obj_only.size(); k++)
    {
        CollisionTesting act(*nh_);
        act.init(data_from_bag, "planning_scene_res.bag",fixed_frame_);
        act.setCollisionFrame("odom_combined");
        act.addPointCloud( obj_only[k]->cloud, planning_precision , &tum_os_table_to_odom_combined);
        //act.addPointCloud(table,planning_precision ,&tum_os_table_to_odom_combined);
        act.updateCollisionModel();
        act.kinematic_state->updateKinematicLinks();
        act.publish_markers = !silent;
        ct_obj_only.push_back(act);
    }

    std::vector<Push> pushes;

    bool greedy = false;

    //greedy = true;

    if (greedy )
    {
        for (int arm_i = 0; arm_i < 2; arm_i ++)
            for (size_t k = 0; k < obj_only.size(); k++)
                generate_valid_pushes(object_posterior_belief,
                                      k,
                                      arm_i ,
                                      fixed_to_ik,
                                      tum_os_table_to_odom_combined,
                                      obj,
                                      full_environment,
                                      true,
                                      ct_full_env,
                                      true,
                                      ct_obj_excluding,
                                      ct_table,
                                      obj_only,
                                      &pushes);

        ROS_INFO("PUSHES SIZE %zu", pushes.size());

        for (size_t  p =0 ; p < pushes.size(); p++)
        {
            Push &a = pushes[p];
            print_push(a);
            std::cout << "Push " << p << " removes " << a.num_removed << " arm " << a.arm << " clus " << a.cluster_index << " vec " << a.object_motion.x() << " "
                      << a.object_motion.y() << " " << a.object_motion.z() << " " << "len" << a.object_motion.length();// << std::endl;
            for (std::set<int>::iterator it = pushes[p].blocked_by.begin() ; it != pushes[p].blocked_by.end() ; ++it)
                std::cout << " B " << *it;
            std::cout << std::endl;
        }

        //finish();

        if (pushes.size() > 0)
        {

            bool success = false;

            for (size_t  index = 0; (!success) && (index < pushes.size()); index++)
            {

                ROS_INFO("PUSHES INDEX %zu", index);

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
                    RobotArm::reset_arms(arm);
                    success = true;
                }
            }

        }

    }
    else
    {

        //silent = false;
        // multistep planning
        for (int arm_i = 0; arm_i < 2; arm_i ++)
            for (size_t k = 0; k < obj_only.size(); k++)
                generate_valid_pushes(object_posterior_belief,
                                      k,
                                      arm_i ,
                                      fixed_to_ik,
                                      tum_os_table_to_odom_combined,
                                      obj,
                                      full_environment,
                                      false,
                                      ct_full_env,
                                      false,
                                      ct_obj_excluding,
                                      ct_table,
                                      obj_only,
                                      &pushes);
        pause_simulator();

        silent = true;

        std::vector<std::set<int> > not_blocked;

        std::vector<std::set<int> > blocked;

        std::vector<std::vector<Push*> > pushes_by_cluster;
        pushes_by_cluster.resize(ct_obj_only.size());

        std::set<int> full_set;

        ROS_INFO("PRE BLOCKING");

        not_blocked.resize(ct_obj_only.size());

        for (size_t j = 0; j < ct_obj_only.size(); j++)
        {
            not_blocked[j].insert(j);
            full_set.insert(j);
        }

        //check for each grasp if it is blocked by another object
        for (size_t k = 0; k < pushes.size(); ++k)
        {

            //std::cout << "Push " << k << " cluster " << pushes[k].cluster_index <<  " vector " << pushes[k].object_motion.x() << " " << pushes[k].object_motion.y() << " " << pushes[k].object_motion.z() << std::endl;
            print_push(pushes[k]);

            // different indexing, by cluster id
            pushes_by_cluster[pushes[k].cluster_index].push_back(&pushes[k]);

            for (size_t j = 0; j < ct_obj_only.size(); j++)
            {
                if (j != pushes[k].cluster_index)
                {
                    //if (pushes[k].blocked_by.find(j)==pushes[k].blocked_by.end())

                    //ct_obj_only[j].push_offset(tf::Vector3(0,0,0));

                    if ((ct_obj_only[j].inCollision(pushes[k].arm, pushes[k].from)) ||
                            (ct_obj_only[j].inCollision(pushes[k].arm, pushes[k].to)))
                    {
                        //std::cout << j << "blocks " << pushes[k].cluster_index << std::endl;
                        pushes[k].blocked_by.insert(j);
                    }
                    else
                    {
                        not_blocked[pushes[k].cluster_index].insert(j);
                    }

                    //ct_obj_only[j].pop_offset();
                }
            }
        }

        ROS_INFO("POST BLOCKING");

        blocked.resize(not_blocked.size());

        for (size_t k = 0; k < not_blocked.size(); ++k)
        {
            //std::set<int>::iterator kt =std::set<int> result;
//std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(),
            //  std::inserter(result, result.end()));
            std::set_difference(full_set.begin(), full_set.end(), not_blocked[k].begin(), not_blocked[k].end(), std::inserter(blocked[k],blocked[k].begin()));
            //blocked[k].resize(kt-blocked[k].begin());
            std::cout << "Cluster " << k ;//<< " is not blocked by ";
            //for (std::set<int>::iterator it = not_blocked[k].begin(); it != not_blocked[k].end(); ++it)
                //std::cout << *it << " ";
            std::cout << "\t is blocked by ";
            for (std::set<int>::iterator it = blocked[k].begin(); it != blocked[k].end(); ++it)
                std::cout << *it << " ";
            std::cout << std::endl;
        }

        // plan ahead
        int horizon = std::min(2,(int)obj_only.size()); // remove fixed horizon
        std::vector<std::vector<plan> > graph;
        graph.resize(horizon+1);

        std::cout << " HORIZON " << horizon << std::endl;

        // start with any object that is not blocked in first step
        for (size_t j = 0; j < ct_obj_only.size(); j++)
        {
            plan act;
            bool can_be_removed = !still_blocked(j,act.moved_objects,blocked[j]);
            act.remove_order.push_back(j);
            act.moved_objects.insert(j);
            if (can_be_removed)
            {
                graph[0].push_back(act);
                std::cout << "Plan init with cluster " << j << std::endl;
            }
        }

        for (int h = 1; h <= horizon; h++)
        {
            for (std::vector<plan>::iterator it=graph[h-1].begin() ; it!=graph[h-1].end(); ++it)
            {
                plan &act = *it;
                //! debug
                std::cout << "Plan ";
                for (std::vector<int>::iterator jt= it->remove_order.begin(); jt!=it->remove_order.end(); ++jt)
                {
                    std::cout << *jt << " ";
                }
                std::cout << std::endl;
                //! debug

                for (size_t j = 0; j < ct_obj_only.size(); j++)
                {
                    // if current object is not yet in the plan

                    std::cout << "Testing " << j << std::endl;

                    if (act.moved_objects.find(j) == act.moved_objects.end())
                    {
                        std::cout << "not removed " << j << std::endl;
                        // if object is not blocked
                        if (!still_blocked(j,act.moved_objects,blocked[j]))
                        {
                            std::cout << "not blocked " << j << std::endl;
                            plan new_plan = act;
                            new_plan.remove_order.push_back(j);
                            new_plan.moved_objects.insert(j);
                            graph[h].push_back(new_plan);

                            std::cout << " new entry with size : " << new_plan.remove_order.size() << " : ";

                            for (std::vector<int>::iterator kt= new_plan.remove_order.begin(); kt!=new_plan.remove_order.end(); ++kt)
                            {
                                std::cout << *kt << " ";
                            }
                            std::cout << std::endl;

                        }

                    }
                }
            }
        }

        size_t full_depth = 0;

        std::cout << "After planning: graph size" << graph.size() << std::endl;
        for (size_t depth = 0 ; depth < graph.size(); depth++) //std::vector<std::vector<plan> >::iterator it = graph.begin(); it!= graph.end(); it++)
        {
            std::cout << "depth " << depth <<" size "  << graph[depth].size() << std::endl;
            if (graph[depth].size() > 0)
                full_depth = depth;
        }

        std::vector<manipulation_plan> manip_plans;


        // for all abstract plans, checkall combinations of manipulation primitves that constitute them
        for (std::vector<plan>::iterator it=graph[full_depth].begin() ; (it!=graph[full_depth ].end()); ++it)
        {
            bool found = false;
            // current abstract plan
            plan &cap = *it;

            std::cout << "Plan " << std::endl;

            for (std::vector<int>::iterator jt= it->remove_order.begin(); jt!=it->remove_order.end(); ++jt)
            {

                std::cout << *jt << " ";
            }
            std::cout << std::endl;

            manipulation_plan current_plan;
            current_plan.primitive.resize(cap.remove_order.size());
            current_plan.moved_objects.resize(cap.remove_order.size());
            current_plan.abstract_plan_index = it - graph[full_depth].begin();

            int depth = 0;
            int max_depth = cap.remove_order.size() - 1;

            std::cout << "max_depth" << max_depth << std::endl;

            std::vector<int> indices;
            indices.resize(cap.moved_objects.size());
            std::fill( indices.begin(), indices.end(), 0);

            for (int k = 0; k < max_depth; ++k)
                std::cout << "cluster" << cap.remove_order[k] << " num primitives " << pushes_by_cluster[cap.remove_order[k]].size() << std::endl;

            int debi = 0;

            while (!found)
            {

                current_plan.primitive[depth] = indices[depth];
                current_plan.moved_objects[depth] = cap.remove_order[depth];

                // leave shorter plans in as fallbacks in case we don't find a full length plan
                if (depth < max_depth)
                {
                    manipulation_plan curr_fallback_plan = current_plan;
                    curr_fallback_plan.primitive.resize(depth);
                    curr_fallback_plan.moved_objects.resize(depth);

                    std::cout << " pushing  fallback plan depth" << depth << " ";
                    print_manip_plan(curr_fallback_plan);
                    //curr_fallback_plan.primitive.push_back(indices[depth]);
                    //curr_fallback_plan.moved_objects.push_back(cap.remove_order[depth]);
                    //curr_fallback_plan.abstract_plan_index = it - graph[full_depth].begin();
                    manip_plans.push_back(curr_fallback_plan);
                }

                indices[depth]++;

                debi++;
                if (debi > 50)
                    found = true;

                //std::cout << "depth " << depth << std::endl;
                //std::cout << " indices[depth]" << indices[depth] << std::endl;
                //std::cout <<" cap.remove_order[depth] " <<  cap.remove_order[depth] << std::endl;
                //std::cout << " pushes_by_cluster.size() " << pushes_by_cluster.size() << std::endl;
                //std::cout << "pushes_by_cluster[cap.remove_order[depth]].size()" << pushes_by_cluster[cap.remove_order[depth]].size() << std::endl;

                // we reached highest primitive in this depth, ->backtracking
                if (indices[depth] > (int) pushes_by_cluster[cap.remove_order[depth]].size())
                {

                    // reset this depth to zero
                    indices[depth] = 0;

                    if (depth == 0)
                        found = true;

                    depth--;
                }
                else
                {

                    if (depth==max_depth)
                    {
                        std::cout << "manip_plan___________________________depth " << depth << " ";
                        print_manip_plan(current_plan);

                        manip_plans.push_back(current_plan);

                        for (int k = 0; k <  max_depth; k++)
                            std::cout << current_plan.primitive[k] << " ";
                        std::cout << std::endl;
                    }


                    if (depth < max_depth)
                    {
                        depth++;
                    }
                }

                //std::cout << indices[depth] << " max at idx " << pushes_by_cluster[cap.remove_order[depth]].size() << std::endl;

            }

            /*while (!found)
            {
                while
                  ((indices[depth] > pushes_by_cluster[plan.remove_order[depth]].size()) && !found)
                 {
                     indices[depth] = 0;
                 }
                // check if object is still reachable
                current_plan.primitive[depth] = indices[depth];
                if (depth < max_depth)
                {

                    depth+=1;
                } else
                {
                    indices[depth] = 0;
                    indices[depth-1]++;
                }

            }

            for (std::vector<int>::iterator jt= it->remove_order.begin(); jt!=it->remove_order.end(); ++jt)
            {

            }

            */

            // lets try if it looks executable

        }

        double belief_size = object_posterior_belief.size();

        //std::vector<manipulation_plan> manip_plans;
        for (std::vector<manipulation_plan>::iterator im = manip_plans.begin(); im != manip_plans.end(); ++im)
        {
            std::cout << "Manip Plan Objects: ";
            size_t k;
            for (k = 0; k < im->moved_objects.size(); ++k)
            {
                std::cout << " " << im->moved_objects[k];
            }
            std::cout << " primitives:";
            for (k = 0; k < im->moved_objects.size(); ++k)
            {
                std::cout << " " << im->primitive[k];
            }

            std::cout << " num removed:";
            int sum = 0;
            double expected_time = 0;
            for (k = 0; k < im->moved_objects.size(); ++k)
            {
                double removed = pushes_by_cluster[im->moved_objects[k]][im->primitive[k]]->num_removed;
                sum += removed;
                expected_time += ( k + 1 ) * (removed / belief_size);
                std::cout << " " << removed;

                im->percentage.push_back(removed / belief_size);
            }
            double rest = belief_size - sum;
            //! arbitrary number 100
            expected_time += ( im->moved_objects.size() + 10 ) * rest;

            im->expected_time = expected_time;

            std::cout << " SUM " << sum << " of " << belief_size << " expected time " << expected_time;


            std::cout << std::endl;
        }

        std::sort (manip_plans.begin(), manip_plans.end(), compare_manipulation_plan_expected);

        for (std::vector<manipulation_plan>::iterator im = manip_plans.begin(); im != manip_plans.end(); ++im)
        {
            size_t k;
            for (k = 0; k < im->moved_objects.size(); ++k)
            {
                std::cout << " " << im->moved_objects[k];
            }
            std::cout << " Expected time: " << im->expected_time << std::endl;
        }

        // check if the manip plan is executable by checking for collisions with adjusted cluster positions
        // and cluster/ cluster collisions
        for (std::vector<manipulation_plan>::iterator im = manip_plans.begin(); im != manip_plans.end(); ++im)
        {
            bool blocked = false;
            size_t k;
            std::vector<CollisionTesting*> toPop;

            std::vector<tf::Vector3> obj_relative_positions;
            obj_relative_positions.resize(obj_only.size());
            std::fill(obj_relative_positions.begin(), obj_relative_positions.end(), tf::Vector3(0,0,0));

            std::cout << "Plan";
            for (k = 0; k < im->moved_objects.size(); ++k)
            {
                std::cout << " " << im->moved_objects[k];
            }

            std::cout << " coll " << "moved objects size" << im->moved_objects.size() << std::endl;

            for (k = 0; (k < im->moved_objects.size()) && !blocked; ++k)
            {

                size_t cluster_idx = im->moved_objects[k];
                Push *curr_push = pushes_by_cluster[cluster_idx][im->primitive[k]];

                ct_obj_only[cluster_idx].push_offset(curr_push->object_motion);
                toPop.push_back(&ct_obj_only[cluster_idx]);

                obj_relative_positions[cluster_idx] += curr_push->object_motion;

                bool colliding = false;
                tf::Transform identity_transform;
                identity_transform.setIdentity();

                for (size_t i = 0; i < obj_only.size(); ++i)
                    if (i != cluster_idx)
                    {
                        tf::Transform otherTransform;
                        otherTransform.setRotation(tf::Quaternion(0,0,0,1));
                        otherTransform.setOrigin(obj_relative_positions[i]);
                        tf::Transform ownTransform;
                        ownTransform.setRotation(tf::Quaternion(0,0,0,1));
                        ownTransform.setOrigin(obj_relative_positions[cluster_idx]);
                        bool collid = obj_only[cluster_idx]->checkCollision(ownTransform,otherTransform,*obj_only[i]);
                        if (collid)
                            ROS_ERROR("COLLISION");
                        colliding |= collid;
                    }


                // check reachability for future primitives
                // we know already the first primitive is executable
                if (k > 0)
                {
                    for (size_t i = 0; i < ct_obj_only.size(); ++i)
                    {
                        if (i != cluster_idx)
                        {
                            if ((ct_obj_only[i].inCollision(curr_push->arm, curr_push->from)) ||
                                    (ct_obj_only[i].inCollision(curr_push->arm, curr_push->to)))
                                blocked = true;
                        }
                    }

                }
                std::cout << (colliding ? "C" : (blocked ? "X" : "0"));
                //std::cout << " " << im->moved_objects[k];
            }

            im->executable = !blocked;

            std::cout << std::endl;

            for (k = 0; k < toPop.size(); ++k)
                toPop[k]->pop_offset();
        }

        /*
        struct manipulation_plan
        {
            std::vector<int> primitive;
            std::vector<double> percentage;
        };
        */

        /*for (size_t j = 0; j < ct_obj_only.size(); j++)
        {
            for (size_t k = 0; k < pushes_by_cluster[j].size(); ++k)
            {
                std::cout << " CLUSTER " << j << " ";
                print_push(*pushes_by_cluster[j][k]);
            }
        }*/

        //finish();

        start_simulator();

        bool success = false;

        bool last_failed = false;
        int failed_cluster = -1;
        int failed_primitive = -1;

        RobotArm::getInstance(arm)->open_gripper(0.01);

        for (std::vector<manipulation_plan>::iterator im = manip_plans.begin(); (im != manip_plans.end()) && !success; ++im)
        {

            if ((im->executable) && (im->moved_objects.size()>0))
            {
                size_t cluster_idx = im->moved_objects[0];
                size_t primitive_idx = im->primitive[0];

                // don't send the same goal to move_arm twice

                //Push *curr_push = pushes_by_cluster[cluster_idx][im->primitive[0]];

                //ROS_INFO("PUSHES INDEX %zu", index);


                Push &act_push = *pushes_by_cluster[cluster_idx][primitive_idx];;
                int arm = act_push.arm;

                std::vector<double> result;
                result.resize(7);
                std::fill( result.begin(), result.end(), 0 );

                std::vector<double> result_push;
                result_push.resize(7);
                std::fill( result_push.begin(), result_push.end(), 0 );

                int err = get_ik(arm, act_push.from, result);
                int err2 = get_ik(arm, act_push.to , result_push);

                ROS_INFO("ERROR CODES %i %i", err, err2);

                std::cout << "TRYING MANIPULATION PLAN";
                print_manip_plan(*im);
                //std::cout << std::endl;

                if ((last_failed) && (cluster_idx == failed_cluster) && (primitive_idx == failed_primitive))
                {
                    ROS_INFO("Skipping plan with same initial primitive as the one that just failed");
                    continue;
                }

                int failure = RobotArm::getInstance(arm)->move_arm(act_push.from);
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
                        tf::Pose higher = act_push.from;
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
                    RobotArm::reset_arms(arm);
                    success = true;
                }
                else
                {
                    last_failed = true;
                    failed_cluster = cluster_idx;
                    failed_primitive = primitive_idx;
                }


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

    ros::ServiceClient reset_planning_scene_client =
                nh_->serviceClient<std_srvs::Empty>("/collider_node/reset");

    while (ros::ok())
    {
        std_srvs::Empty empt;
        reset_planning_scene_client.call(empt);

        if (!data_from_bag)
            getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(2));

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

        //RobotArm::reset_arms();

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

    start_simulator();

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
        start_simulator();
        RobotArm::reset_arms();
        finish();
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
