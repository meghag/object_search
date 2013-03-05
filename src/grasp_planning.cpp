#include "../include/grasp_planning.h"
#include "../include/van_der_corput.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues);

GraspPlanning::GraspPlanning()
{
    markerArrayPub_ = 0L;
    nh_ = 0L;
    initGrasps();
}

bool GraspPlanning::inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax)
{
    if ((point.x() > bbmin.x()) && (point.x() < bbmax.x()) &&
            (point.y() > bbmin.y()) && (point.y() < bbmax.y()) &&
            (point.z() > bbmin.z()) && (point.z() < bbmax.z()))
        return true;
    else
        return false;
}

void GraspPlanning::checkGrasps(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked)
{
    // min coordinates of aabb
    std::vector<tf::Vector3> bb_min;
    // max coordinates of aabb
    std::vector<tf::Vector3> bb_max;
    // should this bounding box be empty => true or should contain a point => false
    std::vector<bool> bb_full;

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

    std::vector<size_t> bb_cnt;
    bb_cnt.resize(bb_min.size());

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
            for (size_t k = 0; (k < bb_min.size()); ++k)
            {
                if (inside(curr, bb_min[k], bb_max[k]))
                {
                    bb_cnt[k]++;
                    if (!bb_full[k])
                        good = false;
                }

            }
        }

        //std::cout << std::endl;
        for (size_t j = 0; j < bb_min.size(); j++)
        {
            if (bb_full[j] && (bb_cnt[j] < 10))
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


GraspBoxSet
GraspPlanning::graspBoxSetFromMsg(const tum_os::BoxSet &msg)
{
    GraspBoxSet ret;
    ret.name = msg.name;
    for (std::vector<std_msgs::Bool>::const_iterator it = msg.include_points.begin(); it != msg.include_points.end();  ++it)
        ret.bb_full.push_back(it->data);
    for (std::vector<geometry_msgs::Point>::const_iterator it = msg.min.begin(); it != msg.min.end();  ++it)
    {
        tf::Point pt;
        tf::pointMsgToTF(*it,pt);
        ret.bb_min.push_back(pt);
    }
    for (std::vector<geometry_msgs::Point>::const_iterator it = msg.max.begin(); it != msg.max.end();  ++it)
    {
        tf::Point pt;
        tf::pointMsgToTF(*it,pt);
        ret.bb_max.push_back(pt);
    }
    for (std::vector<geometry_msgs::Pose>::const_iterator it = msg.approach.begin(); it != msg.approach.end();  ++it)
    {
        tf::Pose ps;
        tf::poseMsgToTF(*it,ps);
        ret.approach.push_back(ps);
    }
    return ret;
}


tum_os::BoxSet
GraspPlanning::graspBoxSetToMsg(const GraspBoxSet &boxset)
{
    tum_os::BoxSet ret;
    ret.name = boxset.name;
    for (std::vector<bool>::const_iterator it = boxset.bb_full.begin(); it != boxset.bb_full.end(); ++it)
    {
        std_msgs::Bool bl;
        bl.data = *it;
        ret.include_points.push_back(bl);
    }
    for (std::vector<tf::Point>::const_iterator it = boxset.bb_min.begin(); it != boxset.bb_min.end(); ++it)
    {
        geometry_msgs::Point pt;
        tf::pointTFToMsg(*it, pt);
        ret.min.push_back(pt);
    }
    for (std::vector<tf::Point>::const_iterator it = boxset.bb_max.begin(); it != boxset.bb_max.end(); ++it)
    {
        geometry_msgs::Point pt;
        tf::pointTFToMsg(*it, pt);
        ret.max.push_back(pt);
    }
    for (std::vector<tf::Pose>::const_iterator it = boxset.approach.begin(); it != boxset.approach.end(); ++it)
    {
        geometry_msgs::Pose ps;
        tf::poseTFToMsg(*it, ps);
        ret.approach.push_back(ps);
    }

    return ret;
}


void GraspPlanning::minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( *cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

void GraspPlanning::checkGraspsIK(int arm, tf::Stamped<tf::Pose> fixed_to_ik, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked)
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


template <class PointCloudPtr>
std::vector<tf::Pose> GraspPlanning::calcGrasps(const PointCloudPtr object_model, const PointCloudPtr environment_model, bool check_ik, tf::Stamped<tf::Pose> fixed_to_ik, size_t grasp_type, size_t sample_size)
{
    std::vector<tf::Pose> ik_checked,random,low,high,checked,filtered, reachable, collision_free, ret;
    tf::Vector3 cluster_min, cluster_max;

    minmax3d(cluster_min, cluster_max, object_model);
    cluster_min -= tf::Vector3(.15,.15,.15);
    cluster_max += tf::Vector3(.15,.15,.15);

    ret.resize(sample_size);
    for (int i = 0; i < sample_size; ++i)
        ret.push_back(VanDerCorput::vdc_pose_bound(cluster_min, cluster_max, i));

    return ret;
}

void GraspPlanning::initGrasps()
{
    GraspBoxSet act;

    /*
       act.bb_min.push_back(tf::Vector3(0.21,-0.02,-0.02));
       act.bb_max.push_back(tf::Vector3(0.22,0.02,0.02));
       act.bb_full.push_back(true);

       act.bb_min.push_back(tf::Vector3(0.22,-0.02,-0.02));
       act.bb_max.push_back(tf::Vector3(0.23,0.02,0.02));
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
    act.bb_full.push_back(false);*/

    act.bb_min.push_back(tf::Vector3(0.23,-0.01,-0.02));
    act.bb_max.push_back(tf::Vector3(0.26,0.01,0.02));
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

    tf::Pose push;
    push.setOrigin(tf::Vector3(0.1,0,0));
    push.setRotation(tf::Quaternion(0,0,0,1));
    act.approach.push_back(push);

    act.name = "push_forward";

    /*

        double xShift = .18; // distance toolframe to wrist, we work in wrist later for ik etc

        // we want to see some points centered between the grippers
        act.bb_min.push_back(tf::Vector3(xShift + 0.03,-0.02,-.02));
        act.bb_max.push_back(tf::Vector3(xShift + 0.04, 0.02, .02));
        act.bb_full.push_back(true);

        // we want to see some points centered between the grippers
        act.bb_min.push_back(tf::Vector3(xShift + 0.04,-0.02,-.02));
        act.bb_max.push_back(tf::Vector3(xShift + 0.05, 0.02, .02));
        act.bb_full.push_back(true);

        //coarsest of approximation for gripper fingers when gripper is open
        act.bb_min.push_back(tf::Vector3(xShift + 0.00,0.03,-.02));
        act.bb_max.push_back(tf::Vector3(xShift + 0.05,0.09, .02));
        act.bb_full.push_back(false);

        act.bb_min.push_back(tf::Vector3(xShift + 0.00,-0.09,-.02));
        act.bb_max.push_back(tf::Vector3(xShift + 0.05,-0.03, .02));
        act.bb_full.push_back(false);

        // we want to be able to approach from far away, so check the space we sweep when approaching and grasping
        act.bb_min.push_back(tf::Vector3(xShift - 0.2 ,-0.09,-.03));
        act.bb_max.push_back(tf::Vector3(xShift + 0.00, 0.09, .03));
        act.bb_full.push_back(false);

        act.name = "grasp_forward";
        */

    grasps.push_back(act);

}

void GraspPlanning::visualizeGrasp(size_t grasp_type, std::string frame_id, int highlight)
{

    if (markerArrayPub_==0)
    {
        if (nh_ == 0)
        {
            nh_ = new ros::NodeHandle();
        }
        markerArrayPub_ = new ros::Publisher();
        *markerArrayPub_ = nh_->advertise<visualization_msgs::MarkerArray>( "grasp_visualization", 0 );
    }

    if (grasp_type > grasps.size())
    {
        ROS_ERROR("GRASP %zu not defined", grasp_type);
        return;
    }

    visualization_msgs::MarkerArray arr;

    GraspBoxSet &grasp = grasps[grasp_type];
    for (size_t i = 0; i < grasp.bb_max.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "grasps";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (grasp.bb_max[i].x() + grasp.bb_min[i].x()) / 2.0f;
        marker.pose.position.y = (grasp.bb_max[i].y() + grasp.bb_min[i].y()) / 2.0f;
        marker.pose.position.z = (grasp.bb_max[i].z() + grasp.bb_min[i].z()) / 2.0f;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = grasp.bb_max[i].x() - grasp.bb_min[i].x();
        marker.scale.y = grasp.bb_max[i].y() - grasp.bb_min[i].y();
        marker.scale.z = grasp.bb_max[i].z() - grasp.bb_min[i].z();
        marker.color.a = 0.5;
        marker.color.r = grasp.bb_full[i] ? 0.0 : 1.0;
        marker.color.g = grasp.bb_full[i] ? 1.0 : 0.0;
        marker.color.b = 0.0;

        if (highlight == (int) i)
        {
            marker.color.a = 0.75;
        }

        arr.markers.push_back(marker);
    }

    markerArrayPub_->publish(arr);
}
