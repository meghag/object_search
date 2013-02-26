
#include "../include/grasp_planning.h"

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
