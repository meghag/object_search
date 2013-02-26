

#ifndef __GRASP_PLANNING_H__
#define __GRASP_PLANNING_H__

#include <rosbag/bag.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tum_os/BoxSet.h>

class GraspBoxSet
{
public:
    std::string name;
    std::vector<tf::Vector3> bb_min;
    std::vector<tf::Vector3> bb_max;
    std::vector<bool> bb_full;
    std::vector<tf::Pose> approach;
};

class GraspPlanning
{
public :

    explicit GraspPlanning()
        {};

    template <class PointCloudPtr>
    std::vector<tf::Pose> calcGrasps(const PointCloudPtr object_model, const PointCloudPtr environment_model, size_t grasp_type = 0, size_t sample_size = 10000);

    bool inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax);
    void checkGrasps(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked);

    static GraspBoxSet graspBoxSetFromMsg(const tum_os::BoxSet &msg);
    static tum_os::BoxSet graspBoxSetToMsg(const GraspBoxSet &boxset);

};

#endif
