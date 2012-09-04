
#ifndef __TABLE_TOP_OBJECT_H__
#define __TABLE_TOP_OBJECT_H__

#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <octomap_ros/OctomapROS.h>

//represents a cluster as a pointcloud and the octomap including its occupancy and the space hidden by it as seen from the sensor
class TableTopObject
{
public:

    TableTopObject(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box);

    bool checkCollision(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject);

    octomap::OcTreeROS *m_octoMap;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

};


#endif
