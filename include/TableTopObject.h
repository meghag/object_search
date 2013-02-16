
#ifndef __TABLE_TOP_OBJECT_H__
#define __TABLE_TOP_OBJECT_H__

#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <octomap_ros/OctomapROS.h>

typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointT;

//represents a cluster as a pointcloud and the octomap including its occupancy and the space hidden by it as seen from the sensor
class TableTopObject
{
public:

    TableTopObject();

    TableTopObject(pcl::PointCloud<PointT>::Ptr cloud_in_box);

    TableTopObject(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT>::Ptr cloud_in_box);

    TableTopObject(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT> cloud_in_box);

    bool checkCollision(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject);

    bool checkCollisionPointcloud(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject);
    bool checkCoveredPointcloud(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject);

    void projectToPlanePerspective(const tf::Vector3 sensorOrigin, const double tableHeight, const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_projected);
    void projectToPlanePerspective2(const tf::Vector3 sensorOrigin, const double tableHeight, const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>::Ptr &cloud_projected);

    pcl::PointCloud<PointT>::Ptr getAsCloud();

    octomap::OcTreeROS *m_octoMap;
	//octomap::OcTreeROS *m_octoMap_copy;
    pcl::PointCloud<PointT>::Ptr cloud;
	//pcl::PointCloud<PointT>::Ptr cloud_copy;

    bool has_octo;

    void addPointCloud(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT>::Ptr cloud_in_box);
    void addPointCloud2(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT> cloud_in_box);

    void initOcto();

};


#endif
