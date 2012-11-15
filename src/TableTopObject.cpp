#include <TableTopObject.h>

using namespace octomap;


void TableTopObject::projectToPlanePerspective(tf::Vector3 sensorOrigin, float tableHeight, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_projected)
{
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        tf::Vector3 pt(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
        tf::Vector3 fromSens = pt - sensorOrigin;
        float factor = (tableHeight - sensorOrigin.z()) / fromSens.z();
        fromSens *= factor;
        fromSens += sensorOrigin;
        pcl::PointXYZRGB newpt = cloud->points[i];
        newpt.x = fromSens.x();
        newpt.y = fromSens.y();
        newpt.z = fromSens.z();
        cloud_projected->points.push_back(newpt);
    }
}

TableTopObject::TableTopObject()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    has_octo = false;
}


TableTopObject::TableTopObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box)
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = cloud_in_box;
    has_octo = false;
}


TableTopObject::TableTopObject(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box)
{
    // is that necessary?
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    has_octo = false;
    addPointCloud(sensorOrigin,tableHeight,cloud_in_box);
}


void TableTopObject::initOcto()
{
    if (!has_octo)
    {
        double m_res = 0.01;
        double m_probHit = 0.7;
        double m_probMiss = 0.4;
        double m_thresMin = 0.12;
        double m_thresMax = 0.97;

        m_octoMap = new OcTreeROS(m_res);
        m_octoMap->octree.setProbHit(m_probHit);
        m_octoMap->octree.setProbMiss(m_probMiss);
        m_octoMap->octree.setClampingThresMin(m_thresMin);
        m_octoMap->octree.setClampingThresMax(m_thresMax);
        has_octo = true;
    }

}

void TableTopObject::addPointCloud(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box)
{
    initOcto();

    for (size_t i = 0; i < cloud_in_box->points.size(); i++)
    {
        cloud->points.push_back(cloud_in_box->points[i]);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZRGB>);

    projectToPlanePerspective(sensorOrigin, tableHeight ,cloud_in_box, cloud_in_box_projected);

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


}


bool TableTopObject::checkCollision(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject)
{

    tf::Transform resultingTransform = otherTransform.inverseTimes(ownTransform);

    geometry_msgs::Transform trans;

    //tf::transformTFToMsg(resultingTransform, trans);
    //std::cout << "resulting transform :" << trans << std::endl;

    for (OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(16),
            end = m_octoMap->octree.end(); it != end; ++it)
    {

        tf::Vector3 vec(it.getX(), it.getY(), it.getZ());

        vec = resultingTransform * vec;

        octomath::Vector3 coord;

        coord.x() = vec.x();
        coord.y() = vec.y();
        coord.z() = vec.z();

        octomap::OcTreeKey key;

        // can this happen ?
        if (!otherObject.m_octoMap->octree.genKey(coord, key))
            continue;

        octomap::OcTreeNode *node = otherObject.m_octoMap->octree.search(key);

        if (node && otherObject.m_octoMap->octree.isNodeOccupied(node))
            return true;

    }
    return false;
}

bool TableTopObject::checkCollisionPointcloud(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject)
{

    initOcto();

    tf::Transform resultingTransform = otherTransform.inverseTimes(ownTransform);

    geometry_msgs::Transform trans;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {

        tf::Vector3 vec(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        vec = resultingTransform * vec;

        octomath::Vector3 coord;

        coord.x() = vec.x();
        coord.y() = vec.y();
        coord.z() = vec.z();

        octomap::OcTreeKey key;

        // can this happen ?
        if (!otherObject.m_octoMap->octree.genKey(coord, key))
            continue;

        octomap::OcTreeNode *node = otherObject.m_octoMap->octree.search(key);

        if (node && otherObject.m_octoMap->octree.isNodeOccupied(node))
            return true;

    }
    return false;
}

bool TableTopObject::checkCoveredPointcloud(tf::Transform ownTransform, tf::Transform otherTransform, TableTopObject &otherObject)
{

    //if (!has_octo)
      //  return false;

    initOcto();

    tf::Transform resultingTransform = otherTransform.inverseTimes(ownTransform);

    geometry_msgs::Transform trans;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {

        tf::Vector3 vec(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        vec = resultingTransform * vec;

        octomath::Vector3 coord;

        coord.x() = vec.x();
        coord.y() = vec.y();
        coord.z() = vec.z();

        octomap::OcTreeKey key;

        // this can happen when we have an empty octree, which would of course not cover the object, so return false
        if (!otherObject.m_octoMap->octree.genKey(coord, key)) {
			ROS_ERROR("genKey inside checkCoveredPointcloud returned false");
            return false;
		}

        octomap::OcTreeNode *node = otherObject.m_octoMap->octree.search(key);

        if (!node)
            return false;
        //if (node && (!otherObject.m_octoMap->octree.isNodeOccupied(node)))
        //  return false;

    }
    return true;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr TableTopObject::getAsCloud()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(16),
            end = m_octoMap->octree.end(); it != end; ++it)
    {
        if (m_octoMap->octree.isNodeOccupied(*it))
        {

            pcl::PointXYZRGB pt;

            pt.x = it.getX();
            pt.y = it.getY();
            pt.z = it.getZ();

            cloud->points.push_back(pt);
        }

    }
    return cloud;
}
