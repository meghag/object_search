#include <TableTopObject.h>

using namespace octomap;

TableTopObject::TableTopObject()
{
    cloud.reset(new pcl::PointCloud<PointT>);
    has_octo = false;
}


TableTopObject::TableTopObject(pcl::PointCloud<PointT>::Ptr cloud_in_box)
{
    cloud.reset(new pcl::PointCloud<PointT>);
    cloud = cloud_in_box;
    has_octo = false;
}


TableTopObject::TableTopObject(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT>::Ptr cloud_in_box)
{
    // is that necessary?
    cloud.reset(new pcl::PointCloud<PointT>);
    has_octo = false;
    addPointCloud(sensorOrigin,tableHeight,cloud_in_box);
}

TableTopObject::TableTopObject(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT> cloud_in_box)
{
    // is that necessary?
    cloud.reset(new pcl::PointCloud<PointT>);
    has_octo = false;
    addPointCloud2(sensorOrigin,tableHeight,cloud_in_box);
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
		//m_octoMap_copy = new OcTreeROS(m_res);
        m_octoMap->octree.setProbHit(m_probHit);
        m_octoMap->octree.setProbMiss(m_probMiss);
        m_octoMap->octree.setClampingThresMin(m_thresMin);
        m_octoMap->octree.setClampingThresMax(m_thresMax);
        has_octo = true;
    }

}

void TableTopObject::projectToPlanePerspective(const tf::Vector3 sensorOrigin, const double tableHeight, const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_projected)
{
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        tf::Vector3 pt(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
        tf::Vector3 fromSens = pt - sensorOrigin;
        float factor = (tableHeight - sensorOrigin.z()) / fromSens.z();
        fromSens *= factor;
        fromSens += sensorOrigin;
        PointT newpt = cloud->points[i];
        newpt.x = fromSens.x();
        newpt.y = fromSens.y();
        newpt.z = fromSens.z();
        cloud_projected->points.push_back(newpt);
    }
}

void TableTopObject::projectToPlanePerspective2(const tf::Vector3 sensorOrigin, const double tableHeight, const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>::Ptr &cloud_projected)
{
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        tf::Vector3 pt(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
        tf::Vector3 fromSens = pt - sensorOrigin;
        float factor = (tableHeight - sensorOrigin.z()) / fromSens.z();
        fromSens *= factor;
        fromSens += sensorOrigin;
        PointT newpt = cloud.points[i];
        newpt.x = fromSens.x();
        newpt.y = fromSens.y();
        newpt.z = fromSens.z();
        cloud_projected->points.push_back(newpt);
    }
}

void TableTopObject::addPointCloud(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT>::Ptr cloud_in_box)
{
    initOcto();

    for (size_t i = 0; i < cloud_in_box->points.size(); i++)
    {
        cloud->points.push_back(cloud_in_box->points[i]);
		//cloud_copy->points.push_back(cloud_in_box->points[i]);
    }

    pcl::PointCloud<PointT>::Ptr cloud_in_box_projected (new pcl::PointCloud<PointT>);

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

	//*m_octoMap_copy = *m_octoMap;

}

void TableTopObject::addPointCloud2(const tf::Vector3 sensorOrigin, const double tableHeight, pcl::PointCloud<PointT> cloud_in_box)
{
    initOcto();

    for (size_t i = 0; i < cloud_in_box.points.size(); i++)
    {
        cloud->points.push_back(cloud_in_box.points[i]);
		//cloud_copy->points.push_back(cloud_in_box.points[i]);
    }

    pcl::PointCloud<PointT>::Ptr cloud_in_box_projected(new pcl::PointCloud<PointT>);

    projectToPlanePerspective2(sensorOrigin, tableHeight, cloud_in_box, cloud_in_box_projected);

    octomap::KeySet occupied_cells;

    for (size_t i = 0; i < cloud_in_box.points.size(); ++i)
    {
        octomap::KeyRay ray;
        octomath::Vector3 start;
        start.x() = cloud_in_box.points[i].x;
        start.y() = cloud_in_box.points[i].y;
        start.z() = cloud_in_box.points[i].z;
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

	//*m_octoMap_copy = *m_octoMap;

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
        if (!otherObject.m_octoMap->octree.genKey(coord, key))
            return false;

        octomap::OcTreeNode *node = otherObject.m_octoMap->octree.search(key);

        if (!node)
            return false;
        //if (node && (!otherObject.m_octoMap->octree.isNodeOccupied(node)))
        //  return false;

    }
    return true;
}

pcl::PointCloud<PointT>::Ptr TableTopObject::getAsCloud()
{

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    for (OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(16),
            end = m_octoMap->octree.end(); it != end; ++it)
    {
        if (m_octoMap->octree.isNodeOccupied(*it))
        {

            PointT pt;

            pt.x = it.getX();
            pt.y = it.getY();
            pt.z = it.getZ();

            cloud->points.push_back(pt);
        }

    }
    return cloud;
}
