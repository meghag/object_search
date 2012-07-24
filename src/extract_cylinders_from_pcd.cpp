#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointT;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *object_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded " << object_cloud->width * object_cloud->height << " data points from pcd filed " << std::endl;
  
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  //pcl::PointCloud<PointT>::Ptr remaining_cloud (new pcl::PointCloud<PointT> ());

  int i = 1;
  while (1) {
    std::stringstream ss;
      ss << "oc" << i << ".pcd";
      std::cerr << "PointCloud representing the remaining cloud component: " << object_cloud->points.size () << " data points." << std::endl;
      writer.write (ss.str(), *object_cloud, false);
      // Estimate point normals
      ne.setSearchMethod (tree);
      ne.setInputCloud (object_cloud);
      ne.setKSearch (50);
      ne.compute (*cloud_normals);

      // Create the segmentation object for cylinder segmentation and set all the parameters
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_CYLINDER);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight (0.1);
      seg.setMaxIterations (10000);
      seg.setDistanceThreshold (0.02);
      seg.setRadiusLimits (0.01, 0.05);
      seg.setInputCloud (object_cloud);
      seg.setInputNormals (cloud_normals);
      Eigen::Vector3f v;
      v << 0.0f, 0.0f, 1.0f;
      //      v[2] = 1.0f;
      seg.setAxis(v);
      seg.setEpsAngle(0.1);

      // Obtain the cylinder inliers and coefficients
      seg.segment (*inliers_cylinder, *coefficients_cylinder);
      std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

      // Write the cylinder inliers to disk
      extract.setInputCloud (object_cloud);
      extract.setIndices (inliers_cylinder);
      extract.setNegative (false);
      extract.filter (*cloud_cylinder);
      if (cloud_cylinder->points.empty ())  {
	std::cerr << "Can't find the cylindrical component." << std::endl;
	break;
      } else {
	std::stringstream ss;
	ss << "cylinder" << i << ".pcd";
	std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	writer.write (ss.str(), *cloud_cylinder, false);
      }
      extract.setNegative (true);
      extract.filter(*object_cloud);

      extract_normals.setNegative (true);
      extract_normals.setInputCloud (cloud_normals);
      extract_normals.setIndices (inliers_cylinder);
      extract_normals.filter (*cloud_normals);
      i++;
  }

  return (0);
}
