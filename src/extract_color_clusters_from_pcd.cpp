#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

void extract_color_clusters (const PointCloud<PointXYZRGB> &cloud, 
                               const std::vector<int> &indices,
                                const boost::shared_ptr<KdTree<PointXYZRGB> > &tree,
                                float tolerance, std::vector<PointIndices> &clusters,
                                unsigned int min_pts_per_cluster, 
                                unsigned int max_pts_per_cluster);

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *object_cloud) == -1) //* load the file
  {
    ROS_ERROR("Couldn't read file.");
    return (-1);
  }
  ROS_INFO("Loaded %d data points from pcd file.", object_cloud->width * object_cloud->height);
  // ROS_INFO("height =  %d, width = %d", object_cloud->height, object_cloud->width);
  
  pcl::PCDWriter writer;
  
  // Creating the KdTree object for the search method of the extraction
  KdTree<PointXYZRGB>::Ptr tree (new KdTreeFLANN<PointXYZRGB>);
  vector<PointIndices> cluster_indices;
  
  vector<int> temp;
  for (size_t j = 0; j < object_cloud->points.size(); ++j)
	temp.push_back(j);
  boost::shared_ptr<const vector<int> > indices (new vector<int> (temp)); 
  
  tree->setInputCloud (object_cloud, indices);
  extract_color_clusters(*object_cloud, *indices, tree, 0.03, cluster_indices, 1000, 8000);

  string fields = getFieldsList(*object_cloud);
  cerr << fields << ", " << fields.size() << endl;
  
  int j = 1;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloud<PointXYZRGB>::Ptr cloud_cluster(new PointCloud<PointXYZRGB>);
    /*************** Separating out and saving each cluster ***************/
    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	  cloud_cluster->points.push_back(object_cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    ROS_INFO("Point Cloud representing the Cluster: %zu data points", cloud_cluster->points.size());
    //ROS_INFO("Cluster height = %d, width = %d", cloud_cluster->height, cloud_cluster->width);

    stringstream ss;
    ss << "cluster_" << j << ".pcd";
    writer.write (ss.str(), *cloud_cluster, false); 
    j++;
  }

  return (0);
}
