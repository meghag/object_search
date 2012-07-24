#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

using namespace std;
using namespace pcl;

//float frgb

void
extract_color_clusters (const PointCloud<PointXYZRGB> &cloud, 
			const std::vector<int> &indices,
			const boost::shared_ptr<KdTree<PointXYZRGB> > &tree,
			float tolerance, std::vector<PointIndices> &clusters,
			unsigned int min_pts_per_cluster, 
			unsigned int max_pts_per_cluster)
{
  cerr << "Entered extract " << endl;
  /*
    for (size_t j = 0; j < indices.size (); ++j)
    cerr << indices[j] << "  ";
    cerr<<endl;
  */
  
  // note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
  //and indices[i]
  if (tree->getInputCloud()->points.size() != cloud.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", (unsigned long)tree->getInputCloud ()->points.size (), (unsigned long)cloud.points.size ());
      return;
    }

  //cerr << "Passed no. of points check " << endl;
  //cerr << "No. of indices" << indices.size() << endl;
  //cerr << "No. of tree indices " << tree->getIndices()->size() <<  endl;

  
  if (tree->getIndices ()->size () != indices.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%lu) than the input set (%lu)!\n", (unsigned long)tree->getIndices ()->size (), (unsigned long)indices.size ());
      return;
    }

  //cerr << "Passed no. of indices check " << endl;
  
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (indices.size (), false);
 
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  PointCloud<PointXYZRGB> cloud_copy = cloud;
  uint32_t rgb[2];
  uint8_t r[2],g[2],b[2];
  uint8_t max_col[2];

  // Process all points in the indices vector
  for (size_t i = 0; i < indices.size (); ++i)
    {
      if (processed[i])
	continue;
 
      std::vector<int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (i);
 
      processed[i] = true;
 
      while (sq_idx < (int)seed_queue.size ())
	{
	  // Search for sq_idx
	  if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
	    {
	      //cerr << "sq_idx = "<<sq_idx << ", Q size = " << (int)seed_queue.size() << ", no neighbors found"<<endl;
	      sq_idx++;
	      continue;
	    }
	  //cerr << "Did radius search " << endl;
 
	  for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
	    {
	      if (processed[nn_indices[j]])                             // Has this point been processed before ?
		continue;
 
	      // Perform a simple Euclidean clustering
	      // This is where the change is made for color
	      //cerr << abs(cloud.points[seed_queue[sq_idx]].rgb - cloud.points[nn_indices[j]].rgb) << endl;
	      rgb[0] = *reinterpret_cast<int*>(&cloud_copy.points[seed_queue[0]].rgb);
	      rgb[1] = *reinterpret_cast<int*>(&cloud_copy.points[nn_indices[j]].rgb);
	      for (int q = 0; q < 2; q++) {
		r[q] = ((rgb[q] >> 16) & 0x0000ff);
		g[q] = ((rgb[q] >> 8)  & 0x0000ff);
		b[q] = ((rgb[q])       & 0x0000ff);
		//ROS_INFO("PP: RGB %d : %d, %d, %d",q+1,r[q],g[q],b[q]);
		max_col[q] = std::max(r[q],g[q]);
		max_col[q] = std::max(max_col[q],b[q]);
	      }
	      if (max_col[0] == r[0] && max_col[1] == r[1]) {
		seed_queue.push_back (nn_indices[j]);
		processed[nn_indices[j]] = true;
	      } else if (max_col[0] == g[0] && max_col[1] == g[1]) {
		seed_queue.push_back (nn_indices[j]);
		processed[nn_indices[j]] = true;
	      } else if (max_col[0] == b[0] && max_col[1] == b[1]) {
		seed_queue.push_back (nn_indices[j]);
		processed[nn_indices[j]] = true;
	      } 	
	      /*	
			if (abs(cloud.points[seed_queue[sq_idx]].rgb - cloud.points[nn_indices[j]].rgb) < 0.05e-38) {
			seed_queue.push_back (nn_indices[j]);
			//}
			processed[nn_indices[j]] = true;
			}
	      */
	    }
 
	  sq_idx++;
	}
      //cerr << "Q size: "<< seed_queue.size() << endl;

      /*
	for (size_t j = 0; j < seed_queue.size (); ++j)
	cerr << seed_queue[j] << "  ";
	cerr<<endl;
      */
	 
      //cerr << "Compiling cluster " << endl;
      // If this queue is satisfactory, add to the clusters
      if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
	{
	  pcl::PointIndices r;
	  r.indices.resize (seed_queue.size ());

	  //cerr<<"r size = " << r.indices.size()<<endl;
	   
	  for (size_t j = 0; j < seed_queue.size (); ++j)
	    // This is the only place where indices come into play
	    r.indices[j] = indices[seed_queue[j]];

	  /*
	    for (size_t j = 0; j < r.indices.size (); ++j)
	    cerr << r.indices[j] << " ";
	    cerr<<endl;
	  */
	  //r.indices.assign(seed_queue.begin(), seed_queue.end());
	  sort (r.indices.begin (), r.indices.end ());
	  r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
 
	  r.header = cloud.header;
	  // ROS_INFO("Time stamp & frame id of cluster: %d, %s\n",cloud.header.stamp.sec, cloud.header.frame_id.c_str());

	  clusters.push_back (r);   // We could avoid a copy by working directly in the vector
	}
    }
}
