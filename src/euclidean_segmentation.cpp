/*********************************************
This program reads in a point cloud from the specified pcd file
and extracts clusters out of it based on euclidean clustering.

Date created: Aug 14, 2012
Author: Megha Gupta

***********************************************/


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

#include "tum_os/Clusters.h"

using namespace std;
using namespace pcl;

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;
//typedef std::vector<pcl::PointCloud<PointXYZRGB> > pcd_vector;

class EuclideanSegmenter
{
public:
    EuclideanSegmenter(ros::NodeHandle & n, int i): n_(n) {
        clusters_pub_ = n_.advertise<tum_os::Clusters>("clusters",1);
        //toProcessPub_ = n_.advertise<tum_os::toProcessMsg>("to_process",1);
        //pcd_sub_.subscribe(n_, cloud_topic, 1);

        if (i == 0) {
            //Read object cloud from a PCD file
            pcl::PointCloud<PointT>::Ptr object_cloud(new pcl::PointCloud<PointT>);
            if (pcl::io::loadPCDFile<PointT> ("object_cloud.pcd", *object_cloud) == -1) {
                ROS_ERROR("Couldn't read file.");
                return;
            }
            ROS_INFO("Loaded %d data points from pcd file.", object_cloud->width * object_cloud->height);
            objectCloud_ = *object_cloud;
        } else {
            //Read object cloud from a topic
            seg_sub_ = n_.subscribe("/object_cloud", 1, &EuclideanSegmenter::callback, this);
        }
    };

private:
    ros::NodeHandle n_;
    ros::Subscriber seg_sub_;
    ros::Publisher clusters_pub_;
    //ros::Publisher object_pub_;
    //ros::Publisher toProcessPub_;
    pcl::PointCloud<PointT> objectCloud_;

    void callback(const sensor_msgs::PointCloud2::ConstPtr& pcd2) {
        ROS_INFO("Inside Euclidean Segmentation callback.");
        pcl::PointCloud<PointT> cloud;
        fromROSMsg(*pcd2,cloud);
        objectCloud_ = cloud;
        cluster();
    };

    void cluster() {
        pcl::PCDWriter writer;
        pcl::PointCloud<PointT>::Ptr object_cloud(new pcl::PointCloud<PointT>);
        *object_cloud = objectCloud_;
        // Creating the KdTree object for the search method of the extraction
        KdTree<PointT>::Ptr tree (new KdTreeFLANN<PointT>);
        vector<PointIndices> cluster_indices;

        ROS_INFO("Object cloud has %zu points", object_cloud->points.size());
        vector<int> temp;
        for (size_t j = 0; j < object_cloud->points.size(); ++j)
            temp.push_back(j);

        boost::shared_ptr<const vector<int> > indices (new vector<int> (temp));
        tree->setInputCloud (object_cloud, indices);
        euclidean_clustering(*object_cloud, *indices, tree, 0.01, cluster_indices, 2000, 10000);

        int j = 0;
        std::vector<sensor_msgs::PointCloud2> clusters;
        for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            PointCloud<PointT>::Ptr cloud_cluster(new PointCloud<PointT>);
            sensor_msgs::PointCloud2 cluster2;

            /*************** Separating out and saving each cluster ***************/
            for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back(object_cloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            ROS_INFO("Point Cloud representing the Cluster: %zu data points", cloud_cluster->points.size());
            toROSMsg(*cloud_cluster, cluster2);
            clusters.push_back(cluster2);
            /*	stringstream ss;
            ss << "cluster_" << j << ".pcd";
            writer.write (ss.str(), *cloud_cluster, false);
            */
            j++;
        }

        tum_os::Clusters c;
        c.num_clusters = j;
        c.clusters = clusters;
        sensor_msgs::PointCloud2 oc2;
        toROSMsg(*object_cloud, oc2);
        c.object_cloud = oc2;

        clusters_pub_.publish(c);
    };

    void euclidean_clustering (const PointCloud<PointT> &cloud, const vector<int> &indices,
                               const boost::shared_ptr<KdTree<PointT> > &tree,
                               float tolerance, vector<PointIndices> &clusters,
                               unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster) {
        //If the tree was created over <cloud,indices>, we guarantee a 1-1 mapping
        //between what the tree returns and indices[i]
        if (tree->getInputCloud()->points.size() != cloud.points.size ()) {
            ROS_ERROR("[Clusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!",
                      (unsigned long)tree->getInputCloud ()->points.size (), (unsigned long)cloud.points.size ());
            return;
        }

        if (tree->getIndices ()->size () != indices.size ()) {
            ROS_ERROR("[Clusters] Tree built for a different set of indices (%lu) than the input set (%lu)!",
                      (unsigned long)tree->getIndices ()->size (), (unsigned long)indices.size ());
            return;
        }

        // Create a bool vector of processed point indices, and initialize it to false
        std::vector<bool> processed (indices.size (), false);
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        PointCloud<PointT> cloud_copy = cloud;
        //uint32_t rgb[2];
        //uint8_t r[2], g[2], b[2], max_col[2];

        // Process all points in the indices vector
        for (size_t i = 0; i < indices.size (); ++i) {
            if (processed[i])  continue;

            std::vector<int> seed_queue;
            int sq_idx = 0;
            seed_queue.push_back (i);
            processed[i] = true;

            while (sq_idx < (int)seed_queue.size ()) {
                // Search for sq_idx
                if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances)) {
                    sq_idx++;
                    continue;
                }

                for (size_t j = 1; j < nn_indices.size (); ++j) {    // nn_indices[0] should be sq_idx
                    if (processed[nn_indices[j]])                      // Has this point been processed before ?
                        continue;

                    seed_queue.push_back (nn_indices[j]);
                    processed[nn_indices[j]] = true;
                 }
                sq_idx++;
            }

            // If this queue is satisfactory, add to the clusters
            if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster) {
                pcl::PointIndices r;
                r.indices.resize (seed_queue.size ());

                for (size_t j = 0; j < seed_queue.size (); ++j)
                    // This is the only place where indices come into play
                    r.indices[j] = indices[seed_queue[j]];

                //r.indices.assign(seed_queue.begin(), seed_queue.end());
                sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
                r.header = cloud.header;
                clusters.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        }
    };

};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_seg");
    if (argc != 2) {
        ROS_ERROR("Usage: rosrun tum_os color_seg <option>");
        ROS_ERROR("\t \t option = 0 for reading object cloud from pcd file");
        ROS_ERROR("\t \t option = 1 for reading object cloud from a topic");
        return (-1);
    }

    ros::NodeHandle n;
    EuclideanSegmenter cs(n,atoi(argv[1]));
    ros::spin();
    return 0;
}



