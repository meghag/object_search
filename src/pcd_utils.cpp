#include "pcd_utils.h"

int pass_through_gen(pcl::PointCloud<PointT>::Ptr& pcd_orig,
                 pcl::PointCloud<PointT>::Ptr& pcd_filtered,
                 bool filterx, float xmin, float xmax, bool filtery, float ymin, float ymax,
                 bool filterz, float zmin, float zmax)
{
	pcl::PointCloud<PointT>::Ptr cloudx(new pcl::PointCloud<PointT>);	//Filtered cloud
	pcl::PointCloud<PointT>::Ptr cloudy(new pcl::PointCloud<PointT>);	//Filtered cloud

	ROS_INFO("Inside pass through.");
	ROS_INFO("pass thru before filtering: %zu data points", pcd_orig->points.size());
//	for (size_t i = 0; i < 10; ++i)
//		ROS_INFO("\t%f\t%f\t%f",pcd_orig->points[i].x, pcd_orig->points[i].y, pcd_orig->points[i].z);

	/****************** Filter out the non-table points ******************/
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (pcd_orig);
	if (filterx) {
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (xmin, xmax);
		//  pass.setFilterLimitsNegative (true);
		pass.filter (*pcd_filtered);
	} else
		pcd_filtered = pcd_orig;
	if (filtery) {
		pass.setInputCloud (pcd_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (ymin, ymax);
		//  pass.setFilterLimitsNegative (true);
		pass.filter (*pcd_filtered);
	} else
		pcd_filtered = pcd_orig;
	if (filterz) {
		pass.setInputCloud (pcd_filtered);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (zmin, zmax);
		//  pass.setFilterLimitsNegative (true);
		pass.filter (*pcd_filtered);
	}

	ROS_INFO("Filtered cloud size: %zu", pcd_filtered->points.size());

  return (0);
}

int planar_seg(pcl::PointCloud<PointT>::Ptr orig_cloud,
               pcl::PointCloud<PointT>::Ptr p_cloud,
               pcl::PointCloud<PointT>::Ptr o_cloud,
               string fname1, string fname2)
{
  /******************** Planar Segmentation ***************************/

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  //PCDWriter writer;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  Eigen::Vector3f z_axis(0.0, 0.0, 1.0);			//for horizontal planes
  seg.setAxis(z_axis);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  seg.setInputCloud (orig_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

 // Extract the inliers
    extract.setInputCloud (orig_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*p_cloud);
    std::cerr << "PointCloud representing the planar component: " << p_cloud->width * p_cloud->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*o_cloud);
  //pcl::io::savePCDFileASCII ((string)fname1, *p_cloud);
  //std::cerr << "Saved " << p_cloud->points.size () << " data points to " << fname1 << std::endl;

  pcl::io::savePCDFileASCII ((string)fname2, *o_cloud);
  std::cerr << "Saved " << o_cloud->points.size () << " data points to " << fname2 << std::endl;

  return (0);
}

geometry_msgs::Point find_centroid(pcl::PointCloud<PointT> pcd)
{
	ROS_INFO("Inside find_centroid");
	double avgx = 0.0, avgy = 0.0, avgz = 0.0;
	geometry_msgs::Point position;
	for (std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it = pcd.points.begin();
			it != pcd.points.end(); ++it) {
		avgx += it->x;      avgy += it->y;      avgz += it->z;
	}
	position.x = avgx/pcd.points.size();
	position.y = avgy/pcd.points.size();
	position.z = avgz/pcd.points.size();

	return position;
} //End function find_centroid

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<PointT> pcd)
		  {
	ROS_INFO("Inside find_extents");
	vector<double> vecx, vecy, vecz;

	for (std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it = pcd.points.begin();
			it != pcd.points.end(); ++it) {
		vecx.push_back(it->x);       vecy.push_back(it->y);       vecz.push_back(it->z);
	}

	vector<double>::iterator it_minx, it_miny, it_minz, it_maxx, it_maxy, it_maxz;
	it_minx = min_element(vecx.begin(), vecx.end());
	it_miny = min_element(vecy.begin(), vecy.end());
	it_minz = min_element(vecz.begin(), vecz.end());
	it_maxx = max_element(vecx.begin(), vecx.end());
	it_maxy = max_element(vecy.begin(), vecy.end());
	it_maxz = max_element(vecz.begin(), vecz.end());

	PointT pt_minx, pt_miny, pt_minz, pt_maxx, pt_maxy, pt_maxz;
	vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it2 = pcd.points.begin();

	for (vector<double>::iterator pos = vecx.begin(); pos != vecx.end(); ++pos) {
		if (pos == it_minx) pt_minx = *it2;
		if (pos == it_maxx) pt_maxx = *it2;
		++it2;
	}

	it2 = pcd.points.begin();
	for (vector<double>::iterator pos = vecy.begin(); pos != vecy.end(); ++pos) {
		if (pos == it_miny) pt_miny = *it2;
		if (pos == it_maxy) pt_maxy = *it2;
		++it2;
	}

	it2 = pcd.points.begin();
	for (vector<double>::iterator pos = vecz.begin(); pos != vecz.end(); ++pos) {
		if (pos == it_minz) pt_minz = *it2;
		if (pos == it_maxz) pt_maxz = *it2;
		++it2;
	}

	geometry_msgs::Point vertices[6];
	vertices[0].x = pt_minx.x;    vertices[0].y = pt_minx.y;    vertices[0].z = pt_minx.z;
	vertices[1].x = pt_maxx.x;    vertices[1].y = pt_maxx.y;    vertices[1].z = pt_maxx.z;
	vertices[2].x = pt_miny.x;    vertices[2].y = pt_miny.y;    vertices[2].z = pt_miny.z;
	vertices[3].x = pt_maxy.x;    vertices[3].y = pt_maxy.y;    vertices[3].z = pt_maxy.z;
	vertices[4].x = pt_minz.x;    vertices[4].y = pt_minz.y;    vertices[4].z = pt_minz.z;
	vertices[5].x = pt_maxz.x;    vertices[5].y = pt_maxz.y;    vertices[5].z = pt_maxz.z;

	std::vector<geometry_msgs::Point> extent;
	for (int i = 0; i < 6; i++)
		extent.push_back(vertices[i]);

	return extent;
		  } //End function find_extents

char find_color(pcl::PointCloud<PointXYZRGB> pcd)
{
	size_t num_pts = pcd.points.size();
	uint32_t rsum = 0,gsum = 0,bsum = 0;
	uint8_t r,g,b;
	for (size_t i = 0; i < num_pts; i++) {
		uint32_t rgb = *reinterpret_cast<int*>(&pcd.points[i].rgb);
		rsum = rsum + ((rgb >> 16) & 0x0000ff);
		gsum = gsum + ((rgb >> 8)  & 0x0000ff);
		bsum = bsum + ((rgb)       & 0x0000ff);
	}
	r = std::floor(rsum/num_pts);
	g = std::floor(gsum/num_pts);
	b = std::floor(bsum/num_pts);
	ROS_INFO("Avg RGB of cluster: %d, %d, %d",r,g,b);
	uint8_t max_col = std::max(r,g);
	max_col = std::max(max_col,b);
	if (max_col == r) {
		ROS_INFO("The point cloud is predominantly red.");
		return 'r';
	} else if (max_col == g) {
		ROS_INFO("The point cloud is predominantly green.");
		return 'g';
	} else if (max_col == b) {
		ROS_INFO("The point cloud is predominantly blue.");
		return 'b';
	}
	return 'r';
}

char find_color(sensor_msgs::PointCloud2 pcd2)
{
	pcl::PointCloud<PointXYZRGB> pcd;
	fromROSMsg(pcd2, pcd);
	return find_color(pcd);
}

void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<PointT>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( *cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

sensor_msgs::PointCloud2 concatClouds(vector<sensor_msgs::PointCloud2>& clouds)
{
	sensor_msgs::PointCloud2 concat_cloud2;
	if (clouds.size() == 0){
		ROS_ERROR("Empty vector sent to be concatenated.");
		return concat_cloud2;
	} else if (clouds.size() == 1)
		return clouds[0];

	pcl::PointCloud<PointT>::Ptr concat_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(clouds[0], *concat_cloud);
	for (size_t i = 1; i < clouds.size(); i++)
	{
		pcl::PointCloud<PointT> temp_cloud;
		fromROSMsg(clouds[i], temp_cloud);
		*concat_cloud += temp_cloud;
	}
	toROSMsg(*concat_cloud, concat_cloud2);
	concat_cloud2.header.frame_id = "base_link";
	return concat_cloud2;
}

void cluster(sensor_msgs::PointCloud2& cloud2,
		float cluster_tolerance,
		int min_cluster_size,
		int max_cluster_size,
		vector<sensor_msgs::PointCloud2>& clusters)
{
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	fromROSMsg(cloud2, *cloud);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (cluster_tolerance); // 2cm
	ec.setMinClusterSize (min_cluster_size);	//300
	ec.setMaxClusterSize (max_cluster_size);	//10000
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	ROS_INFO("Found %zu clusters", cluster_indices.size());

	int j = 0;
	clusters.resize(cluster_indices.size());
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		sensor_msgs::PointCloud2 cluster2;
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]);
		ROS_INFO("Cluster %d has %zu points", j, cloud_cluster->points.size());
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		toROSMsg(*cloud_cluster, cluster2);
		clusters[j] = cluster2;
		j++;
	}
}

bool touchesTable(pcl::PointCloud<PointT> cloud, double table_height)
{
	//ROS_DEBUG("Inside touchesTable");
	unsigned int count = 0;
	//for (PointCloud<PointT>::iterator it = cloud.points.begin(); it != cloud.points.end(); it++)
	for(vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it = cloud.points.begin(); it != cloud.points.end(); ++it)
	{
		if (abs(it->z - table_height) < 0.02)
			//Close enough to table
			count++;
	}

	if (count > 1)	//Should be a % of cloud size instead of fixed at 50
	{
		ROS_DEBUG("Yes, it does!");
		return true;
	}
	else {
		ROS_DEBUG("No, it doesn't!");
		return false;
	}
}

bool incontact(sensor_msgs::PointCloud2 cloud2_1, sensor_msgs::PointCloud2 cloud2_2,
		float cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
	pcl::PointCloud<PointT> cloud1, cloud2;
	fromROSMsg(cloud2_1, cloud1);
	fromROSMsg(cloud2_2, cloud2);

	//Concatenate the 2 clouds
	pcl::PointCloud<PointT> cloud_concat = cloud1;
	cloud_concat += cloud2;
	pcl::PointCloud<PointT>::Ptr concat(new PointCloud<PointT>);
	*concat = cloud_concat;

	//Do Euclidean clustering and find number of clusters
	//Electric: pcl::KdTree<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB>);
	// Fuerte:
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (concat);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (cluster_tolerance);
	ec.setMinClusterSize (min_cluster_size);
	ec.setMaxClusterSize (max_cluster_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud(concat);
	ec.extract (cluster_indices);
	if (cluster_indices.size() <= 1)
		//The two clusters are closer than 4 cm
		return true;
	else
		return false;
}
