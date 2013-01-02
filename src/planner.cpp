#include "planner.h"

using namespace pcl;
using namespace octomap;
using namespace std;

Planner::Planner (ros::NodeHandle & n, octomap::OcTree& tree): n_(n)
{
	//planActionsServer_ = n_.advertiseService("plan_actions", &DataProcessor::planActionsCallback, this);
	ROS_INFO("Ready to plan actions.");

	octreePub_ = n.advertise<visualization_msgs::MarkerArray>("current_octree_array",100);
	raycastPub_ = n.advertise<visualization_msgs::MarkerArray>("raycast_array",200);
	objectCloudPub_ = n.advertise<sensor_msgs::PointCloud2>("object_cloud",1);

	visibleObjectsSub_ = n_.subscribe("visible_objects", 1, &Planner::visibleObjectsCallback, this);	
	octreeSub_ = n_.subscribe("octree",1, &Planner::octreeCallback, this);

	//tree_ = tree(treeResolution_);
}

Planner::~Planner (void)
{

}

void Planner::octreeCallback (const object_search_pkg::octree::ConstPtr& tree)
{
	tree_ = tree;
}

void Planner::samplePose(TableTopObject& current_config_tto, pcl::PointCloud<PointXYZ> object_cloud, 
						 std::vector<tf::Pose>& object_posterior_belief)
{
	//This function samples the whole space for valid poses of the object_cloud.
	//Then it checks whether they lie in the free space and returns an object belief.
	
	ROS_INFO("before creating samples");
    std::vector<tf::Pose> object_belief;
    for (int k =0; k < 100000; k++)
    {
        object_belief.push_back(vdc_pose_bound(bb_min,bb_max,k));
    }
    ROS_INFO("samples created");
	
    //pub_belief("object_belief", object_belief);
	
    for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it!=object_belief.end(); it++)
    {
        if (checkCoveredPointcloud(*it, identity, current_config_tto, object_cloud))
            object_posterior_belief.push_back(*it);
    }
	
    ROS_INFO("samples checked");
	
    std::cout << "size of object belief " << object_posterior_belief.size() << std::endl;
	
    //pub_belief("object_posterior_belief",object_posterior_belief);
}

bool Planner::checkCoveredPointcloud(tf::Transform ownTransform, tf::Transform otherTransform, 
									 TableTopObject &otherObject, pcl::PointCloud<PointXYZ> cloud)
{
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

bool Planner::generatePercentage(tf::Pose object_belief, 
								 std::vector<PointCloud<PointXYZ> clusters)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr percentage_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//create tabletop representation with one cluster missing at a time
	int max_idx = -1;
	double max_perc = 0;
	std::vector<TableTopObject*> obj_excluding;
	for (size_t i = 0; i < clusters.size(); i++) {
		TableTopObject *act = new TableTopObject();
		for (size_t j = 0; j < clusters.size(); j++) {
			if (j != i)
				act->addPointCloud(sensorsInMap.getOrigin(), bb_min.z(),clusters[j]);
		}
		obj_excluding.push_back(act);
		size_t num_remaining = 0;
		for (std::vector<tf::Pose>::iterator it = object_posterior_belief.begin(); it != object_posterior_belief.end(); it++) {
			if (obj.checkCoveredPointcloud(*it, identity, *act))
				num_remaining++;
		}
		
		double percentage = (object_posterior_belief.size() == 0 ? 1 :
							 (object_posterior_belief.size() - num_remaining)/ (double) object_posterior_belief.size());
		
		std::cout << "Removing Cluster " << i << " would reveal "
		<< object_posterior_belief.size() - num_remaining
		<< " of remaining hypotheses " << " that is "
		<< 100 * percentage << "%" << std::endl;
	}
	return true;
}	

void Planner::plan() 
{
	std::vector<tf::Pose> object_posterior_belief;
	samplePose(current_config_tto, target_object_cloud, object_posterior_belief);
	generatePercentage(object_posterior_belief, clusters);
	//Define bounding box
	tf::Vector3 bb_min(0.5,-0.4,0.69);
    tf::Vector3 bb_max(1.0,0.4,1.0);
	
	tf::Stamped<tf::Pose> sensorsInMap = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());
	
	//Now that we know percentage hidden by each visible object, do the following for each visible object.
	std::map<std::pair<int, float> > hypothesis_revealed_map;
	for (size_t cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++) {
		//Step 1: Sample whole space for upright poses of the visible object
		std::vector<tf:Pose> dest_poses;
		std::vector<tf::Pose> object_belief;
		for (int k =0; k < 100000; k++)
		{
			dest_poses.push_back(vdc_pose_bound(bb_min,bb_max,k));
		}
		ROS_INFO("Samples created for object %zu", cluster_idx);
			
		//Step 2: Discard poses that lie in currently occupied regions
		std::vector<tf::Pose> free_dest_poses;
		ROS_INFO("MG: Creating a TableTopObject (TTO) for object %zu", cluster_idx);
		TableTopObject visibleObject(sensorsInMap.getOrigin(), bb_min.z(), clusters[cluster_idx]);
		
		for (std::vector<tf::Pose>::iterator it = dest_poses.begin(); it!=dest_poses.end(); it++)
		{
			if (visibleObject.checkCoveredPointcloud(*it,identity,*obj_excluding[cluster_idx]))
				free_dest_poses.push_back(*it);
		}
		
		ROS_INFO("Samples checked for object %zu", cluster_idx);
		
		std::cout << "size of free destination poses for object " 
				<< cluster_idx << " = " 
				<< free_dest_poses.size() << std::endl;
		
		//pub_belief("object_posterior_belief",object_posterior_belief);
		
		//Step 3: 
		
		//Step 3: For each destination pose, do the following.
		for (size_t pose_idx = 0; pose_idx < free_dest_poses.size(); pose_idx++) {
			//Step 4: Update visible nodes
			

			//Step 5: Update hidden nodes
			
		}
		std::map<std::pair<int, std::vector<tf::Pose> > > cluster_to_destination_poses_map;
		if (horizon == 0)
			hypothesis_revealed_map.insert(std::make_pair(cluster_idx,0.0));
		else {
			float p = hypothesis_revealed_map[cluster_idx].second() + percentage;
		}
	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle n;
	//octomap::OcTree tree(0.05);
	object_search::Planner planner(n);
	ros::spin();
	return 0;
}

/*

 void Planner::visibleObjectsCallback(
                                      const object_search_pkg::objLocationMap::ConstPtr& now_visible)
									  {
										  map<int, int>::iterator it;
										  //set<int> knownCells;
										  for (it = now_visible->begin(); it != now_visible->end(); it++) {
											  int id = it->first;
											  int cell_idx = it->second;
											  if (knownObjects_.count(id) == 0) {
												  // This object id doesn't exist. This object is a new visible object. Insert in map.
												  knownObjects_.insert(pair<int, int>(id, cell_idx));
												  visibleObjects_.insert(pair<int, int>(id, cell_idx));
												  } else if (visibleObjects_.count(id) == 0) {
													  // This object is known but was not visible in the last step. Insert in visible map.
													  visibleObjects_.insert(pair<int, int>(id, cell_idx));
													  } else {
														  //This object was visible in the last step as well.
														  // Update location in both maps?
														  }
														  knownCells_.insert(cell_idx);
														  }
														  }

 int DataProcessor::findHiddenVoxels (PointCloud<PointXYZ>::Ptr& object)
 {
	 //				findHiddenVoxels(*cloud_cluster, tree, bbx_max,
	 //										  STEREO_ORIGIN, tree_resolution, num_hidden_voxels);


	 //Now find the boundary/corner voxels of the object 
	 geometry_msgs::Point top_left_corner, top_right_corner, end_left_ray, end_right_ray;
	 vector<geometry_msgs::Point> corners = find_corners(object);	
	 top_left_corner = corners[0];
	 top_right_corner = corners[1];
	 end_left_ray = corners[2];
	 end_right_ray = corners[3];

	 point3d back_point1(end_left_ray.x,end_left_ray.y,end_left_ray.z), 
	 back_point2(end_right_ray.x, end_right_ray.y, end_right_ray.z);
	 visualization_msgs::MarkerArray raycast_marker_array;
	 int traversed_node_count = 0;
	 visualization_msgs::Marker one_voxel_marker;

	 //Now do ray tracing
	 std::vector<point3d> ray1, ray2;
	 if (!tree_.computeRay(STEREO_ORIGIN,back_point1,ray1)) {
		 ROS_ERROR("Ray tracing failed. One coordinate probably out of range.");
		 } else {
			 ROS_INFO("Number of nodes traversed by ray 1= %zu", ray1.size());
			 //int traversed_node_count = create_marker_array(ray1, ray1.begin(), raycast_marker_array);
			 for (vector<point3d>::iterator itr = ray1.begin(); itr != ray1.end(); itr++) {
				 geometry_msgs::Point one_voxel;
				 one_voxel.x = itr->x();
				 one_voxel.y = itr->y();
				 one_voxel.z = itr->z();
				 one_voxel_marker = set_marker("base_link","ray_cast_markers",traversed_node_count,
				                               visualization_msgs::Marker::CUBE,one_voxel,0.02,0.0,1.0,0.0,1.0);
											   raycast_marker_array.markers.push_back(one_voxel_marker);
											   traversed_node_count++;
											   }
											   ROS_INFO("Last Traversed node ray 1: x = %f, y = %f, z = %f", ray1[ray1.size()-1].x(), 
											   ray1[ray1.size()-1].y(), ray1[ray1.size()-1].z());
											   //raycast_pub.publish(raycast_marker_array);
											   //ROS_INFO("Published the raycast marker array");
											   }
											   if (!tree_.computeRay(STEREO_ORIGIN,back_point2,ray2)) {
												   ROS_ERROR("Ray tracing failed. One coordinate probably out of range.");
												   } else {
													   ROS_INFO("Number of nodes traversed by ray 2 = %zu", ray2.size());
													   //vector<point3d>::iterator itr;
													   //for (itr = ray2.begin(); itr != ray2.end(); itr++)
													   //ROS_INFO("Traversed node: x = %f, y = %f, z = %f", itr->x(), itr->y(), itr->z());
													   //traversed_node_count+= create_marker_array(ray2, ray2.begin(), raycast_marker_array);
													   for (vector<point3d>::iterator itr = ray2.begin(); itr != ray2.end(); itr++) {
														   geometry_msgs::Point one_voxel;
														   one_voxel.x = itr->x();
														   one_voxel.y = itr->y();
														   one_voxel.z = itr->z();
														   one_voxel_marker = set_marker("base_link","ray_cast_markers",traversed_node_count,
														                                 visualization_msgs::Marker::CUBE,one_voxel,0.02,0.0,1.0,0.0,1.0);
																						 raycast_marker_array.markers.push_back(one_voxel_marker);
																						 traversed_node_count++;
																						 }
																						 ROS_INFO("Last Traversed node ray 2: x = %f, y = %f, z = %f", ray2[ray2.size()-1].x(), 
																						 ray2[ray2.size()-1].y(), ray2[ray2.size()-1].z());
																						 }
																						 raycastPub_.publish(raycast_marker_array);
																						 ROS_INFO("Published the raycast marker array");

																						 //Calculate the longer side of the trapezium hidden 
																						 float trapezium_width1 = abs(ray1[ray1.size()-1].y()-ray2[ray2.size()-1].y());
																						 float trapezium_width2 = abs(top_right_corner.y - top_left_corner.y);
																						 float trapezium_length = abs(top_left_corner.x - end_left_ray.x);
																						 float base_area = 0.5*(trapezium_width1+trapezium_width2)*trapezium_length;
																						 float object_height = top_left_corner.z; 			//*it_maxz;
																						 float hidden_volume = base_area*object_height;
																						 float unit_volume = pow(treeResolution_,3);
																						 int n_hidden_voxels = (int)(hidden_volume/unit_volume);
																						 ROS_INFO("Number of hidden voxels = %d", n_hidden_voxels);


																						 return n_hidden_voxels;

																						 //ROS_INFO("Back inside process.cpp: n_hidden_voxels = %d", num_hidden_voxels);

																						 }


																						 vector<geometry_msgs::Point> DataProcessor::find_corners(PointCloud<PointXYZ>::Ptr& cloud)	
																						 {
																							 vector<double> vecx, vecy, vecz;
																							 vector<geometry_msgs::Point> corners;
																							 //float minx, miny, minz, maxx, maxy, maxz;
																							 for (std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> >::iterator it1 = cloud->points.begin(); it1 != cloud->points.end(); ++it1) {
																								 vecx.push_back(it1->x);
																								 vecy.push_back(it1->y);
																								 vecz.push_back(it1->z);
																								 }

																								 vector<double>::iterator it_minx, it_miny, it_minz, it_maxx, it_maxy, it_maxz;
																								 it_minx = min_element(vecx.begin(), vecx.end());
																								 it_miny = min_element(vecy.begin(), vecy.end());
																								 it_minz = min_element(vecz.begin(), vecz.end());
																								 it_maxx = max_element(vecx.begin(), vecx.end());
																								 it_maxy = max_element(vecy.begin(), vecy.end());
																								 it_maxz = max_element(vecz.begin(), vecz.end());

																								 geometry_msgs::Point top_left_corner, top_right_corner, end_left_ray, end_right_ray;
																								 top_left_corner.x = *it_minx + (*it_maxx - *it_minx)/2;
																								 top_left_corner.y = *it_miny;
																								 top_left_corner.z = *it_maxz;
																								 top_right_corner.x = top_left_corner.x;
																								 top_right_corner.y = *it_maxy;
																								 top_right_corner.z = top_left_corner.z;
																								 end_left_ray.z = 0.5;
																								 end_right_ray.z = 0.5;

																								 //Assuming symmetrical objects, 
																								 //solve for the coordinates of the point on the back.
																								 float m1 = (0.5-STEREO_ORIGIN.z())/(top_left_corner.z-STEREO_ORIGIN.z());
																								 end_left_ray.x = STEREO_ORIGIN.x() + m1*(top_left_corner.x-STEREO_ORIGIN.x());
																								 if (end_left_ray.x > BBX_MAX.x()) {
																									 //The ray intersects the BBX first
																									 m1 = (BBX_MAX.x()-STEREO_ORIGIN.x())/(top_left_corner.x-STEREO_ORIGIN.x());
																									 end_left_ray.x = BBX_MAX.x();
																									 end_left_ray.z = STEREO_ORIGIN.z() + m1*(top_left_corner.z-STEREO_ORIGIN.z());
																									 } 
																									 end_left_ray.y = STEREO_ORIGIN.y() + m1*(top_left_corner.y-STEREO_ORIGIN.y());

																									 float m2 = (0.5-STEREO_ORIGIN.z())/(top_right_corner.z-STEREO_ORIGIN.z());
																									 end_right_ray.x = STEREO_ORIGIN.x() + m2*(top_right_corner.x-STEREO_ORIGIN.x());
																									 if (end_right_ray.x > BBX_MAX.x()) {
																										 //The ray intersects the BBX first
																										 m2 = (BBX_MAX.x()-STEREO_ORIGIN.x())/(top_right_corner.x-STEREO_ORIGIN.x());
																										 end_right_ray.x = BBX_MAX.x();
																										 end_right_ray.z = STEREO_ORIGIN.z() + m2*(top_right_corner.z-STEREO_ORIGIN.z());
																										 } 
																										 end_right_ray.y = STEREO_ORIGIN.y() + m2*(top_right_corner.y-STEREO_ORIGIN.y());

																										 corners.push_back(top_left_corner);
																										 corners.push_back(top_right_corner);
																										 corners.push_back(end_left_ray);
																										 corners.push_back(end_right_ray);

																										 return corners;
																										 }
																										 }
																										 */		