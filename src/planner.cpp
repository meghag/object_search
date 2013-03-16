#include "planner.h"

//void pubCloud(const std::string &topic_name, const pcl::PointCloud<PointT>::Ptr &cloud, std::string frame_id = fixed_frame_);

bool inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax)
{
    if ((point.x() > bbmin.x()) && (point.x() < bbmax.x()) &&
		(point.y() > bbmin.y()) && (point.y() < bbmax.y()) &&
		(point.z() > bbmin.z()) && (point.z() < bbmax.z()))
        return true;
    else
        return false;
}

geometry_msgs::Pose tfPoseToGeometryPose(tf::Pose tp)
{
	geometry_msgs::Pose gp;
	gp.position.x = tp.getOrigin().getX();
	gp.position.y = tp.getOrigin().getY();
	gp.position.z = tp.getOrigin().getZ();

	gp.orientation.x = tp.getRotation().getX();
	gp.orientation.y = tp.getRotation().getY();
	gp.orientation.z = tp.getRotation().getZ();
	gp.orientation.w = tp.getRotation().getW();

	return gp;
}

//van der corput sequence
double vdc(int n, double base = 2)
{
    double vdc = 0, denom = 1;
    while (n)
    {
        vdc += fmod(n, base) / (denom *= base);
        n /= base; // note: conversion from 'double' to 'int'
    }
    return vdc;
}

//get the nth element of a dense sequence of poses filling 0..1 in x, y, z and the SO(3) for rotation
// based on van der corput sequence with relatively prime bases
tf::Pose vdc_pose(int n)
{
    tf::Pose ret;
    ret.setOrigin(tf::Vector3(vdc(n,bases[0]),vdc(n,bases[1]),vdc(n,bases[2])));
    double u[3];
    for (int i= 0; i<3; i++)
        u[i] = vdc(n,bases[i+3]);
    double q[4];
    q[0] = sqrt(1 - u[0]) * sin(2 * M_PI * u[1]);
    q[1] = sqrt(1 - u[0]) * cos(2 * M_PI * u[1]);
    q[2] = sqrt(u[0]) * sin(2 * M_PI * u[2]);
    q[3] = sqrt(u[0]) * cos(2 * M_PI * u[2]);
    //ret.setRotation(tf::Quaternion(q[0],q[1],q[2],q[3]));

    //Temp hack for only upright poses.
    srand((unsigned)time(NULL));
    double theta = ((double)rand()/((double)RAND_MAX + 1))*44.0/7;
    tf::Matrix3x3 rot_RPY;
    tf::Quaternion quat;
    rot_RPY.setRPY(0.0, 0.0, theta);
    rot_RPY.getRotation(quat);
    ret.setRotation(quat);
    return ret;
}

tf::Pose vdc_pose_bound(tf::Vector3 min, tf::Vector3 max, int n)
{
    tf::Pose ret = vdc_pose(n);
    ret.getOrigin() = tf::Vector3( min.x() + (ret.getOrigin().x() * (max.x() - min.x())),
								  min.y() + (ret.getOrigin().y() * (max.y() - min.y())),
								  min.z() + (ret.getOrigin().z() * (max.z() - min.z())));
    return ret;
}

void breakpoint()
{
	int user_input;
	std::cout << "Save the screen shot. Press 1 and then 'Enter' after you are done." << std::endl;
	std::cin >> user_input;
}

Planner::Planner (ros::NodeHandle& n, int horizon): n_(n), MAX_HORIZON(horizon)
{
	//planActionsServer_ = n_.advertiseService("plan_actions", &DataProcessor::planActionsCallback, this);
	ROS_INFO("Ready to plan actions.");

	//planRequestSub_ = n_.subscribe("plan_request", 1, &Planner::planRequestCallback, this);
	planRequestServer_ = n_.advertiseService("plan", &Planner::planRequestCallback, this);
	objectCloudPub_ = n_.advertise<sensor_msgs::PointCloud2>("object_cloud",1);
	gridPub_ = n_.advertise<visualization_msgs::Marker>("grid",2);
	clustersPub_ = n_.advertise<sensor_msgs::PointCloud2>("one_cluster",1);
	movablePub_ = n_.advertise<sensor_msgs::PointCloud2>("movable_clusters",1);
	newPosePub_ = n_.advertise<sensor_msgs::PointCloud2>("new_pose",1);
	manipulateClient_ = n_.serviceClient<tum_os::Execute_Plan>("execute_plan");
	newpcdClient_ = n.serviceClient<tum_os::Get_New_PCD>("get_new_pcd");
	source_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("planner_source_pose", 1);
	dest_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("planner_dest_pose", 1);
	bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
	octreePub_ = n_.advertise<visualization_msgs::MarkerArray>("octree",1);
	visibleObjectsPub_ = n_.advertise<visualization_msgs::MarkerArray>("visible_objects",1);

	new_data_wanted_ = false;
	numKnownObjects_ = 0;
	planningIteration = 0;
	totalPlanningTime = 0.0;

	octree_ = new octomap::OcTree(0.01);

	//generate object we search as a pointcloud
	pcl::PointCloud<PointT>::Ptr object_cloud (new pcl::PointCloud<PointT>);
	//TODO: Make the dimensions of target object parameters
	target_x = 0.03;	 target_y = 0.03;	 target_z = 0.03;
	ROS_INFO("Filling the point cloud for my object");

	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	object_cloud->width  = 100;
	object_cloud->height = 1;
	object_cloud->points.resize (object_cloud->width * object_cloud->height);

	// Generate the data
	for (size_t i = 0; i < object_cloud->points.size (); ++i)
	{
		object_cloud->points[i].x = target_x * rand () / (RAND_MAX + 1.0f);
		object_cloud->points[i].y = target_y * rand () / (RAND_MAX + 1.0f);
		object_cloud->points[i].z = target_z * rand () / (RAND_MAX + 1.0f);
	}

	/*
	int j = 0;
	for (int i=0; i < 500; i++)
	{
		tf::Pose act = vdc_pose(j++);
		act.setOrigin(tf::Vector3(0,0,0));
		tf::Pose rel;
		rel.setOrigin(tf::Vector3(.0125,0,0));
		//rel.setOrigin(tf::Vector3(.05,0,0));
		rel.setRotation(tf::Quaternion(0,0,0,1));

		act = act * rel;
		PointT pt;
		pt.x = act.getOrigin().x();
		pt.y = act.getOrigin().y();
		pt.z = act.getOrigin().z() * 4;

		object_cloud->points.push_back(pt);
	}
	*/
	toROSMsg(*object_cloud, targetCloud2_);
	targetDimensions_ = tf::Vector3(0.03, 0.03, 0.03);
	ROS_INFO("MG: Publishing my object cloud on topic my_object");
	pubCloud("my_object", object_cloud, fixed_frame_);

	//breakpoint();

	cameraOrigin_ = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str(),ros::Time(0));
	octomap::pose6d octomapCameraPose = octomap::poseTfToOctomap(cameraOrigin_);
	octomapCameraOrigin_ = octomapCameraPose.trans();
	ros::Duration(3).sleep();
	ROS_INFO("ready to plan. Waiting for a plan request.");

	/*octreePub_ = n.advertise<visualization_msgs::MarkerArray>("current_octree_array",100);
	raycastPub_ = n.advertise<visualization_msgs::MarkerArray>("raycast_array",200);
	visibleObjectsSub_ = n_.subscribe("visible_objects", 1, &Planner::visibleObjectsCallback, this);
	octreeSub_ = n_.subscribe("octree",1, &Planner::octreeCallback, this);
	tree_ = tree(treeResolution_);
	*/
}

Planner::~Planner (void) {}

void Planner::pubCloud(const std::string &topic_name, const pcl::PointCloud<PointT>::Ptr &cloud, std::string frame_id)
{
    ros::Publisher *cloud_pub;

    if (cloud_publishers.find(topic_name) == cloud_publishers.end())
    {

        cloud_pub = new ros::Publisher();
        *cloud_pub = n_.advertise<sensor_msgs::PointCloud2>(topic_name,0,true);

        cloud_publishers.insert(std::pair<std::string, ros::Publisher*>(topic_name, cloud_pub ));
        //std::cout << "created new publisher" << cloud_pub << std::endl;
    }
    else
    {
        cloud_pub = cloud_publishers.find(topic_name)->second;
        //std::cout << "found pub on " << cloud_pub->getTopic() << ", reusing it" << std::endl;
    }

    sensor_msgs::PointCloud2 out; //in map frame

    pcl::toROSMsg(*cloud,out);
    out.header.frame_id = frame_id;
    out.header.stamp = ros::Time::now();
    cloud_pub->publish(out);

    //ROS_DEBUG("published frame %s %i x %i points on %s", out.header.frame_id.c_str(), out.height, out.width, topic_name.c_str());
}

void Planner::pub_belief(const std::string &topic_name,const std::vector<tf::Pose> poses)
{
	//ROS_DEBUG("MG: Inside pub_belief");
    ros::Publisher *pose_ary_pub;

    if (belief_publishers.find(topic_name) == belief_publishers.end())
    {

        pose_ary_pub = new ros::Publisher();

        *pose_ary_pub = n_.advertise<geometry_msgs::PoseArray>(topic_name,0,true);

        belief_publishers.insert(std::pair<std::string, ros::Publisher*>(topic_name, pose_ary_pub ));
        //std::cout << "created new publisher" << cloud_pub << std::endl;
    }
    else
    {
        pose_ary_pub = belief_publishers.find(topic_name)->second;
        //std::cout << "found pub on " << cloud_pub->getTopic() << ", reusing it" << std::endl;
    }

    geometry_msgs::PoseArray ps_ary;
    ps_ary.header.frame_id = fixed_frame_;

    for (std::vector<tf::Pose>::const_iterator it = poses.begin(); it!=poses.end(); it++)
    {
        geometry_msgs::Pose pose_msg;
        tf::poseTFToMsg(*it, pose_msg);
        ps_ary.poses.push_back(pose_msg);
    }

    pose_ary_pub->publish(ps_ary);
    //ROS_DEBUG("MG: Exiting pub_belief");
}

tf::Stamped<tf::Pose> Planner::getPose(const std::string target_frame,const std::string lookup_frame, ros::Time tm = ros::Time(0))
{
	//ROS_DEBUG("MG: Inside get Pose: Returns a transform between two frames as a TF Stamped Pose");
    //init();
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    bool had_to_retry = false;

    //listener_->waitForTransform(target_frame, lookup_frame, tm, ros::Duration(0.5));
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            //listener_->lookupTransform(target_frame, lookup_frame,tm, transform);
            listener.lookupTransform(target_frame, lookup_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            std::string what = ex.what();
            what.resize(100);
            ROS_INFO("getPose: tf::TransformException ex.what()='%s', will retry",what.c_str());
            transformOk = false;
            had_to_retry = true;
            //listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
        }
        if (!transformOk)
            rate.sleep();
    }

    if (had_to_retry)
        ROS_INFO("Retry sucessful");

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    //ROS_DEBUG("MG: Exiting get Pose");
    return ret;
}

TableTopObject Planner::createTTO(sensor_msgs::PointCloud2& cloud2)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(cloud2, *cloud);
	//ROS_DEBUG("Creating a TableTopObject (TTO) for other_cloud");
	TableTopObject cloudTTO(cameraOrigin_.getOrigin(), (float)BB_MIN.z(), *cloud);
	//ROS_DEBUG("Publishing the other cloud TTO on topic other_cloud_TTO");
	//pubCloud("other_cloud_TTO", otherCloudTTO.getAsCloud() , fixed_frame_);
	return cloudTTO;
}

void Planner::display_bbx()
{
	//Display BBX in RViz
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "bbx";
	marker.id = 1;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	//marker.pose = pose;
	marker.scale.x = 0.005;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(0.0);
	vector<geometry_msgs::Point> points;
	points.resize(5);
	geometry_msgs::Point pt;
	pt.z = tableHeight_;

	pt.x = BB_MIN.getX();
	pt.y = BB_MIN.getY();
	points[0] = pt;
	pt.y = BB_MAX.getY();
	points[1] = pt;
	pt.x = BB_MAX.getX();
	points[2] = pt;
	pt.y = BB_MIN.getY();
	points[3] = pt;
	pt.x = BB_MIN.getX();
	points[4] = pt;
	marker.points = points;
	for (int c = 0; c < 100; c++)
		gridPub_.publish(marker);
}

void Planner::display_octree(octomap::OcTree* octree)
{
	std::list<octomap::OcTreeVolume> freeVoxels;
	std::list<octomap::OcTreeVolume> occupiedVoxels;
	octree->getFreespace(freeVoxels);
	octree->getOccupied(occupiedVoxels);
	ROS_INFO("Num free voxels  = %zu, num occupied voxels = %zu", freeVoxels.size(), occupiedVoxels.size());
	//Display BBX in RViz
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.resize(freeVoxels.size()+occupiedVoxels.size());
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "octree";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	//marker.pose = pose;
	marker.scale.x = 0.01;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 0.2;
	marker.lifetime = ros::Duration(0.0);
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.x = 1.0;
	marker.lifetime = ros::Duration(100.0);
	int i = 0;
	//octomap::point3d min = octree_->getBBXMin, max;

	/*
	int n_x_voxels = (octree->getBBXMax().x() - octree->getBBXMin().x())/octree->getResolution();
	int n_y_voxels = (octree->getBBXMax().y() - octree->getBBXMin().y())/octree->getResolution();
	int n_z_voxels = (octree->getBBXMax().z() - octree->getBBXMin().z())/octree->getResolution();


	for (size_t p = 0; (int)p < n_x_voxels; p++)
		for (size_t j = 0; (int)j < n_y_voxels; j++)
			for (size_t k = 0; (int)k < n_z_voxels; k++) {
				//Find voxel center
				octomap::point3d center(octree->getBBXMin().x()+(p+1)*octree->getResolution(),
						octree->getBBXMin().y()+(j+1)*octree->getResolution(),
						octree->getBBXMin().z()+(k+1)*octree->getResolution());
				octomap::OcTreeNode *node = octree->search(center);
				if (!node) {
					//Hidden voxel
					marker.pose.position.x = center.x();
					marker.pose.position.y = center.y();
					marker.pose.position.z = center.z();
					marker.scale.x = octree->getResolution();
					marker.scale.y = octree->getResolution();
					marker.scale.z = octree->getResolution();
					marker.color.r = 0.0;
					marker.color.g = 1.0;
					marker.color.b = 1.0;
					marker.color.a = 1.0;
					marker_array.markers[i] = marker;
					marker.id = i;
					i++;
				}
			}
*/

/*
	//Free voxels in green
	for (std::list<octomap::OcTreeVolume>::iterator it = freeVoxels.begin(); it != freeVoxels.end(); ++it) {
		//ROS_INFO("free voxel center = %f %f %f", (it->first).x(), (it->first).y(), (it->first).z());
		if (octree->inBBX(it->first)) {
			marker.pose.position.x = it->first.x();
			marker.pose.position.y = it->first.y();
			marker.pose.position.z = it->first.z();
			marker.scale.x = it->second;
			marker.scale.y = it->second;
			marker.scale.z = it->second;
			//marker_array.markers[i] = marker;
			marker.id = i;
			if (i < 10) {
				octomap::OcTreeKey key;
				octree->genKey(it->first, key);
				octomap::OcTreeNode *node = octree->search(key);
				ROS_INFO("Occupancy: %f, value = %f", node->getOccupancy(), node->getValue());
			}
			i++;
		}
	}

	//ROS_INFO("octree bbx min = %f %f %f", octree_->getBBXMin().x(), (octree_->getBBXMin()).y(), (octree_->getBBXMin()).z());
	//ROS_INFO("octree bbx max = %f %f %f", (octree_->getBBXMax()).x(), (octree_->getBBXMax()).y(), (octree_->getBBXMax()).z());
*/

	//Occupied voxels in red
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.a = 0.5;
	i = 0;
	for (std::list<octomap::OcTreeVolume>::iterator it = occupiedVoxels.begin(); it != occupiedVoxels.end(); ++it) {
		if (octree_->inBBX(it->first)) {
			marker.pose.position.x = it->first.x();
			marker.pose.position.y = it->first.y();
			marker.pose.position.z = it->first.z();
			marker.scale.x = it->second;
			marker.scale.y = it->second;
			marker.scale.z = it->second;
			marker_array.markers[i] = marker;
			marker.id = i;
			/*
			if (i < 10) {
				octomap::OcTreeKey key;
				octree->genKey(it->first, key);
				octomap::OcTreeNode *node = octree->search(key);
				ROS_INFO("Occupancy: %f, value = %f", node->getOccupancy(), node->getValue());
			}
			*/
			i++;
		}
	}

/*
	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it!= end; ++it)
	{
		octomap::OcTreeNode *node = octree->search(it.getKey());

		if (octree->isNodeOccupied(node)) {
			marker.pose.position.x = it.getX();
			marker.pose.position.y = it.getY();
			marker.pose.position.z = it.getZ();
			marker.scale.x = it.getSize();
			marker.scale.y = it.getSize();
			marker.scale.z = it.getSize();
			marker_array.markers[i] = marker;
			marker.id = i;
			i++;
		}
	}
	  //manipulate node, e.g.:
	  std::cout << "Node center: " << it.getCoordinate() << std::endl;
	  std::cout << "Node size: " << it.getSize() << std::endl;
	  std::cout << "Node value: " << it->getValue() << std::endl;
	}
	*/

	octreePub_.publish(marker_array);
}

void Planner::projectToPlanePerspective(const tf::Vector3 sensorOrigin,
		const double tableHeight,
		pcl::PointCloud<PointT>::Ptr& cloud,
		pcl::PointCloud<PointT>::Ptr& cloud_projected)
{
	//pcl::PointCloud<PointT> cloud;
	//fromROSMsg(cloud2, cloud);
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

void Planner::updateHiddenVoxels(const tf::Vector3 sensorOrigin,
		octomap::OcTree* octree,
		sensor_msgs::PointCloud2 cloud2,
		const double tableHeight,
		bool add_as_occupied)
{
    pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    fromROSMsg(cloud2, *cloud);

    projectToPlanePerspective(sensorOrigin, tableHeight, cloud, cloud_projected);

    octomap::KeySet occupied_cells;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        octomap::KeyRay ray;
        octomath::Vector3 start;
        start.x() = cloud->points[i].x;
        start.y() = cloud->points[i].y;
        start.z() = cloud->points[i].z;
        octomath::Vector3 end;
        end.x() = cloud_projected->points[i].x;
        end.y() = cloud_projected->points[i].y;
        end.z() = cloud_projected->points[i].z;
        octree->computeRayKeys (start, end, ray);
        for (octomap::KeyRay::iterator it = ray.begin(); it != ray.end(); it++)
        {
            occupied_cells.insert(*it);
        }
    }

    for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
    	if (add_as_occupied) {
    		//Add hidden voxels as occupied
    		octree->updateNode(*it, true, false);
    	} else {
    		//Delete this node
    		//octree->deleteNode(*it);
    		octree->updateNode(*it, false, false);
    	}
    }
}

void Planner::updateFreeVoxels(octomap::OcTree* octree)
{
	//std::list<octomap::OcTreeVolume> freeVoxels;
	//octree->getFreespace(freeVoxels);

	unsigned int n_x_voxels = (octree->getBBXMax().x() - octree->getBBXMin().x())/octree->getResolution();
	unsigned int n_y_voxels = (octree->getBBXMax().y() - octree->getBBXMin().y())/octree->getResolution();
	unsigned int n_z_voxels = (octree->getBBXMax().z() - octree->getBBXMin().z())/octree->getResolution();

	for (size_t i = 0; i < n_x_voxels; i++) {
		for (size_t j = 0; j < n_y_voxels; j++)
			for (size_t k = 0; k < n_z_voxels; k++) {
				//Find voxel center
				octomap::point3d center(octree->getBBXMin().x()+(i+1)*octree->getResolution(),
						octree->getBBXMin().y()+(j+1)*octree->getResolution(),
						octree->getBBXMin().z()+(k+1)*octree->getResolution());
				octomap::OcTreeKey key;

				// this can happen when we have an empty octree, which would of course not cover the object, so return false
				if (!octree->genKey(center, key)) {
					ROS_ERROR("Octree is empty!");
					return;
				}

				octomap::OcTreeNode *node = octree->search(key);
				if (!node || (node && !octree->isNodeOccupied(node))) {
					//This coordinate is in free space
					octree->updateNode(key, false);
					//ROS_INFO("Occupancy of now free node = %f", (octree->updateNode(key, false))->getOccupancy());
					//freeVoxels.push_back(std::make_pair(center, octree->getResolution()));
				}
			}
	}
}

void Planner::generateOctree(vector<sensor_msgs::PointCloud2> known_objects, vector<bool> moved,
		octomap::OcTree* octree)
{
	vector<sensor_msgs::PointCloud2> selectiveVec, vec_known_object_cloud2 = known_objects;
	for (size_t i = 0; i < known_objects.size(); i++) {
		if (beenMoved_[i] == false)
			selectiveVec.push_back(known_objects[i]);
	}
	sensor_msgs::PointCloud2 selectiveCloud2 = concatClouds(selectiveVec);

	//sensor_msgs::PointCloud2 newObjectCloud2 = concatClouds(current_config);
	sensor_msgs::PointCloud2 known_object_cloud2 = concatClouds(vec_known_object_cloud2);
	//pcl::PointCloud<PointT>::Ptr current_config_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr known_object_cloud(new pcl::PointCloud<PointT>);
	//fromROSMsg(newObjectCloud2, *current_config_cloud);
	fromROSMsg(known_object_cloud2, *known_object_cloud);
	//pubCloud("current_simulated_config", current_config_cloud, fixed_frame_);
	//pubCloud("current_simulated_config", known_object_cloud, fixed_frame_);
	//ROS_INFO("Published known objects on current_simulated_config topic");

	octomap::Pointcloud octomapCloud;
	octomap::pointcloudPCLToOctomap (*known_object_cloud, octomapCloud);
	octomap::point3d octo_bbmin(BB_MIN.x(), BB_MIN.y(), BB_MIN.z()-0.01), octo_bbmax(BB_MAX.x(), BB_MAX.y(), BB_MAX.z());
	//octomap::OcTree* octree = new octomap::OcTree(0.01);
	octree->setBBXMin(octo_bbmin);
	octree->setBBXMax(octo_bbmax);
	octree->bbxSet();
	octree->useBBXLimit(true);
	//octree->enableChangeDetection(true);

	octree->insertScan(octomapCloud, octomapCameraOrigin_);
	/*
	ROS_INFO("Displaying new octree before updating it");
	breakpoint();
	display_octree(octree);
	breakpoint();
	*/
	updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree, selectiveCloud2, tableHeight_, true);
	updateFreeVoxels(octree);
}

bool Planner::planRequestCallback(tum_os::PlanService::Request &plan_request,
                   tum_os::PlanService::Response &plan_response)
{
	//clusters_msg  = *(ros::topic::waitForMessage<tum_os::Clusters>("/clusters"));

	objectCloud2_ = plan_request.object_cloud;
	planarCloud2_ = plan_request.planar_cloud;
	if (planningIteration == 0) {
		tableHeight_ = plan_request.table_height;
		BB_MIN = tf::Vector3(plan_request.bb_min[0], plan_request.bb_min[1], plan_request.bb_min[2]);
		BB_MAX = tf::Vector3(plan_request.bb_max[0], plan_request.bb_max[1], plan_request.bb_max[2]);
		ROS_INFO("table height = %f", tableHeight_);
	}
	objectCloudPub_.publish(objectCloud2_);
	display_bbx();

	//breakpoint();

	cluster(objectCloud2_, 0.02, 100, 10000, clustersDetected_);
	//cluster(objectCloud2_, 0.02, 300, 1000, clustersDetected_);
	cluster_to_known_idx_map_.clear();

	//Filling bounding boxes of detected clusters
	pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
	visualization_msgs::MarkerArray m_array;
	for (unsigned int c = 0; c < clustersDetected_.size(); c++) {
		clustersDetected_[c].header.frame_id = "base_link";
		fromROSMsg(clustersDetected_[c], *clusterCloud);
		tf::Vector3 min, max;
		minmax3d(min, max, clusterCloud);

		//Extend BBX to table
		min.setZ(tableHeight_);

		//Get bounding box of target cloud
		ROS_INFO("Finding bounding box of cloud");
		object_manipulation_msgs::ClusterBoundingBox bbx;
		//target_cloud2.header.frame_id = "base_link";
		//getClusterBoundingBox(target_cloud2, bbx.pose_stamped, bbx.dimensions);
		bbx.pose_stamped.pose.position = find_centroid(*clusterCloud);
		bbx.dimensions.x = max.getX() - min.getX();
		if (bbx.dimensions.x <= 0.03)
			bbx.dimensions.x += 0.04;			//Adding some depth to objects because the real depth may not be seen.
		bbx.dimensions.y = max.getY() - min.getY();
		bbx.dimensions.z = max.getZ() - min.getZ();
		ROS_INFO("bbx position: %f %f %f, dimensions: %f %f %f", bbx.pose_stamped.pose.position.x, bbx.pose_stamped.pose.position.y, bbx.pose_stamped.pose.position.z,
				bbx.dimensions.x, bbx.dimensions.y, bbx.dimensions.z);

		visualization_msgs::Marker bbx_marker = set_marker("base_link", "bbx", c, visualization_msgs::Marker::CUBE,
					bbx.pose_stamped.pose, bbx.dimensions, 0.0f, 0.0f, 1.0f, 1.0);
		m_array.markers.push_back(bbx_marker);

		//fill the bbx with points and generate a point cloud
		pcl::PointCloud<PointT>::Ptr object_cloud (new pcl::PointCloud<PointT>);
		ROS_DEBUG("Filling the point cloud for object to be sampled");

		// Fill in the cloud data
		object_cloud->width  = clusterCloud->points.size()/1.0;			//Just a heuristic for reducing cloud size
		object_cloud->height = 1;
		object_cloud->points.resize (object_cloud->width * object_cloud->height);
		for (size_t i = 0; i < object_cloud->points.size (); ++i)
		{
			object_cloud->points[i].x = min.getX() + bbx.dimensions.x * rand () / (RAND_MAX + 1.0f);
			object_cloud->points[i].y = min.getY() + bbx.dimensions.y * rand () / (RAND_MAX + 1.0f);
			object_cloud->points[i].z = min.getZ() + bbx.dimensions.z * rand () / (RAND_MAX + 1.0f);
		}

		sensor_msgs::PointCloud2 objectCloud2;
		toROSMsg(*object_cloud, objectCloud2);
		objectCloud2.header.frame_id = "base_link";
		clustersDetected_[c] = objectCloud2;
		pubCloud("filled_cluster_bbx", object_cloud, fixed_frame_);

		vector<geometry_msgs::Point> extents = find_extents(*object_cloud);
		tf::Vector3 centroid(extents[0].x + (extents[1].x - extents[0].x)/2.0f,
				extents[2].y + (extents[3].y - extents[2].y)/2.0f,
				extents[4].z + (extents[5].z - extents[4].z)/2.0f);
		tf::Vector3 dimensions(extents[1].x - extents[0].x, extents[3].y - extents[2].y, extents[5].z - extents[4].z);

		ROS_INFO("Before: known objects map size = %zu, num known objects = %u, cluster to known idx map size = %zu, beenMoved_ map size = %zu",
					knownObjects_.size(), numKnownObjects_, cluster_to_known_idx_map_.size(), beenMoved_.size());

		bool new_object = true;
		for (unsigned int j = 0; j < knownObjects_.size(); j++) {
			ROS_DEBUG("distance between centroids = %f", abs((knownObjects_[j].first - centroid).length()));
			if (abs((knownObjects_[j].first - centroid).length()) < 0.05) {
				new_object = false;
				//cluster_to_known_idx_map_.insert(std::make_pair(c,j));
				cluster_to_known_idx_map_[c] = j;
				knownObjects_[j] = std::make_pair(centroid, clustersDetected_[c]);
				knownObjectDimensions_[j] = dimensions;
				break;
			}
		}

		if (new_object) {
			knownObjects_.insert(std::make_pair(numKnownObjects_, std::make_pair(centroid, clustersDetected_[c])));
			knownObjectDimensions_.push_back(dimensions);
			beenMoved_.insert(std::make_pair(numKnownObjects_, false));
			cluster_to_known_idx_map_.insert(std::make_pair(c,numKnownObjects_));
			numKnownObjects_++;
		}
		//breakpoint();
	}

	visibleObjectsPub_.publish(m_array);

	ROS_INFO("After: known objects map size = %zu, num known objects = %u, cluster to known idx map size = %zu, beenMoved_ map size = %zu",
			knownObjects_.size(), numKnownObjects_, cluster_to_known_idx_map_.size(), beenMoved_.size());

	//breakpoint();
	ros::Duration(2.0).sleep();

	//vector<sensor_msgs::PointCloud2> vecShelfCloud;
	//vecShelfCloud.push_back(objectCloud2_);
	//vecShelfCloud.push_back(planarCloud2_);

	vector<sensor_msgs::PointCloud2> selectiveVec, vec_known_object_cloud2;;
	vec_known_object_cloud2.resize(numKnownObjects_);
	for (size_t i = 0; i < numKnownObjects_; i++) {
		vec_known_object_cloud2[i] = knownObjects_[i].second;
		if (beenMoved_[i] == false)
			selectiveVec.push_back(knownObjects_[i].second);
	}

	sensor_msgs::PointCloud2 selectiveCloud2 = concatClouds(selectiveVec);
	sensor_msgs::PointCloud2 known_object_cloud2 = concatClouds(vec_known_object_cloud2);
	pcl::PointCloud<PointT>::Ptr known_object_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(known_object_cloud2, *known_object_cloud);
	pubCloud("current_simulated_config", known_object_cloud, fixed_frame_);
	ROS_INFO("Published known objects on current_simulated_config topic");

	octomap::Pointcloud octomapCloud;
	//octomap::pointcloudPCLToOctomap (*current_config_cloud, octomapCloud);
	octomap::pointcloudPCLToOctomap (*known_object_cloud, octomapCloud);

	//pcl::PointCloud<PointT>::Ptr shelf_cloud(new pcl::PointCloud<PointT>);
	//sensor_msgs::PointCloud2 shelfCloud2 = concatClouds(vecShelfCloud);
	//sensor_msgs::PointCloud2 shelfCloud2 = concatClouds(clustersDetected_);
	//fromROSMsg(shelfCloud2, *shelf_cloud);

	octomap::point3d octo_bbmin(BB_MIN.x(), BB_MIN.y(), BB_MIN.z()-0.01), octo_bbmax(BB_MAX.x(), BB_MAX.y(), BB_MAX.z());
	ROS_INFO("Size of octree = %zu", octree_->size());

	if (octree_->size() == 1) {
		octree_->setBBXMin(octo_bbmin);
		octree_->setBBXMax(octo_bbmax);
		octree_->bbxSet();
		octree_->useBBXLimit(true);
		//octree_->enableChangeDetection(true);
		ROS_INFO("after setting octo bbx: octo_bbmin = %f %f %f", octree_->getBBXMin().x(), octree_->getBBXMin().y(), octree_->getBBXMin().z());
		ROS_INFO("after setting octo bbx: octo_bbmax = %f %f %f", octree_->getBBXMax().x(), octree_->getBBXMax().y(), octree_->getBBXMax().z());
		octree_->insertScan(octomapCloud, octomapCameraOrigin_);
		//updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree_, objectCloud2_, tableHeight_, true);
		updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree_, selectiveCloud2, tableHeight_, true);
		updateFreeVoxels(octree_);
		/*
			//int n_x_voxels = (octree_->getBBXMax().x() - octree_->getBBXMin().x())/octree_->getResolution();
			int n_y_voxels = (octree_->getBBXMax().y() - octree_->getBBXMin().y())/octree_->getResolution();
			int n_z_voxels = (octree_->getBBXMax().z() - octree_->getBBXMin().z())/octree_->getResolution();

			//for (size_t i = 0; i < n_x_voxels; i++) {
				for (size_t j = 0; j < n_y_voxels; j++)
					for (size_t k = 0; k < n_z_voxels; k++) {
						//Find voxel center
						octomap::point3d ray_end(octree_->getBBXMax().x(),
								octree_->getBBXMin().y()+(j+1)*octree_->getResolution(),
								octree_->getBBXMin().z()+(k+1)*octree_->getResolution());
						octomap::point3d direction(ray_end - sensor_origin);
						octomap::point3d end;
						octree_->castRay(sensor_origin, direction, end, true, 1.5);
						octree_->insertRay(sensor_origin, end);
					}
			//}
		 */
		ROS_INFO("Displaying octree");
		display_octree(octree_);
		//breakpoint();
	} else {
		octomap::OcTree* octree = new octomap::OcTree(0.01);
		octree->setBBXMin(octo_bbmin);
		octree->setBBXMax(octo_bbmax);
		octree->bbxSet();
		octree->useBBXLimit(true);
		//octree->enableChangeDetection(true);

		octree->insertScan(octomapCloud, octomapCameraOrigin_);
		ROS_INFO("Displaying new octree before updating it");
		//breakpoint();
		display_octree(octree);
		breakpoint();

		std::list<octomap::OcTreeVolume> occupiedVoxels;
		octree->getOccupied(occupiedVoxels);
		unsigned int n_occupied_before = occupiedVoxels.size();
		updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree, selectiveCloud2, tableHeight_, true);
		updateFreeVoxels(octree);

		ROS_INFO("Displaying new octree after update");
		//breakpoint();
		display_octree(octree);
		//breakpoint();
		std::list<octomap::OcTreeVolume> occupiedVoxels2;
		octree->getOccupied(occupiedVoxels2);
		unsigned int n_occupied_after = occupiedVoxels2.size();
		octree_ = octree;

		if (n_occupied_after - n_occupied_before < 5) {
			ROS_INFO("All space explored.");
			ROS_INFO("It took %f seconds to explore.", totalPlanningTime);
			breakpoint();
		}

		/*
		octomap::OcTree* octree = new octomap::OcTree(0.01);
		octree->insertScan(octomapCloud, octomapCameraOrigin_);
		updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree, shelfCloud2, tableHeight_, true);
		updateFreeVoxels(octree);
		for (std::list<octomap::OcTreeVolume>::iterator it = freeVoxels_.begin(); it != freeVoxels_.end(); ++it) {
			octomap::OcTreeKey key;
			octree->genKey(it->first, key);
			octree->updateNode(key, false);
		}

		octree_ = octree;
		ROS_INFO("Displaying octree");
		display_octree(octree_);
		breakpoint();
		*/
	}
	//octree_->updateInnerOccupancy() ??

	call_plan(objectCloud2_);

	plan_response.result = true;
	new_data_wanted_ = true;
	return true;
}

void Planner::call_plan(sensor_msgs::PointCloud2 objectCloud2)
{
	//objectCloudPub_.publish(objectCloud2);
	pcl::PointCloud<PointT>::Ptr objectCloud(new pcl::PointCloud<PointT>);
	fromROSMsg(objectCloud2, *objectCloud);
	pubCloud("object_cloud_permanent", objectCloud, fixed_frame_);

	ROS_INFO("Displaying the object cloud for planning");
	//breakpoint();

	ROS_INFO("Found %zu clusters", clustersDetected_.size());
	if (clustersDetected_.size() == 0) {
		ROS_ERROR("EXITING.");
		//ros::shutdown();
	}

	/*
	//Detecting target object based on color
	for (size_t i = 0; i < clustersDetected_.size(); i++)
	{
		clustersDetected_[i].header.frame_id = "base_link";
		if (find_color(clustersDetected_[i]) == 'g')
		{
			ROS_INFO("Target object spotted! SUCCESS!");
			clustersPub_.publish(clustersDetected_[i]);
			//breakpoint();
			//ros::shutdown();
		}
		//breakpoint();
	}
	 */

	vector<Move> best_next_action_sequence;
	double total_percentage_revealed_so_far = 0.0;
	vector<bool> beenMoved(false, beenMoved_.size());
	vector<sensor_msgs::PointCloud2> config;
	config.resize(knownObjects_.size());
	for (unsigned int i = 0; i < beenMoved_.size(); i++) {
		if (beenMoved_[i] == true)
			beenMoved[i] = true;
		config[i] = knownObjects_[i].second;
	}
	vector<unsigned int> moved_so_far;
	time_t tstart, tend;
	ROS_INFO("Starting to plan.");
	tstart = time(0);
	last_tstart = tstart;
	int orig_horizon = MAX_HORIZON;
	//plan(MAX_HORIZON, clustersDetected_, octree_, objectCloud2_, best_next_action_sequence, total_percentage_revealed_so_far);
	plan(MAX_HORIZON, config, octree_, beenMoved, best_next_action_sequence, total_percentage_revealed_so_far, moved_so_far);
	//random_plan(MAX_HORIZON, config, octree_, objectCloud2_, best_next_action_sequence, total_percentage_revealed_so_far);
	tend = time(0);
	MAX_HORIZON = orig_horizon;
	planningIteration++;

	ROS_INFO("Found the best next action sequence for horizon %d of length %zu", MAX_HORIZON, best_next_action_sequence.size());
	ROS_INFO("It took %f seconds to plan.", difftime(tend, tstart));
	totalPlanningTime += difftime(tend, tstart);
	action_sequence_ = best_next_action_sequence;
	ROS_INFO("%zu moves in the action sequence", best_next_action_sequence.size());
	if (best_next_action_sequence.size() == 0)
		breakpoint();

	geometry_msgs::PoseStamped source_pose, dest_pose;
	source_pose.header.frame_id = "base_link";
	source_pose.header.stamp = ros::Time::now();
	dest_pose.header.frame_id = "base_link";
	dest_pose.header.stamp = ros::Time::now();
	vector<sensor_msgs::PointCloud2> current_config = clustersDetected_;
	for (size_t i = 0; i < best_next_action_sequence.size(); i++)
	{
		ROS_INFO("cluster idx = %d, \n source location = %f %f %f, \n dest location = %f %f %f, push = %d", best_next_action_sequence[i].cluster_idx,
				best_next_action_sequence[i].sourcePose.getOrigin().getX(), best_next_action_sequence[i].sourcePose.getOrigin().getY(),
				best_next_action_sequence[i].sourcePose.getOrigin().getZ(), best_next_action_sequence[i].destPose.getOrigin().getX(),
				best_next_action_sequence[i].destPose.getOrigin().getY(), best_next_action_sequence[i].destPose.getOrigin().getZ(),
				best_next_action_sequence[i].push);

		source_pose.pose = tfPoseToGeometryPose(best_next_action_sequence[i].sourcePose);
		dest_pose.pose = tfPoseToGeometryPose(best_next_action_sequence[i].destPose);
		source_pose_pub_.publish(source_pose);
		dest_pose_pub_.publish(dest_pose);
		ROS_INFO("Move the object.");
		breakpoint();
	}

	new_data_wanted_ = true;

	//Manually execute the plan
	//manual_plan_execution(config);

	//Execute plan
	if (best_next_action_sequence.size() > 0)
		execute_plan();

	//Simulate Plan execution
	//simulate_plan_execution();

/*
	if (new_data_wanted_)
	{
		ros::Duration(2.0).sleep();
		breakpoint();
		ROS_INFO("DUPLO: Asking for new point cloud.");
		tum_os::Get_New_PCD srv_newpcd;
		srv_newpcd.request.new_pcd_wanted = true;
		if (!newpcdClient_.call(srv_newpcd))
		{
			ROS_ERROR("Failed to call service get new pcd.");
			//return -1;
		}
		new_data_wanted_ = false;
	}
*/
	//return true;
}


void Planner::manual_plan_execution(vector<sensor_msgs::PointCloud2> current_config)
{
	//vector<sensor_msgs::PointCloud2> current_config = clustersDetected_;
	vector<sensor_msgs::PointCloud2> new_config;

	geometry_msgs::PoseStamped source_pose, dest_pose;
	source_pose.header.frame_id = "base_link";
	source_pose.header.stamp = ros::Time::now();
	dest_pose.header.frame_id = "base_link";
	dest_pose.header.stamp = ros::Time::now();

	//bool action_success = true;
	//unsigned int moved_idx = action_sequence_[0].cluster_idx;
	for (size_t i = 0; i < action_sequence_.size(); i++)
	{
		ROS_INFO("Executing move %zu in simulation", i);
		source_pose.pose = tfPoseToGeometryPose(action_sequence_[i].sourcePose);
		dest_pose.pose = tfPoseToGeometryPose(action_sequence_[i].destPose);
		source_pose_pub_.publish(source_pose);
		dest_pose_pub_.publish(dest_pose);
		//breakpoint();

		//Executing plan in simulation
		simulateMove(current_config, action_sequence_[i], new_config);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);
		ROS_INFO("Published new config on new_simulated_config topic");

		unsigned int known_idx = cluster_to_known_idx_map_[action_sequence_[i].cluster_idx];
		ROS_INFO("Action %zu: known index = %u", i, known_idx);
		beenMoved_[known_idx] = true;
		pcl::PointCloud<PointT> new_moved_cloud;
		fromROSMsg(new_config[action_sequence_[i].cluster_idx], new_moved_cloud);
		vector<geometry_msgs::Point> extents = find_extents(new_moved_cloud);
		tf::Vector3 new_centroid(extents[0].x + (extents[1].x - extents[0].x)/2.0f,
				extents[2].y + (extents[3].y - extents[2].y)/2.0f,
				extents[4].z + (extents[5].z - extents[4].z)/2.0f);
		//tf::Vector3 dimensions(extents[1].x - extents[0].x, extents[3].y - extents[2].y, extents[5].z - extents[4].z);
		ROS_INFO("Here");
		knownObjects_[known_idx] = std::make_pair(new_centroid, new_config[action_sequence_[i].cluster_idx]);

		current_config = new_config;
		new_config.clear();
		//breakpoint();
	}
}

void Planner::simulate_plan_execution()
{
	vector<sensor_msgs::PointCloud2> current_config = clustersDetected_;
	vector<sensor_msgs::PointCloud2> new_config;

	geometry_msgs::PoseStamped source_pose, dest_pose;
	source_pose.header.frame_id = "base_link";
	source_pose.header.stamp = ros::Time::now();
	dest_pose.header.frame_id = "base_link";
	dest_pose.header.stamp = ros::Time::now();

	//bool action_success = true;
	//unsigned int moved_idx = action_sequence_[0].cluster_idx;
	for (size_t i = 0; i < action_sequence_.size(); i++)
	{
		ROS_INFO("Executing move %zu in simulation", i);
		source_pose.pose = tfPoseToGeometryPose(action_sequence_[i].sourcePose);
		dest_pose.pose = tfPoseToGeometryPose(action_sequence_[i].destPose);
		source_pose_pub_.publish(source_pose);
		dest_pose_pub_.publish(dest_pose);
		//breakpoint();

		//Executing plan in simulation
		simulateMove(current_config, action_sequence_[i], new_config);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);
		ROS_INFO("Published new config on new_simulated_config topic");

		unsigned int known_idx = cluster_to_known_idx_map_[action_sequence_[i].cluster_idx];
		ROS_INFO("Action %zu: known index = %u", i, known_idx);
		beenMoved_[known_idx] = true;
		pcl::PointCloud<PointT> new_moved_cloud;
		fromROSMsg(new_config[action_sequence_[i].cluster_idx], new_moved_cloud);
		vector<geometry_msgs::Point> extents = find_extents(new_moved_cloud);
		tf::Vector3 new_centroid(extents[0].x + (extents[1].x - extents[0].x)/2.0f,
				extents[2].y + (extents[3].y - extents[2].y)/2.0f,
				extents[4].z + (extents[5].z - extents[4].z)/2.0f);
		//tf::Vector3 dimensions(extents[1].x - extents[0].x, extents[3].y - extents[2].y, extents[5].z - extents[4].z);
		ROS_INFO("Here");
		knownObjects_[known_idx] = std::make_pair(new_centroid, new_config[action_sequence_[i].cluster_idx]);

		ROS_INFO("Displaying old octree before updating it");
		breakpoint();
		display_octree(octree_);
		breakpoint();

		current_config = new_config;
		new_config.clear();
		//breakpoint();
	}

	vector<sensor_msgs::PointCloud2> selectiveVec, vec_known_object_cloud2;;
	vec_known_object_cloud2.resize(numKnownObjects_);
	for (size_t i = 0; i < numKnownObjects_; i++) {
		vec_known_object_cloud2[i] = knownObjects_[i].second;
		if (beenMoved_[i] == false)
			selectiveVec.push_back(knownObjects_[i].second);
	}
	sensor_msgs::PointCloud2 selectiveCloud2 = concatClouds(selectiveVec);

	sensor_msgs::PointCloud2 newObjectCloud2 = concatClouds(current_config);
	sensor_msgs::PointCloud2 known_object_cloud2 = concatClouds(vec_known_object_cloud2);
	pcl::PointCloud<PointT>::Ptr current_config_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr known_object_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(newObjectCloud2, *current_config_cloud);
	fromROSMsg(known_object_cloud2, *known_object_cloud);
	//pubCloud("current_simulated_config", current_config_cloud, fixed_frame_);
	pubCloud("current_simulated_config", known_object_cloud, fixed_frame_);
	ROS_INFO("Published known objects on current_simulated_config topic");
	//currentConfigPCL_ = newObjectCloud2;
	//newObjectCloud2.header.frame_id = fixed_frame_;
	//Euclidean segmentation
	//cluster(newObjectCloud2, 0.02, 50, 10000, clustersDetected_);
	cluster(known_object_cloud2, 0.02, 50, 10000, clustersDetected_);

	octomap::Pointcloud octomapCloud;
	//octomap::pointcloudPCLToOctomap (*current_config_cloud, octomapCloud);
	octomap::pointcloudPCLToOctomap (*known_object_cloud, octomapCloud);
	octomap::point3d octo_bbmin(BB_MIN.x(), BB_MIN.y(), BB_MIN.z()-0.01), octo_bbmax(BB_MAX.x(), BB_MAX.y(), BB_MAX.z());

	ROS_INFO("Size of octree = %zu", octree_->size());
	if (octree_->size() == 1) {
		ROS_INFO("Octree size is 1");
		octree_->insertScan(octomapCloud, octomapCameraOrigin_);
		updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree_, newObjectCloud2, tableHeight_, true);
		updateFreeVoxels(octree_);
		ROS_INFO("Displaying octree");
		display_octree(octree_);
		breakpoint();
	} else {
		octomap::OcTree* octree = new octomap::OcTree(0.01);
		octree->setBBXMin(octo_bbmin);
		octree->setBBXMax(octo_bbmax);
		octree->bbxSet();
		octree->useBBXLimit(true);
		//octree->enableChangeDetection(true);

		octree->insertScan(octomapCloud, octomapCameraOrigin_);
		ROS_INFO("Displaying new octree before updating it");
		breakpoint();
		display_octree(octree);
		breakpoint();

		updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree, selectiveCloud2, tableHeight_, true);
		updateFreeVoxels(octree);

		ROS_INFO("Displaying new octree after update");
		breakpoint();
		display_octree(octree);
		breakpoint();

		octree_ = octree;
	}

	//call_plan(newObjectCloud2);
	call_plan(known_object_cloud2);
}


void Planner::samplePose(sensor_msgs::PointCloud2 target_cloud2, 
						 octomap::OcTree* octree,
						 tf::Vector3 sampling_bb_min,
						 tf::Vector3 sampling_bb_max,
						 vector<tf::Pose>& object_posterior_belief,
						 bool check_hidden,
						 bool check_visible,
						 bool push)
{
	//This function samples the whole space for valid poses of the object_cloud.
	//Then it checks whether they lie in the free space and returns an object belief.
	/*
	if (check_hidden && !check_visible)
		ROS_DEBUG("Sampling in occluded space");
	else if (!check_hidden && check_visible)
		ROS_DEBUG("Sampling in free space");
	*/
	
	//TODO: Sample only hidden space, not occupied.

	//ROS_DEBUG("before creating samples");
	int n_belief = 50;
	if (check_hidden)
		n_belief = 50000;
    std::vector<tf::Pose> object_belief;
    //for (int k =0; k < 100000; k++)
    for (int k = 0; k < n_belief; k++)
        object_belief.push_back(vdc_pose_bound(sampling_bb_min, sampling_bb_max,k));

    //ROS_DEBUG("samples created");
    pub_belief("object_belief", object_belief);

    pcl::PointCloud<PointT>::Ptr targetCloud(new pcl::PointCloud<PointT>);
    fromROSMsg(target_cloud2, *targetCloud);

    tf::Vector3 min, max;
    minmax3d(min, max, targetCloud);
    float object_height = max.getZ() - min.getZ();
    //ROS_DEBUG("Cloud z midpoint = %f", min.z() + (max.z() - min.z())/2.0);

    tf::Transform identity;
    identity.setIdentity();
    for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it!=object_belief.end(); it++)
    {
    	//Check if the resulting pose touches the table
    	if (check_visible && (it->getOrigin().getZ() - object_height/2 - tableHeight_ <= 0.02) &&
    			(it->getOrigin().getZ() - object_height/2 - tableHeight_ > 0)) {
    		if (push) {
    			it->setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    			object_posterior_belief.push_back(*it);
    		}
    		//Now check if the pose is hidden or not
    		//TODO: This is wrong!!! Need to pass modified octree with the object being moved missing
    		else if (checkHiddenOrVisible(target_cloud2, octree, *it, identity, check_hidden, check_visible))
    			object_posterior_belief.push_back(*it);
    	} else if (check_hidden && (it->getOrigin().getZ() - object_height/2 - tableHeight_ <= 0.02) &&
    			(it->getOrigin().getZ() - object_height/2 - tableHeight_ > 0))
    		//Reject samples that are not on table but in the air.
    		if (checkHiddenOrVisible(target_cloud2, octree, *it, identity, check_hidden, check_visible))
    			object_posterior_belief.push_back(*it);
    }

    //ROS_DEBUG("samples checked");
    ROS_DEBUG("Size of object belief = %zu", object_posterior_belief.size());
    pub_belief("valid_pose_samples",object_posterior_belief);
    ROS_DEBUG("Published valid poses");
    //breakpoint();
}

bool Planner::checkHiddenOrVisible(sensor_msgs::PointCloud2 object_cloud2,
						  octomap::OcTree* octree,
						  tf::Transform ownTransform, 
						  tf::Transform otherTransform,
						  bool check_hidden,
						  bool check_visible)
{
	//ROS_DEBUG("Inside checkHiddenOrVisible");
	//This function checks if the object_cloud is hidden by the other_cloud.
	//Returns true if hidden, false if in free space.
	tf::Transform resultingTransform = otherTransform.inverseTimes(ownTransform);
	geometry_msgs::Transform trans;
	
	pcl::PointCloud<PointT> cloud, temp;
	fromROSMsg(object_cloud2, cloud);
	temp.points.resize(cloud.points.size());
	
	pcl::PointCloud<PointT>::Ptr targetCloud(new pcl::PointCloud<PointT>);
	fromROSMsg(object_cloud2, *targetCloud);
	tf::Vector3 min, max;
	minmax3d(min, max, targetCloud);
	tf::Vector3 centroid(min.x() + (max.x() - min.x())/2.0, min.y() + (max.y() - min.y())/2.0, min.z() + (max.z() - min.z())/2.0);
	//ROS_DEBUG("Cloud midpoint = %f, %f, %f", centroid.x(), centroid.y(), centroid.z());

	//ROS_DEBUG("Going in");
	for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        tf::Vector3 vec(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        PointT pt;
        vec = resultingTransform * vec;
        octomath::Vector3 coord;

        if (check_hidden) {
        	//ROS_DEBUG("%zu", i);
        	coord.x() = vec.x();
        	coord.y() = vec.y();
        	coord.z() = vec.z();
        } else if (check_visible) {
        	//TODO: Confirm this transformation is correct.
       	    coord.x() = cloud.points[i].x + ownTransform.getOrigin().getX() - centroid.x();
       	    coord.y() = cloud.points[i].y + ownTransform.getOrigin().getY() - centroid.y();
       	    coord.z() = cloud.points[i].z + ownTransform.getOrigin().getZ() - centroid.z();
       	    const tf::Quaternion rot(ownTransform.getRotation());
       	    tf::Vector3 orig(coord.x() - ownTransform.getOrigin().getX(), coord.y() - ownTransform.getOrigin().getY(), coord.z() - ownTransform.getOrigin().getZ());
       	    tf::Vector3 moved = orig.rotate(rot.getAxis(), rot.getAngle());
       	    coord.x() = moved.getX() + ownTransform.getOrigin().getX();

        	pt.x = coord.x();
        	pt.y = coord.y();
        	pt.z = coord.z();
        	//pt.r = cloud.points[i].r;
        	//pt.g = cloud.points[i].g;
        	//pt.b = cloud.points[i].b;
        	temp.points[i] = pt;
        }

        octomap::OcTreeKey key;
		
        // this can happen when we have an empty octree, which would of course not cover the object, so return false
        if (!octree->genKey(coord, key))
            return false;

        octomap::OcTreeNode *node = octree->search(key);
		
        if (check_hidden && !node)
        	//This coordinate is in free space => not hidden
            return false;
        else if (check_hidden && node && !octree->isNodeOccupied(node))
        	//This coordinate is in free space
        	return false;
        else if (check_visible && node && octree->isNodeOccupied(node))
        	//This coordinate is in occupied or hidden space
        	return false;
        //else if (check_visible && !node)
        	//This coordinate is neither in occupied nor free space
        	//return false;
    }

	if (check_visible) {
		sensor_msgs::PointCloud2 temp2;
		toROSMsg(temp,temp2);
		temp2.header.frame_id = "base_link";
		newPosePub_.publish(temp2);
	}
    return true;
}

void Planner::make_grid(vector<sensor_msgs::PointCloud2> config)
{
	ROS_INFO("Imposing a grid.");
	float grid_resolution = 0.0;
	for (size_t i = 0; i < config.size(); i++)
	{
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(config[i], *cloud);
		tf::Vector3 min, max;
		minmax3d(min, max, cloud);
		float x_size = max.getX() - min.getX();
		float y_size = max.getY() - min.getY();
		if (std::max(x_size, y_size) > grid_resolution)
			grid_resolution = std::max(x_size, y_size);
	}
	ROS_INFO("Grid resolution = %f", grid_resolution);
	grid_resolution_ = grid_resolution;
}


void Planner::display_grid()
{
	int n_rows = ceil((BB_MAX.getX() - BB_MIN.getX())/grid_resolution_);
	int n_cols = ceil((BB_MAX.getY() - BB_MIN.getY())/grid_resolution_);

	//Display grid in RViz
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "grid";
	marker.id = 0;
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	//marker.pose = pose;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.005;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(0.0);
	vector<geometry_msgs::Point> points;
	points.resize(2*(n_rows+1+n_cols+1));
	geometry_msgs::Point pt;
	pt.z = tableHeight_;
	for (int r = 0; r < n_rows; r++) {
		pt.x = BB_MIN.getX() + r*grid_resolution_;
		pt.y = BB_MAX.getY();
		points[2*r] = pt;
		pt.y = BB_MIN.getY();
		points[2*r+1] = pt;
	}
	for (int c = 0; c < n_cols; c++) {
		pt.x = BB_MAX.getX();
		pt.y = BB_MAX.getY() - c*grid_resolution_;
		points[2*n_rows + 2*c] = pt;
		pt.x = BB_MIN.getX();
		points[2*n_rows + 2*c+1] = pt;
	}
	marker.points = points;
	for (int c = 0; c < 100; c++)
		gridPub_.publish(marker);
}

void Planner::findGridLocations(vector<sensor_msgs::PointCloud2> config)
{
	//Find row-col location for each cluster in the grid
	vector<vector<int> > grid_locations;
	grid_locations.resize(config.size());
	for (size_t i = 0; i < config.size(); i++)
	{
		grid_locations[i].resize(4);
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(config[i], *cloud);
		tf::Vector3 min, max;
		minmax3d(min, max, cloud);
		//Rows are front to back, columns are left to right
		grid_locations[i][0] = ceil((min.getX() - BB_MIN.getX())/grid_resolution_);		//min row
		grid_locations[i][1] = ceil((BB_MAX.getY() - max.getY())/grid_resolution_);		//min col
		//grid_locations[i][2] = ceil((max.getX() - BB_MIN.getX())/grid_resolution_);		//max row
		/* Setting max row to min row */
		grid_locations[i][2] = grid_locations[i][0];
		grid_locations[i][3] = ceil((BB_MAX.getY() - min.getY())/grid_resolution_);		//max col
		//grid_locations_.insert(std::pair<int, pair<int, int> >((int)i, make_pair(row, col)));
		ROS_INFO("Grid location for cluster %zu: %d %d %d %d", i, grid_locations[i][0], grid_locations[i][1],
				grid_locations[i][2], grid_locations[i][3]);
	}
	grid_locations_.clear();
	grid_locations_ = grid_locations;
}

bool Planner::inFront(pcl::PointCloud<PointT> cloud, int cluster_idx)
{
	int min_row = grid_locations_[cluster_idx][0];
	int min_col = grid_locations_[cluster_idx][1];
	int max_row = grid_locations_[cluster_idx][2];
	int max_col = grid_locations_[cluster_idx][3];

	bool left_side_blocked = false;
	bool right_side_blocked = false;

	if (min_row == 0 && max_row == 0)
		return true;
	else {
		//Check if this col has any other object in the same col but lesser row
		for (unsigned int i = 0; i < grid_locations_.size(); i++)
		{
			if ((int)i == cluster_idx)
				continue;
			vector<int> temp_vec = grid_locations_[i];
			if (temp_vec[1] == min_col && temp_vec[0] < min_row)
				left_side_blocked = true;
			if (temp_vec[1] == max_col && temp_vec[0] < max_row)
				right_side_blocked = true;
			if (temp_vec[3] == max_col && temp_vec[2] < max_row)
				right_side_blocked = true;
			if (temp_vec[3] == min_col && temp_vec[2] < min_row)
				left_side_blocked = true;

			if (left_side_blocked || right_side_blocked)
				return false;
		}
	}
	//ROS_ERROR("inFront: Should not be here.");
	ROS_INFO("Cluster %d: Exiting inFront", cluster_idx);
	return true;
}

void Planner::findMovable(vector<sensor_msgs::PointCloud2> visible_clusters,
		vector<sensor_msgs::PointCloud2>& movable_clusters, vector<int>& movable_idx)
{
	//Find which clusters have full frontal visibility
	for (size_t i = 0; i < visible_clusters.size(); i++)
	{
		pcl::PointCloud<PointT> cloud;
		fromROSMsg(visible_clusters[i], cloud);

		if (touchesTable(cloud, tableHeight_) && (cloud.points.size() >= 125)) { //The size check is to ensure that we see a big enough cluster
			//This cluster is in contact with the table
			ROS_DEBUG("Cluster %zu touches the table", i);
			//TODO: Need better logic for inFront using bounding boxes
			//if (inFront(cloud, (int)i))	{
			if (inFrontAlt(cluster_to_known_idx_map_[i])) {
				ROS_DEBUG("Cluster %zu is in front and fully visible", i);
				movable_clusters.push_back(visible_clusters[i]);
				//movable_idx.push_back((int)i);
				movable_idx.push_back((int)(cluster_to_known_idx_map_[i]));
			}
		}
	}
	movablePub_.publish(concatClouds(movable_clusters));
	ROS_INFO("Found %zu movable clusters", movable_clusters.size());
	//breakpoint();
}

bool Planner::inFrontAlt(unsigned int cluster_idx)
{
	float this_lefty = knownObjects_[cluster_idx].first.y() + knownObjectDimensions_[cluster_idx].y()/2.f;	//Left boundary
	float this_righty = knownObjects_[cluster_idx].first.y() - knownObjectDimensions_[cluster_idx].y()/2.f;	//Right boundary

	for (size_t i = 0; i < knownObjects_.size(); i++) {
		if (knownObjects_[i].first.x() - knownObjects_[cluster_idx].first.x() >= 0)
			continue;

		float known_lefty = knownObjects_[i].first.y() + knownObjectDimensions_[i].y()/2.f;	//Left boundary
		float known_righty = knownObjects_[i].first.y() - knownObjectDimensions_[i].y()/2.f;	//Right boundary

		if (this_righty - known_lefty < 0.04 && this_lefty >= known_lefty)
			return false;
		else if (known_righty - this_lefty < 0.04 && known_righty >= this_righty)
			return false;
		else if (this_lefty <= known_lefty && this_lefty >= known_righty && this_righty >= known_righty && this_righty <= known_lefty)
			return false;
	}
	return true;
}



bool Planner::generatePercentageIfRemoved(vector<tf::Pose> object_belief,
								 vector<sensor_msgs::PointCloud2> movable_clusters,
								 vector<int> movable_idx,
								 map<int, double>& percentage)
{
	ROS_DEBUG("Generating percentage");
	pcl::PointCloud<PointT>::Ptr percentage_cloud(new pcl::PointCloud<PointT>);

	//create tabletop representation with one cluster missing at a time
	tf::Transform identity;
	identity.setIdentity();
	//percentage.resize(visible_clusters.size());
	pcl::PointCloud<PointT> temp_cloud;
	for (size_t i = 0; i < movable_clusters.size(); i++) {
		ROS_DEBUG("Cluster %zu", i);
		pcl::PointCloud<PointT>::Ptr concat_cloud(new pcl::PointCloud<PointT>);

		for (size_t j = 0; j < movable_clusters.size(); j++) {
			fromROSMsg(movable_clusters[j], temp_cloud);
			if (i == 0 && j == 1)
				*concat_cloud = temp_cloud;
			else if (i != 0 && j == 0)
				*concat_cloud = temp_cloud;
			else if (j != i) {
				*concat_cloud += temp_cloud;
			}
		}
		//ROS_DEBUG("Creating a TableTopObject (TTO) for cloud without cluster %zu", i);

		size_t num_remaining = 0;
		sensor_msgs::PointCloud2 concat_cloud2;
		toROSMsg(*concat_cloud, concat_cloud2);
		TableTopObject concatCloudTTO = createTTO(concat_cloud2);
		for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it != object_belief.end(); it++) {
			if (checkHiddenOrVisible(targetCloud2_, octree_, *it, identity, true, false))
				num_remaining++;
		}

		double this_percentage = (object_belief.size() == 0 ? 1 :
							 (object_belief.size() - num_remaining)/ (double) object_belief.size());

		ROS_DEBUG("Removing Cluster %zu would reveal %zu of remaining hypotheses that is %f percent",
				i, object_belief.size() - num_remaining, 100 * this_percentage);

		percentage[movable_idx[i]] = this_percentage;
	}
	return true;
}


double Planner::generatePercentageIfDisplaced(vector<tf::Pose> object_belief,
								 vector<sensor_msgs::PointCloud2> new_config,
								 octomap::OcTree* octree)
{
	ROS_DEBUG("Generating percentage");

	//create tabletop representation for the new config
	tf::Transform identity;
	identity.setIdentity();

	size_t num_remaining = 0;
/*	sensor_msgs::PointCloud2 otherCloud2 = concatClouds(new_config);
	//TableTopObject otherCloudTTO = createTTO(otherCloud2);

	pcl::PointCloud<PointT>::Ptr o_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(otherCloud2, *o_cloud);
	octomap::pose6d octomapCameraPose = octomap::poseTfToOctomap (cameraOrigin_);
	const octomap::point3d sensor_origin(octomapCameraPose.trans());
	octomap::Pointcloud octomapCloud;
	octomap::pointcloudPCLToOctomap (*o_cloud, octomapCloud);
	octomap::OcTree* octree = new octomap::OcTree(0.01);
	octree->insertScan(octomapCloud, sensor_origin);
	//for (size_t i = 0; i < freeVoxels_.size(); i++) {
	for (std::list<octomap::OcTreeVolume>::iterator it = freeVoxels_.begin(); it != freeVoxels_.end(); ++it) {
		//octomap::point3d coord(current_config->points[i].x, current_config->points[i].y, current_config->points[i].z);
		octomap::OcTreeKey key;
		octree->genKey(it->first, key);
		octree->updateNode(key, false);
	}
	//for (size_t i = 0; i < occupiedVoxels_.size(); i++) {
	for (std::list<octomap::OcTreeVolume>::iterator it = occupiedVoxels_.begin(); it != occupiedVoxels_.end(); ++it) {
		octomap::OcTreeKey key;
		octree->genKey(it->first, key);
		octree->updateNode(key, true);
	}
	octree_ = octree;
	ROS_INFO("Displaying octree");
	breakpoint();
*/

	for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it != object_belief.end(); it++) {
		if (checkHiddenOrVisible(targetCloud2_, octree, *it, identity, true, false))
			num_remaining++;
	}

	double this_percentage = (object_belief.size() == 0 ? 1 :
			(object_belief.size() - num_remaining)/ (double) object_belief.size());

	ROS_DEBUG("This move would reveal %zu of remaining hypotheses that is %f percent",
			object_belief.size() - num_remaining, 100 * this_percentage);

	return this_percentage;
}

void Planner::findPossibleMoves(octomap::OcTree* octree,
		vector<sensor_msgs::PointCloud2> movable_clusters,
		vector<int> movable_idx,
		vector<Move>& possible_moves)
{
	ROS_DEBUG("Finding possible moves");
	//Given current configuration and visible objects, find all valid moves of these objects.
	//TableTopObject concatCloudTTO = createTTO(other_cloud);

	for (size_t cluster_idx = 0; cluster_idx < movable_clusters.size(); cluster_idx++) {
	//for (size_t cluster_idx = 0; cluster_idx < 3; cluster_idx++) {

		//Finding source pose of this cluster
		pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
		fromROSMsg(movable_clusters[cluster_idx], *clusterCloud);

		vector<geometry_msgs::Point> extents = find_extents(*clusterCloud);
		tf::Vector3 centroid(extents[0].x + (extents[1].x - extents[0].x)/2.0f,
				extents[2].y + (extents[3].y - extents[2].y)/2.0f,
				extents[4].z + (extents[5].z - extents[4].z)/2.0f);

		tf::Vector3 dimensions(extents[1].x - extents[0].x, extents[3].y - extents[2].y, extents[5].z - extents[4].z);

		tf::Pose source_pose;
		source_pose.setOrigin(centroid);
		source_pose.setRotation(tf::Quaternion(0,0,0,1));

		//Find destination poses
		bool push = false;
		vector<tf::Pose> destination_poses;
		ROS_DEBUG("Finding destination poses for visible cluster %zu", cluster_idx);
		//Check if the object is too big to be grasped, can only be pushed
		//if (max.y() - min.y() > 0.085f) {

		if (extents[3].y - extents[2].y > 0.09f) {
			ROS_INFO("Object too big in y dimension. Can't be grasped from front. Needs to be pushed.");
			tf::Vector3 sampling_bb_min(centroid.x(), max(centroid.y()-0.15f, BB_MIN.y()+dimensions.y()/2.0f+0.01), BB_MIN.z());
			tf::Vector3 sampling_bb_max(min(centroid.x(), BB_MAX.x()-dimensions.x()/2.0f-0.01),
					min(centroid.y()+0.15f, BB_MAX.y()-dimensions.y()/2.0f-0.01), BB_MAX.getZ());
			push = true;
			//samplePose(movable_clusters[cluster_idx], concatCloudTTO, sampling_bb_min, sampling_bb_max, destination_poses, false, true);
			samplePose(movable_clusters[cluster_idx], octree, sampling_bb_min, sampling_bb_max, destination_poses, false, true, push);
		} else {
			//tf::Vector3 sampling_bb_min(centroid.x(), max(centroid.y()-0.15f, BB_MIN.y()+dimensions.y()/2.0f+0.01), BB_MIN.z());
			//tf::Vector3 sampling_bb_max(min(centroid.x(), BB_MAX.x()-dimensions.x()/2.0f-0.01),
				//				min(centroid.y()+0.15f, BB_MAX.y()-dimensions.y()/2.0f-0.01), BB_MAX.getZ());
			tf::Vector3 sampling_bb_min(BB_MIN.x()+0.05f, BB_MIN.y()+0.05f, centroid.z() - 0.02);
			tf::Vector3 sampling_bb_max(BB_MAX.getX()-0.05f, BB_MAX.getY()-0.05f, centroid.z() + 0.02);
			//samplePose(movable_clusters[cluster_idx], concatCloudTTO, sampling_bb_min, sampling_bb_max, destination_poses, false, true);
			samplePose(movable_clusters[cluster_idx], octree, sampling_bb_min, sampling_bb_max, destination_poses, false, true, push);
		}

		//Create moves for this object
		//vector<Move> moves;
		//Move temp_move(visible_idx[cluster_idx], visible_clusters[cluster_idx], source_pose, destination_poses[0]);
		//moves.resize(destination_poses.size(), temp_move);
		ROS_INFO("1. size of free destination poses for object %d = %zu",
				movable_idx[cluster_idx], destination_poses.size());

		//int j = 0;
		for (std::vector<tf::Pose>::iterator it = destination_poses.begin(); it!=destination_poses.end(); it++)
		{
			Move this_move(movable_idx[cluster_idx], movable_clusters[cluster_idx], source_pose, *it, push);
			//moves[j].destPose = *it;
			//j++;
			possible_moves.push_back(this_move);
		}
		//ROS_DEBUG("Samples checked for object %zu", cluster_idx);
		//possible_moves.insert(possible_moves.end(), moves.begin(), moves.end());

		ROS_DEBUG("2. size of possible moves now = %zu", possible_moves.size());
	}
}

void Planner::simulateMove(vector<sensor_msgs::PointCloud2> config,
		Move move, vector<sensor_msgs::PointCloud2>& new_config)
{
	//ROS_DEBUG("Inside simulateMove");
	new_config = config;

	//Find what the new point cloud would be after the cluster has been moved
	sensor_msgs::PointCloud2 moved_cloud2 = move.objectToMove;
	pcl::PointCloud<PointT>::Ptr moved_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(moved_cloud2, *moved_cloud);

	//tf::Pose transform = move.sourcePose.inverseTimes(move.destPose);

	//Assuming only translation
	double translation_x = move.destPose.getOrigin().getX() - move.sourcePose.getOrigin().getX();
	double translation_y = move.destPose.getOrigin().getY() - move.sourcePose.getOrigin().getY();
	double translation_z = move.destPose.getOrigin().getZ() - move.sourcePose.getOrigin().getZ();
	//ROS_DEBUG("Translation in this move: %f %f %f", translation_x, translation_y, translation_z);

	tf::Pose source_to_dest;
	const tf::Vector3 translation(translation_x, translation_y, translation_z);
	source_to_dest.setOrigin(translation);
	const tf::Quaternion rot(move.destPose.getRotation());
	source_to_dest.setRotation(rot);

	for (size_t i = 0; i < moved_cloud->points.size(); i++)
	{
		moved_cloud->points[i].x += translation_x;
		moved_cloud->points[i].y += translation_y;
		moved_cloud->points[i].z += translation_z;

		tf::Vector3 orig(moved_cloud->points[i].x - move.destPose.getOrigin().getX(),
				moved_cloud->points[i].y - move.destPose.getOrigin().getY(),
				moved_cloud->points[i].z - move.destPose.getOrigin().getZ());
		tf::Vector3 moved = orig.rotate(rot.getAxis(), rot.getAngle());	//rot*orig;
		moved_cloud->points[i].x = moved.getX() + move.destPose.getOrigin().getX();
		moved_cloud->points[i].y = moved.getY() + move.destPose.getOrigin().getY();
		moved_cloud->points[i].z = moved.getZ() + move.destPose.getOrigin().getZ();
	}

	//Replace the point cloud for the cluster being moved with the point cloud at the destination
	toROSMsg(*moved_cloud, new_config[move.cluster_idx]);
}

void Planner::plan(int horizon,
		vector<sensor_msgs::PointCloud2> config,
		octomap::OcTree* octree,
		vector<bool> beenMoved,
		vector<Move>& action_sequence_so_far,
		double& total_percentage_revealed_so_far,
		vector<unsigned int>& moved_so_far)
//sensor_msgs::PointCloud2 other_cloud,
{
	//Config: All known objects
	//other_cloud: The whole object cloud with all clusters, big and small
	//movable_clusters: Clusters that are big enough to be manipulated, that are in contact with the table, and in front

	ROS_DEBUG("Plan: horizon %d", horizon);

	//Find movable clusters
	ROS_DEBUG("Finding movable clusters");
	vector<sensor_msgs::PointCloud2> movable_clusters;
	vector<int> movable_idx;
	//findMovable(config, movable_clusters, movable_idx);
	//findMovable(visible_clusters, movable_clusters, movable_idx);
	findMovable(config, movable_clusters, movable_idx);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu movable clusters", horizon, movable_clusters.size());

	if (movable_clusters.size() == 0) {
		ROS_ERROR("No movable clusters. Exiting.");
		return;
	}
	//breakpoint();

	//Detecting target object based on size
	if (horizon == MAX_HORIZON) {
		for (size_t k = 0; k < movable_clusters.size(); k++) {
			//if (fabs(bbx.dimensions.x - target_x) < 0.04 && fabs(bbx.dimensions.y - target_y) < 0.03 && fabs(bbx.dimensions.z - target_z) < 0.02)
			unsigned int idx = movable_idx[k];
			if (fabs(knownObjectDimensions_[idx].x() - target_x) < 0.04 && fabs(knownObjectDimensions_[idx].y() - target_y) < 0.03 && fabs(knownObjectDimensions_[idx].z() - target_z) < 0.02)
			{
				//TODO: Also check for color?
				ROS_ERROR("Target object spotted! SUCCESS!");
				clustersPub_.publish(knownObjects_[idx].second);
				time_t tfound = time(0);
				ROS_INFO("Took %f seconds to find target", totalPlanningTime + difftime(tfound,last_tstart));
				breakpoint();
				//ros::shutdown();
			}
		}
	}

	//Sample target pose
	ROS_DEBUG("Sampling target object pose in occluded space");
	std::vector<tf::Pose> object_posterior_belief;
	//tf::Vector3 sampling_bb_min(BB_MIN.getX()+0.05, BB_MIN.getY()+0.05, BB_MIN.getZ());
	tf::Vector3 sampling_bb_min(BB_MIN.getX()+0.05, BB_MIN.getY()+0.05, tableHeight_+target_z/2.0 - 0.02);
	//tf::Vector3 sampling_bb_max(BB_MAX.getX()-0.05, BB_MAX.getY()-0.05, BB_MAX.getZ());
	tf::Vector3 sampling_bb_max(BB_MAX.getX()-0.05, BB_MAX.getY()-0.05, tableHeight_+target_z/2.0 + 0.02);
	//samplePose(targetCloud2_, createTTO(other_cloud), sampling_bb_min, sampling_bb_max, object_posterior_belief, true, false);
	samplePose(targetCloud2_, octree, sampling_bb_min, sampling_bb_max, object_posterior_belief, true, false, false);
	ROS_INFO("Sampled target pose. Found %zu valid samples", object_posterior_belief.size());
	//breakpoint();

	/*
	if (object_posterior_belief.size() == 0 && horizon == MAX_HORIZON) {
		ROS_ERROR("All space explored.");
		time_t texplored = time(0);
		ROS_INFO("Took %f seconds to explore all space", totalPlanningTime + difftime(texplored,last_tstart));
		breakpoint();
		return;
	}
	*/

	//make_grid(config);
	//if (horizon == MAX_HORIZON) {
		//display_grid();
		//pubCloud("other_cloud_TTO", (createTTO(other_cloud)).getAsCloud() , fixed_frame_);
		//ROS_INFO("Displaying grid.");
		//breakpoint();
	//}
	//findGridLocations(config);

	//Find possible moves
	vector<Move> possible_moves;
	findPossibleMoves(octree, movable_clusters, movable_idx, possible_moves);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu possible moves", horizon, possible_moves.size());

	double info_gain = -1.0;
	vector<Move> best_action_sequence;
	Move best_action = possible_moves[0];

	//For each possible move, do the following.
	unsigned int move_in_collision = 0;
	for (size_t move_idx = 0; move_idx < possible_moves.size(); move_idx++) {
	//for (size_t move_idx = 0; move_idx < (size_t)std::min(5, (int)possible_moves.size()); move_idx++) {
		Move this_move = possible_moves[move_idx];

		if (horizon == MAX_HORIZON)
			moved_so_far.clear();

		//Checking if this object has already been moved in this iteration
		vector<unsigned int>::iterator it = find(moved_so_far.begin(), moved_so_far.end(), this_move.cluster_idx);
		if (it != moved_so_far.end())
			continue;

		ROS_DEBUG("Horizon %d: Simulating move %zu", horizon, move_idx);
		double percentage_revealed_so_far = total_percentage_revealed_so_far;

		vector<Move> action_sequence = action_sequence_so_far;
		action_sequence.push_back(this_move);

		//Simulate move & find resulting config
		pcl::PointCloud<PointT>::Ptr config_cloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		//fromROSMsg(other_cloud, *config_cloud);
		fromROSMsg(concatClouds(config), *config_cloud);
		ROS_DEBUG("Publishing current simulated config cloud on topic current_simulated_config");
		pubCloud("current_simulated_config", config_cloud, fixed_frame_);
		vector<sensor_msgs::PointCloud2> new_config;
		simulateMove(config, this_move, new_config);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);
		

		if (move_idx == 6) {
			ROS_INFO("Displaying new config after one of the moves.");
			//breakpoint();
		}


		bool in_contact = false;
		//TODO: This is not working reliably!
		for (unsigned int m = 0; m < new_config.size(); m++) {
			if (m != this_move.cluster_idx)
				if (incontact(new_config[m], new_config[this_move.cluster_idx], 0.05, 150, 2000)) {
					ROS_DEBUG("The moved cloud touches some other object. reject this move.");
					in_contact = true;
					move_in_collision++;
					break;
				}
		}

		//breakpoint();
		//if (in_contact)	continue;

		moved_so_far.push_back(this_move.cluster_idx);
		//Generate new octree resulting from this move
		octomap::OcTree* new_octree = new octomap::OcTree(0.01);
		//for (unsigned int k = 0; k < beenMoved_.)
		beenMoved[cluster_to_known_idx_map_[this_move.cluster_idx]] = true;
		generateOctree(new_config, beenMoved, new_octree);

		//Find percentage revealed by this move
		double this_percentage_revealed = generatePercentageIfDisplaced(object_posterior_belief, new_config, new_octree);
		//double this_percentage_revealed = percentage[this_move.cluster_idx];
		percentage_revealed_so_far += this_percentage_revealed;	// + percentage_revealed_so_far;
		ROS_DEBUG("percentage revealed by this move = %f", percentage_revealed_so_far);
		
		if (horizon > 1) {
			//Plan recursively
			//plan(horizon-1, new_config, octree, concatClouds(new_config), action_sequence, percentage_revealed_so_far);
			plan(horizon-1, new_config, new_octree, beenMoved, action_sequence, percentage_revealed_so_far, moved_so_far);
		}

		if (percentage_revealed_so_far > info_gain) {
			//This action reveals more than previously tested actions
			info_gain = percentage_revealed_so_far;
			best_action_sequence = action_sequence;
		}
	}
	
	if (info_gain == 0.0 && horizon == MAX_HORIZON) {
		ROS_INFO("Did not find an informative move. Increasing the horizon.");
		breakpoint();
		horizon++;
		MAX_HORIZON++;
		plan(horizon, config, octree, beenMoved, action_sequence_so_far, total_percentage_revealed_so_far, moved_so_far);
	} else {
		ROS_DEBUG("Horizon %d: Found %u moves in collision", horizon, move_in_collision);
		total_percentage_revealed_so_far = info_gain;
		action_sequence_so_far = best_action_sequence;
		//best_next_action_sequence.insert(best_next_action_sequence.begin(), best_action);
		ROS_DEBUG("Found the best next action sequence for horizon %d of length %zu with info gain = %f",
				horizon, action_sequence_so_far.size(), info_gain);
	}
}


void Planner::random_plan(int horizon,
		vector<sensor_msgs::PointCloud2> config,
		octomap::OcTree* octree,
		sensor_msgs::PointCloud2 other_cloud,
		vector<Move>& action_sequence_so_far,
		double& total_percentage_revealed_so_far)
{
	ROS_DEBUG("Plan: horizon %d", horizon);

	//make_grid(config);
	//if (horizon == MAX_HORIZON)
		//display_grid();
	//findGridLocations(config);

	//Find movable clusters
	ROS_DEBUG("Finding movable clusters");
	vector<sensor_msgs::PointCloud2> movable_clusters;
	vector<int> movable_idx;
	findMovable(config, movable_clusters, movable_idx);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu movable clusters", horizon, movable_clusters.size());

	if (movable_clusters.size() == 0) {
		ROS_ERROR("No movable clusters. Exiting.");
		return;
	}

	//Detecting target object based on size
	for (size_t k = 0; k < movable_clusters.size(); k++) {
		//if (fabs(bbx.dimensions.x - target_x) < 0.04 && fabs(bbx.dimensions.y - target_y) < 0.03 && fabs(bbx.dimensions.z - target_z) < 0.02)
		unsigned int idx = movable_idx[k];
		if (fabs(knownObjectDimensions_[idx].x() - target_x) < 0.04 && fabs(knownObjectDimensions_[idx].y() - target_y) < 0.03 && fabs(knownObjectDimensions_[idx].z() - target_z) < 0.02)
		{
			ROS_ERROR("Target object spotted! SUCCESS!");
			clustersPub_.publish(knownObjects_[idx].second);
			time_t tfound = time(0);
			ROS_INFO("Took %f seconds to find target", totalPlanningTime + difftime(tfound,last_tstart));
			breakpoint();
			//ros::shutdown();
		}
	}

	//Find possible moves
	vector<Move> possible_moves;
	findPossibleMoves(octree, movable_clusters, movable_idx, possible_moves);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu possible moves", horizon, possible_moves.size());

	vector<Move> best_action_sequence;
	Move this_move = possible_moves[0];
	while (true) {
		//Randomly select move
		int move_idx = rand() % (int)(possible_moves.size());

		ROS_DEBUG("Horizon %d: Simulating move %d", horizon, move_idx);
		this_move = possible_moves[move_idx];

		//Simulate move & find resulting config
		pcl::PointCloud<PointT>::Ptr config_cloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		//fromROSMsg(other_cloud, *config_cloud);
		fromROSMsg(concatClouds(config), *config_cloud);
		ROS_DEBUG("Publishing current simulated config cloud on topic current_simulated_config");
		pubCloud("current_simulated_config", config_cloud, fixed_frame_);
		vector<sensor_msgs::PointCloud2> new_config;
		simulateMove(config, this_move, new_config);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);

		bool in_contact = false;
		for (unsigned int m = 0; m < new_config.size(); m++) {
			if (m != this_move.cluster_idx)
				if (incontact(new_config[m], new_config[this_move.cluster_idx], 0.02, 150, 2000)) {
					ROS_DEBUG("The moved cloud touches some other object. reject this move.");
					in_contact = true;
					//move_in_collision++;
					break;
				}
		}

		//breakpoint();
		if (in_contact)	continue;

		vector<Move> action_sequence = action_sequence_so_far;
		action_sequence.push_back(this_move);
		if (horizon > 1) {
			//Plan recursively
			//TODO: Send updated octree
			random_plan(horizon-1, new_config, octree, concatClouds(new_config), action_sequence, total_percentage_revealed_so_far);
		}

		best_action_sequence = action_sequence;
		break;
	}

	action_sequence_so_far = best_action_sequence;
	//best_next_action_sequence.insert(best_next_action_sequence.begin(), best_action);
	ROS_DEBUG("Found the best next action sequence for horizon %d of length %zu", horizon, action_sequence_so_far.size());
}


void Planner::execute_plan()
{
	tum_os::Execute_Plan execute_call;
	for (size_t i = 0; i < action_sequence_.size(); i++)
	{
		execute_call.request.cluster_idx.push_back(action_sequence_[i].cluster_idx);
		execute_call.request.object_to_move.push_back(action_sequence_[i].objectToMove);
		execute_call.request.source_pose.push_back(tfPoseToGeometryPose(action_sequence_[i].sourcePose));
		execute_call.request.dest_pose.push_back(tfPoseToGeometryPose(action_sequence_[i].destPose));
		execute_call.request.push.push_back(action_sequence_[i].push);
	}
	execute_call.request.table_height = tableHeight_;

	ROS_INFO("Calling the execute plan service");
	if (manipulateClient_.call(execute_call)) {
		if (execute_call.response.result) {
			ROS_INFO("Executed the plan");
		} else {
			ROS_ERROR("Execute service called but execution failed.");
			breakpoint();
		}
	} else {
		ROS_ERROR("Failed to call execute plan service.");
		breakpoint();
	}
	new_data_wanted_ = true;
	//return true;
}


void Planner::getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
		geometry_msgs::PoseStamped &pose_stamped,
		geometry_msgs::Vector3 &dimensions) {
	object_manipulation_msgs::FindClusterBoundingBox2 srv;
	srv.request.cluster = cluster;
	if (!bbx_client_.call(srv.request, srv.response)) {
		ROS_ERROR("Failed to call cluster bounding box client");
		//throw CollisionMapException("Failed to call cluster bounding box client");
	}
	pose_stamped = srv.response.pose;
	dimensions = srv.response.box_dims;
	if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0) {
		ROS_ERROR("Cluster bounding box client returned an error (0.0 bounding box)");
		//throw CollisionMapException("Bounding box computation failed");
	}
}

/*
void Planner::simulate_plan_execution()
{
	vector<sensor_msgs::PointCloud2> current_config = clustersDetected_;
	vector<sensor_msgs::PointCloud2> new_config;

	geometry_msgs::PoseStamped source_pose, dest_pose;
	source_pose.header.frame_id = "base_link";
	source_pose.header.stamp = ros::Time::now();
	dest_pose.header.frame_id = "base_link";
	dest_pose.header.stamp = ros::Time::now();

	bool action_success = true;
	vector<octomap::point3d> now_free;
	for (size_t i = 0; i < action_sequence_.size(); i++)
	{
		ROS_INFO("Executing move %zu in simulation", i);
		source_pose.pose = tfPoseToGeometryPose(action_sequence_[i].sourcePose);
		dest_pose.pose = tfPoseToGeometryPose(action_sequence_[i].destPose);
		source_pose_pub_.publish(source_pose);
		dest_pose_pub_.publish(dest_pose);
		//breakpoint();

		//Executing plan in simulation
		simulateMove(current_config, action_sequence_[i], new_config);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);
		ROS_INFO("Published new config on new_simulated_config topic");

		ROS_INFO("Displaying old octree before updating it");
		breakpoint();
		display_octree(octree_);
		breakpoint();



		//Update free & occupied nodes
		ROS_INFO("Updating octree now");
		if (action_success) {
			pcl::PointCloud<PointT> new_moved_object, old_moved_object;
			fromROSMsg(new_config[action_sequence_[i].cluster_idx], new_moved_object);
			fromROSMsg(current_config[action_sequence_[i].cluster_idx], old_moved_object);
			//updateHiddenVoxels(tf::Vector3(sensor_origin.x(), sensor_origin.y(), sensor_origin.z()), octree_,
				//	current_config[action_sequence_[i].cluster_idx], tableHeight_, false);
			//updateHiddenVoxels(tf::Vector3(sensor_origin.x(), sensor_origin.y(), sensor_origin.z()), octree_,
				//				new_config[action_sequence_[i].cluster_idx], tableHeight_, true);

			for (size_t p = 0; p < new_moved_object.points.size(); p++) {
				octomap::point3d new_coord(new_moved_object.points[p].x, new_moved_object.points[p].y, new_moved_object.points[p].z);
				octomap::OcTreeKey new_key;
				octree_->genKey(new_coord, new_key);
				octree_->updateNode(new_key, true, true);
			}
			octree_->getFreespace(freeVoxels_);
			ROS_INFO("Free voxels before update: %zu", freeVoxels_.size());

			for (size_t p = 0; p < old_moved_object.points.size(); p++) {
				octomap::point3d old_coord(old_moved_object.points[p].x, old_moved_object.points[p].y, old_moved_object.points[p].z);
				octomap::OcTreeKey old_key;
				octree_->genKey(old_coord, old_key);
				octomap::OcTreeNode* now_free_node = octree_->updateNode(old_key, false);
				now_free_node->setValue(10.0f);
				//ROS_INFO("Occupancy of now free node = %f, value = %f", now_free_node->getOccupancy(), now_free_node->getValue());
				freeVoxels_.push_back(std::make_pair(old_coord, octree_->getResolution()));
				//octomap::OcTreeNode* now_free_node = octree_->search(old_key);
				//octree_->integrateMiss(now_free_node);
			}

			ROS_INFO("Free voxels after update: %zu", freeVoxels_.size());
		}
		current_config = new_config;
		new_config.clear();
		//breakpoint();
	}
	//octree_->updateInnerOccupancy();
	//ROS_INFO("Displaying old octree after updating it");
	//breakpoint();
	//display_octree(octree_);
	//breakpoint();

	ROS_INFO("Sanity check: Free voxels after update: %zu", freeVoxels_.size());
	//octree_->getOccupied(occupiedVoxels_);


	sensor_msgs::PointCloud2 newObjectCloud2 = concatClouds(current_config);
	currentConfigPCL_ = newObjectCloud2;
	newObjectCloud2.header.frame_id = fixed_frame_;
	//Euclidean segmentation
	cluster(newObjectCloud2, 0.02, 50, 10000, clustersDetected_);

	pcl::PointCloud<PointT>::Ptr shelf_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(newObjectCloud2, *shelf_cloud);
	pubCloud("current_simulated_config", shelf_cloud, fixed_frame_);
	ROS_INFO("Published current config on current_simulated_config topic");
	octomap::Pointcloud octomapCloud;
	octomap::pointcloudPCLToOctomap (*shelf_cloud, octomapCloud);
	octomap::point3d octo_bbmin(BB_MIN.x(), BB_MIN.y(), BB_MIN.z()-0.01), octo_bbmax(BB_MAX.x(), BB_MAX.y(), BB_MAX.z());

	octomap::OcTree* octree = new octomap::OcTree(0.01);
	ROS_INFO("Size of octree_ = %zu", octree_->size());
	if (octree_->size() == 1) {
		ROS_INFO("Octree size is 1");
		octree_->insertScan(octomapCloud, octomapCameraOrigin_);
		updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree_, newObjectCloud2, tableHeight_, true);
		updateFreeVoxels(octree_);
		ROS_INFO("Displaying octree");
		display_octree(octree_);
		breakpoint();
	} else {
		//octomap::OcTree* octree = new octomap::OcTree(0.01);
		octree->setBBXMin(octo_bbmin);
		octree->setBBXMax(octo_bbmax);
		octree->bbxSet();
		octree->useBBXLimit(true);
		//octree->enableChangeDetection(true);

		ROS_INFO("WTF:Displaying new octree.");
		display_octree(octree);
		breakpoint();

		ROS_INFO("Size of octree = %zu", octree->size());
		ROS_INFO("Sanity check: Free voxels after update: %zu", freeVoxels_.size());
		int wtf = 0, cool = 0, updated_nodes = 0;

		unsigned int n_x_voxels = (octree->getBBXMax().x() - octree->getBBXMin().x())/octree->getResolution();
		unsigned int n_y_voxels = (octree->getBBXMax().y() - octree->getBBXMin().y())/octree->getResolution();
		unsigned int n_z_voxels = (octree->getBBXMax().z() - octree->getBBXMin().z())/octree->getResolution();

		ROS_INFO("num voxels = %u", n_x_voxels*n_y_voxels*n_z_voxels);
		for (size_t i = 0; i < n_x_voxels; i++) {
			for (size_t j = 0; j < n_y_voxels; j++)
				for (size_t k = 0; k < n_z_voxels; k++) {
					//Find voxel center
					octomap::point3d center(octree->getBBXMin().x()+(i+1)*octree->getResolution(),
							octree->getBBXMin().y()+(j+1)*octree->getResolution(),
							octree->getBBXMin().z()+(k+1)*octree->getResolution());
					octomap::OcTreeKey key;

					// this can happen when we have an empty octree, which would of course not cover the object, so return false
					if (!octree->genKey(center, key)) {
						ROS_ERROR("Octree is empty!");
						return;
					}

					octomap::OcTreeNode *node = octree->search(key);
					if ((node == NULL) || (!octree->isNodeOccupied(node))) {
						//This coordinate should be occupied
						updated_nodes++;
						octree->updateNode(key, true);
						//freeVoxels.push_back(std::make_pair(center, octree->getResolution()));
					}
				}
		}
		ROS_INFO("updated = %d, wtf = %d, cool = %d", updated_nodes, wtf, cool);
		ROS_INFO("Size of octree = %zu", octree->size());
		std::list<octomap::OcTreeVolume> fv;
		octree->getFreespace(fv);
		ROS_INFO("Num free voxels = %zu", fv.size());
		ROS_INFO("WTF:Displaying new octree.");
		display_octree(octree);
		breakpoint();

		for (std::list<octomap::OcTreeVolume>::iterator it = freeVoxels_.begin(); it != freeVoxels_.end(); ++it) {
			octomap::OcTreeKey key;
			ROS_INFO("%d",octree->genKey(it->first, key));
			octomap::OcTreeNode *node = octree->search(it->first);
			if (!node) {
				octomap::OcTreeNode* temp = octree->updateNode(key, false);
				updated_nodes++;
				if (temp->getOccupancy() > 0.4)
					wtf++;
				else cool++;
			}
		}
		ROS_INFO("updated= %d, wtf = %d, cool = %d", updated_nodes, wtf, cool);

		ROS_INFO("Displaying new octree after updating free voxels only.");
		breakpoint();
		display_octree(octree_);
		breakpoint();

		ROS_INFO("Now updating occupied voxels");

		for (size_t i = 0; i < n_x_voxels; i++) {
			for (size_t j = 0; j < n_y_voxels; j++)
				for (size_t k = 0; k < n_z_voxels; k++) {
					//Find voxel center
					octomap::point3d center(octree_->getBBXMin().x()+(i+1)*octree_->getResolution(),
							octree_->getBBXMin().y()+(j+1)*octree_->getResolution(),
							octree_->getBBXMin().z()+(k+1)*octree_->getResolution());
					octomap::OcTreeKey key;

					// this can happen when we have an empty octree, which would of course not cover the object, so return false
					if (!octree_->genKey(center, key)) {
						ROS_ERROR("Octree is empty!");
						return;
					}

					octomap::OcTreeNode *node = octree_->search(key);
					if (!node) {
						//This coordinate should be occupied
						octree_->updateNode(key, true);
						//freeVoxels.push_back(std::make_pair(center, octree->getResolution()));
					}
				}
		}

	//	octree->insertScan(octomapCloud, octomapCameraOrigin_);
	//	updateHiddenVoxels(tf::Vector3(octomapCameraOrigin_.x(), octomapCameraOrigin_.y(), octomapCameraOrigin_.z()), octree, newObjectCloud2, tableHeight_, true);

		ROS_INFO("Displaying new octree after updating occupied voxels.");
		breakpoint();
		display_octree(octree_);
		breakpoint();

		//updateFreeVoxels(octree);

		octree_ = octree;
		ROS_INFO("Displaying current octree");
		breakpoint();
		display_octree(octree_);
		breakpoint();
	}

	//call_plan(newObjectCloud2);
}
*/


int main (int argc, char** argv)
{
	if (argc != 2) {
		ROS_ERROR("Insufficient number of arguments. Give horizon length as argument.");
		return -1;
	}

	/*
	char* h, *bb_max;
	for (int i = 1; i < argc; i++) {
		if (i + 1 != argc) {// Check that we haven't finished parsing already
			if (string(argv[i]) == "-horizon")
				bb_min = argv[i + 1];
			else if (string(argv[i]) == "-size")
				bb_max = argv[i + 1];
		}
	}
	*/

	ros::init(argc, argv, "planner");
	ros::NodeHandle n;
	//octomap::OcTree tree(0.05);
	Planner planner(n, atoi(argv[1]));
	//pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    //tf::Stamped<tf::Pose> fixed_to_ik;
    //tf::Stamped<tf::Pose> base_to_camera;
	
    //getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1));
	//fixed_to_ik = getPose(fixed_frame_, ik_frame_);
	//base_to_camera = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());
		
	ros::spin();
	return 0;
}


/*
 void Planner::samplePose(sensor_msgs::PointCloud2 target_cloud2,
						 TableTopObject otherCloudTTO,
						 tf::Vector3 sampling_bb_min,
						 tf::Vector3 sampling_bb_max,
						 vector<tf::Pose>& object_posterior_belief,
						 bool check_hidden,
						 bool check_visible)
{
	//This function samples the whole space for valid poses of the object_cloud.
	//Then it checks whether they lie in the free space and returns an object belief.

	if (check_hidden && !check_visible)
		ROS_DEBUG("Sampling in occluded space");
	else if (!check_hidden && check_visible)
		ROS_DEBUG("Sampling in free space");


	//ROS_DEBUG("before creating samples");
	int n_belief = 500;
	if (check_hidden)
		n_belief = 10000;
    std::vector<tf::Pose> object_belief;
    //for (int k =0; k < 100000; k++)
    for (int k = 0; k < n_belief; k++)
        object_belief.push_back(vdc_pose_bound(sampling_bb_min, sampling_bb_max,k));

    //ROS_DEBUG("samples created");
    pub_belief("object_belief", object_belief);

    pcl::PointCloud<PointT>::Ptr targetCloud(new pcl::PointCloud<PointT>);
    fromROSMsg(target_cloud2, *targetCloud);

    tf::Vector3 min, max;
    minmax3d(min, max, targetCloud);
    float object_height = max.getZ() - min.getZ();
    //ROS_DEBUG("Cloud z midpoint = %f", min.z() + (max.z() - min.z())/2.0);

    tf::Transform identity;
    identity.setIdentity();
    for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it!=object_belief.end(); it++)
    {
    	//Check if the resulting pose touches the table
    	if (check_visible && (it->getOrigin().getZ() - object_height/2 - tableHeight_ <= 0.02) &&
    			(it->getOrigin().getZ() - object_height/2 - tableHeight_ > 0)) {
    		//Now check if the pose is hidden or not
    		if (checkHiddenOrVisible(target_cloud2, otherCloudTTO, *it, identity, check_hidden, check_visible))
    			object_posterior_belief.push_back(*it);
    	} else if (check_hidden && (it->getOrigin().getZ() - object_height/2 - tableHeight_ <= 0.02) &&
    			(it->getOrigin().getZ() - object_height/2 - tableHeight_ > 0))
    		//Reject samples that are not on table but in the air.
    		if (checkHiddenOrVisible(target_cloud2, otherCloudTTO, *it, identity, check_hidden, check_visible))
    			object_posterior_belief.push_back(*it);
    }

    //ROS_DEBUG("samples checked");
    ROS_DEBUG("Size of object belief = %zu", object_posterior_belief.size());
    pub_belief("valid_pose_samples",object_posterior_belief);
    //breakpoint();
}

bool Planner::checkHiddenOrVisible(sensor_msgs::PointCloud2 object_cloud2,
						  TableTopObject otherCloudTTO,
						  tf::Transform ownTransform,
						  tf::Transform otherTransform,
						  bool check_hidden,
						  bool check_visible)
{
	//ROS_DEBUG("Inside checkHiddenOrVisible");
	//This function checks if the object_cloud is hidden by the other_cloud.
	//Returns true if hidden, false if in free space.
	tf::Transform resultingTransform = otherTransform.inverseTimes(ownTransform);
	geometry_msgs::Transform trans;

	pcl::PointCloud<PointT> cloud, temp;
	fromROSMsg(object_cloud2, cloud);
	temp.points.resize(cloud.points.size());

	pcl::PointCloud<PointT>::Ptr targetCloud(new pcl::PointCloud<PointT>);
	fromROSMsg(object_cloud2, *targetCloud);
	tf::Vector3 min, max;
	minmax3d(min, max, targetCloud);
	tf::Vector3 centroid(min.x() + (max.x() - min.x())/2.0, min.y() + (max.y() - min.y())/2.0, min.z() + (max.z() - min.z())/2.0);
	//ROS_DEBUG("Cloud midpoint = %f, %f, %f", centroid.x(), centroid.y(), centroid.z());

	//ROS_DEBUG("Going in");
	for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        tf::Vector3 vec(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        PointT pt;
        vec = resultingTransform * vec;
        octomath::Vector3 coord;

        if (check_hidden) {
        	//ROS_DEBUG("%zu", i);
        	coord.x() = vec.x();
        	coord.y() = vec.y();
        	coord.z() = vec.z();
        } else if (check_visible) {
       	    coord.x() = cloud.points[i].x + ownTransform.getOrigin().getX() - centroid.x();
       	    coord.y() = cloud.points[i].y + ownTransform.getOrigin().getY() - centroid.y();
       	    coord.z() = cloud.points[i].z + ownTransform.getOrigin().getZ() - centroid.z();

        	pt.x = coord.x();
        	pt.y = coord.y();
        	pt.z = coord.z();
        	//pt.r = cloud.points[i].r;
        	//pt.g = cloud.points[i].g;
        	//pt.b = cloud.points[i].b;
        	temp.points[i] = pt;
        }

        octomap::OcTreeKey key;

        // this can happen when we have an empty octree, which would of course not cover the object, so return false
        if (!otherCloudTTO.m_octoMap->octree.genKey(coord, key))
            return false;

        octomap::OcTreeNode *node = otherCloudTTO.m_octoMap->octree.search(key);

        if (!node && check_hidden)
        	//This coordinate is either in occupied or free space => Not hidden
            return false;
        else if (check_visible && node && otherCloudTTO.m_octoMap->octree.isNodeOccupied(node))
        	return false;

        //if (node && (!otherObject.m_octoMap->octree.isNodeOccupied(node)))
        //  return false;
    }
	//ROS_DEBUG("Coming out");

	if (check_visible) {
		sensor_msgs::PointCloud2 temp2;
		toROSMsg(temp,temp2);
		temp2.header.frame_id = "base_link";
		newPosePub_.publish(temp2);
	}
    return true;
}
 */
