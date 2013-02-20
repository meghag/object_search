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

	new_data_wanted_ = false;

	//generate object we search as a pointcloud
	pcl::PointCloud<PointT>::Ptr object_cloud (new pcl::PointCloud<PointT>);
	ROS_INFO("Filling the point cloud for my object");

	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	object_cloud->width  = 100;
	object_cloud->height = 1;
	object_cloud->points.resize (object_cloud->width * object_cloud->height);

	// Generate the data
	for (size_t i = 0; i < object_cloud->points.size (); ++i)
	{
		object_cloud->points[i].x = 0.05 * rand () / (RAND_MAX + 1.0f);
		object_cloud->points[i].y = 0.03 * rand () / (RAND_MAX + 1.0f);
		object_cloud->points[i].z = 0.04 * rand () / (RAND_MAX + 1.0f);
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
	ROS_INFO("MG: Publishing my object cloud on topic my_object");
	pubCloud("my_object", object_cloud, fixed_frame_);

	//breakpoint();

	base_to_camera_ = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str(),ros::Time(0));
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
	TableTopObject cloudTTO(base_to_camera_.getOrigin(), (float)BB_MIN.z(), *cloud);
	//ROS_DEBUG("Publishing the other cloud TTO on topic other_cloud_TTO");
	//pubCloud("other_cloud_TTO", otherCloudTTO.getAsCloud() , fixed_frame_);
	return cloudTTO;
}

//void Planner::planRequestCallback(const tum_os::PlanRequest::ConstPtr& plan_request)
bool Planner::planRequestCallback(tum_os::PlanService::Request &plan_request,
                   tum_os::PlanService::Response &plan_response)
{
	//clusters_msg  = *(ros::topic::waitForMessage<tum_os::Clusters>("/clusters"));

	objectCloud2_ = plan_request.object_cloud;
	tableHeight_ = plan_request.table_height;
	BB_MIN = tf::Vector3(plan_request.bb_min[0], plan_request.bb_min[1], plan_request.bb_min[2]);
	BB_MAX = tf::Vector3(plan_request.bb_max[0], plan_request.bb_max[1], plan_request.bb_max[2]);
	ROS_INFO("table height = %f", tableHeight_);
	objectCloudPub_.publish(objectCloud2_);

	//if (1) {
	//Display BBX in RViz
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "bbx";
	marker.id = 1;
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	//marker.pose = pose;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
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
	//}

	//TODO: Fill in the hollows of cluster point clouds
	//Euclidean segmentation
	cluster(objectCloud2_, 0.02, 50, 10000, clustersDetected_);
	//cluster(objectCloud2_, 0.02, 300, 1000, clustersDetected_);

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
	time_t tstart, tend;
	ROS_INFO("Starting to plan.");
	tstart = time(0);
	plan(MAX_HORIZON, clustersDetected_, objectCloud2_, best_next_action_sequence, total_percentage_revealed_so_far);
	tend = time(0);
	ROS_INFO("Found the best next action sequence for horizon %d of length %zu", MAX_HORIZON, best_next_action_sequence.size());
	ROS_INFO("It took %f seconds to plan.", difftime(tend, tstart));
	action_sequence_ = best_next_action_sequence;
	ROS_INFO("%zu moves in the action sequence", best_next_action_sequence.size());

	geometry_msgs::PoseStamped source_pose, dest_pose;
	source_pose.header.frame_id = "base_link";
	source_pose.header.stamp = ros::Time::now();
	dest_pose.header.frame_id = "base_link";
	dest_pose.header.stamp = ros::Time::now();
	for (size_t i = 0; i < best_next_action_sequence.size(); i++)
	{
		ROS_INFO("cluster idx = %d, \n source location = %f %f %f, \n dest location = %f %f %f", best_next_action_sequence[i].cluster_idx,
				best_next_action_sequence[i].sourcePose.getOrigin().getX(), best_next_action_sequence[i].sourcePose.getOrigin().getY(),
				best_next_action_sequence[i].sourcePose.getOrigin().getZ(), best_next_action_sequence[i].destPose.getOrigin().getX(),
				best_next_action_sequence[i].destPose.getOrigin().getY(), best_next_action_sequence[i].destPose.getOrigin().getZ());

		source_pose.pose = tfPoseToGeometryPose(best_next_action_sequence[i].sourcePose);
		dest_pose.pose = tfPoseToGeometryPose(best_next_action_sequence[i].destPose);
		source_pose_pub_.publish(source_pose);
		dest_pose_pub_.publish(dest_pose);
		breakpoint();
	}

	plan_response.result = true;
	new_data_wanted_ = true;

	//Execute plan
	//execute_plan();

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
	return true;
}

void Planner::samplePose(sensor_msgs::PointCloud2 target_cloud2, 
						 TableTopObject otherCloudTTO,
						 vector<tf::Pose>& object_posterior_belief,
						 bool check_hidden,
						 bool check_visible)
{
	//This function samples the whole space for valid poses of the object_cloud.
	//Then it checks whether they lie in the free space and returns an object belief.
	/*
	if (check_hidden && !check_visible)
		ROS_DEBUG("Sampling in occluded space");
	else if (!check_hidden && check_visible)
		ROS_DEBUG("Sampling in free space");
	*/
	
	//ROS_DEBUG("before creating samples");
	int n_belief = 200;
	if (check_hidden)
		n_belief = 10000;
    std::vector<tf::Pose> object_belief;
    //for (int k =0; k < 100000; k++)
    tf::Vector3 sampling_bb_min(BB_MIN.getX()+0.1, BB_MIN.getY()+0.1, BB_MIN.getZ());
    tf::Vector3 sampling_bb_max(BB_MAX.getX()-0.1, BB_MAX.getY()-0.1, BB_MAX.getZ());
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
    	//ROS_DEBUG("%f, %f, %f", it->getOrigin().getX(), it->getOrigin().getY(), it->getOrigin().getZ());
    	//Check if the resulting pose touches the table
    	if (check_visible && (it->getOrigin().getZ() - object_height/2 - tableHeight_ <= 0.02) &&
    			(it->getOrigin().getZ() - object_height/2 - tableHeight_ > 0)) {
    		//TODO If the object is too big to grasp, sample only in neighborhood & use push primitive
    		//Note that push can only be to the right or left
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
        	//This coordinate can be seen => Not hidden
            return false;
        else if (node && check_visible)
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

		//int user_input;
		//std::cout << "Save the screen shot for new pose. Press 1 and then 'Enter' after you are done." << std::endl;
		//std::cin >> user_input;
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
	grid_locations_ = grid_locations;
}

bool Planner::inFront(pcl::PointCloud<PointT> cloud, int cluster_idx)
{
	int min_row = grid_locations_[cluster_idx][0];
	int min_col = grid_locations_[cluster_idx][1];
	int max_row = grid_locations_[cluster_idx][2];
	int max_col = grid_locations_[cluster_idx][3];

	if (min_row == 0 && max_row == 0)
		return true;
	else {
		//Check if this col has any other object in the same col but lesser row
		for (unsigned int i = 0; i < grid_locations_.size(); i++)
		{
			vector<int> temp_vec = grid_locations_[i];
			if (temp_vec[1] == min_col && temp_vec[0] < min_row)
				return false;
			if (temp_vec[1] == max_col && temp_vec[0] < max_row)
				return false;
			if (temp_vec[3] == max_col && temp_vec[2] < max_row)
				return false;
			if (temp_vec[3] == min_col && temp_vec[2] < min_row)
				return false;
		}
	}
	//ROS_ERROR("inFront: Should not be here.");
	ROS_INFO("Cluster %d: Exiting inFront", cluster_idx);
	return true;
}

void Planner::findMovable(vector<sensor_msgs::PointCloud2> config,
		vector<sensor_msgs::PointCloud2>& movable_clusters, vector<int>& movable_idx)
{
	//Find which clusters have full frontal visibility
	for (size_t i = 0; i < config.size(); i++)
	{
		pcl::PointCloud<PointT> cloud;
		fromROSMsg(config[i], cloud);

		//Get bounding box of target cloud
		//ROS_DEBUG("Finding bounding box of cloud");
		//object_manipulation_msgs::ClusterBoundingBox bbx;
		//getClusterBoundingBox(config[i], bbx.pose_stamped, bbx.dimensions);

		if (touchesTable(cloud, tableHeight_) && (cloud.points.size() >= 200)) {
			//This cluster is in contact with the table
			ROS_INFO("Cluster %zu touches the table", i);
			if (inFront(cloud, (int)i))	{
				ROS_INFO("Cluster %zu is in front and fully visible", i);
				movable_clusters.push_back(config[i]);
				movable_idx.push_back((int)i);
			}
		}
	}
	movablePub_.publish(concatClouds(movable_clusters));
	ROS_DEBUG("Found %zu movable clusters", movable_clusters.size());
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
			if (checkHiddenOrVisible(targetCloud2_, concatCloudTTO, *it, identity, true, false))
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
								 vector<sensor_msgs::PointCloud2> new_config)
{
	ROS_DEBUG("Generating percentage");

	//create tabletop representation for the new config
	tf::Transform identity;
	identity.setIdentity();

	size_t num_remaining = 0;
	sensor_msgs::PointCloud2 otherCloud2 = concatClouds(new_config);
	TableTopObject otherCloudTTO = createTTO(otherCloud2);
	for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it != object_belief.end(); it++) {
		if (checkHiddenOrVisible(targetCloud2_, otherCloudTTO, *it, identity, true, false))
			num_remaining++;
	}

	double this_percentage = (object_belief.size() == 0 ? 1 :
			(object_belief.size() - num_remaining)/ (double) object_belief.size());

	ROS_DEBUG("This move would reveal %zu of remaining hypotheses that is %f percent",
			object_belief.size() - num_remaining, 100 * this_percentage);

	return this_percentage;
}

void Planner::findPossibleMoves(sensor_msgs::PointCloud2& other_cloud,
		vector<sensor_msgs::PointCloud2> movable_clusters,
		vector<int> movable_idx,
		vector<Move>& possible_moves)
{
	ROS_DEBUG("Finding possible moves");
	//Given current configuration and visible objects, find all valid moves of these objects.
	TableTopObject concatCloudTTO = createTTO(other_cloud);

	for (size_t cluster_idx = 0; cluster_idx < movable_clusters.size(); cluster_idx++) {
	//for (size_t cluster_idx = 0; cluster_idx < 3; cluster_idx++) {

		//Finding source pose of this cluster
		pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
		fromROSMsg(movable_clusters[cluster_idx], *clusterCloud);
		tf::Vector3 min, max;
		minmax3d(min, max, clusterCloud);
		tf::Vector3 centroid(min.x() + (max.x() - min.x())/2.0, min.y() + (max.y() - min.y())/2.0, min.z() + (max.z() - min.z())/2.0);
		tf::Pose source_pose;
		source_pose.setOrigin(centroid);
		source_pose.setRotation(tf::Quaternion(0,0,0,1));

		//Find destination poses
		vector<tf::Pose> destination_poses;
		ROS_DEBUG("Finding destination poses for visible cluster %zu", cluster_idx);
		samplePose(movable_clusters[cluster_idx], concatCloudTTO, destination_poses, false, true);

		//Create moves for this object
		//vector<Move> moves;
		//Move temp_move(visible_idx[cluster_idx], visible_clusters[cluster_idx], source_pose, destination_poses[0]);
		//moves.resize(destination_poses.size(), temp_move);
		ROS_DEBUG("1. size of free destination poses for object %d = %zu",
				movable_idx[cluster_idx], destination_poses.size());

		//int j = 0;
		for (std::vector<tf::Pose>::iterator it = destination_poses.begin(); it!=destination_poses.end(); it++)
		{
			Move this_move(movable_idx[cluster_idx], movable_clusters[cluster_idx], source_pose, *it);
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
	//TODO: Include rotation
	//ROS_DEBUG("Inside simulateMove");
	new_config = config;

	//Find what the new point cloud would be after the cluster has been moved
	sensor_msgs::PointCloud2 moved_cloud2 = move.objectToMove;
	pcl::PointCloud<PointT>::Ptr moved_cloud(new pcl::PointCloud<PointT>);
	fromROSMsg(moved_cloud2, *moved_cloud);

	//Temp
	//source_pose_pub_.publish(tfPoseToGeometryPose(move.sourcePose));
	//dest_pose_pub_.publish(tfPoseToGeometryPose(move.destPose));

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
/*
		tf::Vector3 orig(moved_cloud->points[i].x, moved_cloud->points[i].y, moved_cloud->points[i].z);
		tf::Vector3 moved = orig.rotate(rot.getAxis(), rot.getAngle());	//rot*orig;
		moved_cloud->points[i].x = moved.getX();
		moved_cloud->points[i].y = moved.getY();
		moved_cloud->points[i].z = moved.getZ();
	*/
	}

	//Replace the point cloud for the cluster being moved with the point cloud at the destination
	toROSMsg(*moved_cloud, new_config[move.cluster_idx]);
}

void Planner::plan(int horizon,
		vector<sensor_msgs::PointCloud2> config,
		sensor_msgs::PointCloud2 other_cloud,
		vector<Move>& action_sequence_so_far,
		double& total_percentage_revealed_so_far)
{
	//Config: All clusters, big and small
	//other_cloud: The whole object cloud with all clusters, big and small
	//visible_clusters: Clusters that are big enough to be manipulated, that are in contact with the table, and in front

	ROS_DEBUG("Plan: horizon %d", horizon);

	//Sample target pose
	ROS_DEBUG("Sampling target object pose in occluded space");
	std::vector<tf::Pose> object_posterior_belief;
	samplePose(targetCloud2_, createTTO(other_cloud), object_posterior_belief, true, false);

	make_grid(config);
	if (horizon == MAX_HORIZON)
		display_grid();
	findGridLocations(config);

	//Find movable clusters
	ROS_DEBUG("Finding movable clusters");
	vector<sensor_msgs::PointCloud2> movable_clusters;
	vector<int> movable_idx;
	findMovable(config, movable_clusters, movable_idx);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu movable clusters", horizon, movable_clusters.size());

	//Find % revealed by each visible object
	//map<int, double> percentage;
	//generatePercentageIfRemoved(object_posterior_belief, movable_clusters, movable_idx, percentage);

	//Find possible moves
	vector<Move> possible_moves;
	findPossibleMoves(other_cloud, movable_clusters, movable_idx, possible_moves);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu possible moves", horizon, possible_moves.size());

	double info_gain = -1.0;
	vector<Move> best_action_sequence;
	Move best_action = possible_moves[0];

	//For each possible move, do the following.

	for (size_t move_idx = 0; move_idx < possible_moves.size(); move_idx++) {
	//for (size_t move_idx = 0; move_idx < 2; move_idx++) {
		ROS_DEBUG("Horizon %d: Simulating move %zu", horizon, move_idx);
		double percentage_revealed_so_far = total_percentage_revealed_so_far;
		Move this_move = possible_moves[move_idx];
		vector<Move> action_sequence = action_sequence_so_far;
		action_sequence.push_back(this_move);

		//Simulate move & find resulting config
		pcl::PointCloud<PointT>::Ptr config_cloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(other_cloud, *config_cloud);
		ROS_DEBUG("Publishing current simulated config cloud on topic current_simulated_config");
		pubCloud("current_simulated_config", config_cloud, fixed_frame_);
		vector<sensor_msgs::PointCloud2> new_config;
		simulateMove(config, this_move, new_config);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);
		
		breakpoint();

		//Find percentage revealed by this move
		double this_percentage_revealed = generatePercentageIfDisplaced(object_posterior_belief, new_config);
		//double this_percentage_revealed = percentage[this_move.cluster_idx];
		percentage_revealed_so_far += this_percentage_revealed;	// + percentage_revealed_so_far;
		
		if (horizon > 1) {
			//Plan recursively
			plan(horizon-1, new_config, concatClouds(new_config), action_sequence, percentage_revealed_so_far);
		} //else {
			//best_next_action_sequence.clear();
		//}

		if (percentage_revealed_so_far > info_gain) {
			//This action reveals more than previously tested actions
			info_gain = percentage_revealed_so_far;
			best_action_sequence = action_sequence;
		}
	}
	
	total_percentage_revealed_so_far = info_gain;
	action_sequence_so_far = best_action_sequence;
	//best_next_action_sequence.insert(best_next_action_sequence.begin(), best_action);
	ROS_DEBUG("Found the best next action sequence for horizon %d of length %zu", horizon, action_sequence_so_far.size());
}

/*
void Planner::random_plan(int horizon,
		vector<sensor_msgs::PointCloud2> config,
		sensor_msgs::PointCloud2 other_cloud,
		vector<Move>& action_sequence_so_far,
		double& total_percentage_revealed_so_far)
{
	//Config: All clusters, big and small
	//other_cloud: The whole object cloud with all clusters, big and small
	//visible_clusters: Clusters that are big enough to be manipulated, that are in contact with the table, and in front

	ROS_DEBUG("Plan: horizon %d", horizon);

	make_grid(config);
	findGridLocations(config);

	//Find movable clusters
	ROS_DEBUG("Finding movable clusters");
	vector<sensor_msgs::PointCloud2> movable_clusters;
	vector<int> movable_idx;
	findMovable(config, movable_clusters, movable_idx);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu movable clusters", horizon, movable_clusters.size());

	//Find possible moves
	vector<Move> possible_moves;
	findPossibleMoves(other_cloud, movable_clusters, movable_idx, possible_moves);
	if (horizon == MAX_HORIZON)
		ROS_INFO("Horizon %d: Found %zu possible moves", horizon, possible_moves.size());

	double info_gain = -1.0;
	vector<Move> best_action_sequence;
	Move best_action = possible_moves[0];

	//For each possible move, do the following.

	for (size_t move_idx = 0; move_idx < possible_moves.size(); move_idx++) {
	//for (size_t move_idx = 0; move_idx < 2; move_idx++) {
		ROS_DEBUG("Horizon %d: Simulating move %zu", horizon, move_idx);
		double percentage_revealed_so_far = total_percentage_revealed_so_far;
		Move this_move = possible_moves[move_idx];
		vector<Move> action_sequence = action_sequence_so_far;
		action_sequence.push_back(this_move);

		//Simulate move & find resulting config
		pcl::PointCloud<PointT>::Ptr config_cloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr new_config_cloud(new pcl::PointCloud<PointT>);
		fromROSMsg(other_cloud, *config_cloud);
		ROS_DEBUG("Publishing current simulated config cloud on topic current_simulated_config");
		pubCloud("current_simulated_config", config_cloud, fixed_frame_);
		vector<sensor_msgs::PointCloud2> new_config;
		simulateMove(config, this_move, new_config);
		fromROSMsg(concatClouds(new_config), *new_config_cloud);
		//ROS_DEBUG("Publishing new simulated config on topic new_simulated_config");
		pubCloud("new_simulated_config", new_config_cloud, fixed_frame_);

		//Find percentage revealed by this move
		double this_percentage_revealed = generatePercentageIfDisplaced(object_posterior_belief, new_config);
		//double this_percentage_revealed = percentage[this_move.cluster_idx];
		percentage_revealed_so_far += this_percentage_revealed;	// + percentage_revealed_so_far;

		if (horizon > 1) {
			//Plan recursively
			plan(horizon-1, new_config, concatClouds(new_config), action_sequence, percentage_revealed_so_far);
		} //else {
			//best_next_action_sequence.clear();
		//}

		if (percentage_revealed_so_far > info_gain) {
			//This action reveals more than previously tested actions
			info_gain = percentage_revealed_so_far;
			best_action_sequence = action_sequence;
		}
	}

	total_percentage_revealed_so_far = info_gain;
	action_sequence_so_far = best_action_sequence;
	//best_next_action_sequence.insert(best_next_action_sequence.begin(), best_action);
	ROS_DEBUG("Found the best next action sequence for horizon %d of length %zu", horizon, action_sequence_so_far.size());
}
*/

void Planner::execute_plan()
{
	tum_os::Execute_Plan execute_call;
	for (size_t i = 0; i < action_sequence_.size(); i++)
	{
		execute_call.request.cluster_idx.push_back(action_sequence_[i].cluster_idx);
		execute_call.request.object_to_move.push_back(action_sequence_[i].objectToMove);
		execute_call.request.source_pose.push_back(tfPoseToGeometryPose(action_sequence_[i].sourcePose));
		execute_call.request.dest_pose.push_back(tfPoseToGeometryPose(action_sequence_[i].destPose));
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

int main (int argc, char** argv)
{
	if (argc != 2) {
		ROS_ERROR("Insufficient number of arguments. Give horizon length as argument.");
		return -1;
	}
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
