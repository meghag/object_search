#include "planner.h"

//void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id = fixed_frame_);

bool touchesTable(pcl::PointCloud<PointXYZRGB> cloud, double table_height)
{
	ROS_INFO("Inside touchesTable");
	unsigned int count = 0;
	//for (PointCloud<PointXYZRGB>::iterator it = cloud.points.begin(); it != cloud.points.end(); it++)
	for(vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it = cloud.points.begin(); it != cloud.points.end(); ++it)
	{
		if (abs(it->z - table_height) < 0.01)
			//Close enough to table
			count++;
	}

	if (count > 50)	//Should be a % of cloud size instead of fixed at 50
	{
		ROS_INFO("Yes, it does!");
		return true;
	}
	else {
		ROS_INFO("No, it doesn't!");
		return false;
	}
}

/*
bool convexPointCloud(pcl::PointCloud<PointXYZRGB> cloud)
{
	//TODO

	ROS_INFO("Inside convexPointCloud");

	//First project the cloud on vertical plane?

	vector<double> vecx, vecy, vecz;
	for (vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it1 = cloud.points.begin(); it1 != cloud.points.end(); ++it1) {
		vecx.push_back(it1->x);
		vecy.push_back(it1->y);
		vecz.push_back(it1->z);
	}
	vector<double>::iterator it_minx, it_miny, it_minz, it_maxx, it_maxy, it_maxz;
	it_minz = min_element(vecz.begin(), vecz.end());
	it_maxz = max_element(vecz.begin(), vecz.end());

	return true;
}
*/

void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( *cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

bool inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax)
{
    if ((point.x() > bbmin.x()) && (point.x() < bbmax.x()) &&
		(point.y() > bbmin.y()) && (point.y() < bbmax.y()) &&
		(point.z() > bbmin.z()) && (point.z() < bbmax.z()))
        return true;
    else
        return false;
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
    ret.setRotation(tf::Quaternion(q[0],q[1],q[2],q[3]));
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

Planner::Planner (ros::NodeHandle& n): n_(n)
{
	//planActionsServer_ = n_.advertiseService("plan_actions", &DataProcessor::planActionsCallback, this);
	ROS_INFO("Ready to plan actions.");

	planRequestSub_ = n_.subscribe("plan_request", 1, &Planner::planRequestCallback, this);
	objectCloudPub_ = n_.advertise<sensor_msgs::PointCloud2>("object_cloud",1);
	clustersPub_ = n_.advertise<sensor_msgs::PointCloud2>("one_cluster",1);

	//generate object we search as a pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	ROS_INFO("Filling the point cloud for my object");
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
		pcl::PointXYZRGB pt;
		pt.x = act.getOrigin().x();
		pt.y = act.getOrigin().y();
		pt.z = act.getOrigin().z() * 4;

		object_cloud->points.push_back(pt);
	}
	toROSMsg(*object_cloud, targetCloud2_);
	ROS_INFO("MG: Publishing my object cloud on topic my_object");
	pubCloud("my_object", object_cloud, fixed_frame_);

	int user_input;
	std::cout << "Save the screen shot for my_object. Press 1 and then 'Enter' after you are done." << std::endl;
	std::cin >> user_input;

	base_to_camera_ = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str(),ros::Time(0));

	//octreePub_ = n.advertise<visualization_msgs::MarkerArray>("current_octree_array",100);
	//raycastPub_ = n.advertise<visualization_msgs::MarkerArray>("raycast_array",200);
	//visibleObjectsSub_ = n_.subscribe("visible_objects", 1, &Planner::visibleObjectsCallback, this);
	//octreeSub_ = n_.subscribe("octree",1, &Planner::octreeCallback, this);
	//tree_ = tree(treeResolution_);
}

Planner::~Planner (void) {}

void Planner::pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id)
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

    //ROS_INFO("published frame %s %i x %i points on %s", out.header.frame_id.c_str(), out.height, out.width, topic_name.c_str());
}

void Planner::pub_belief(const std::string &topic_name,const std::vector<tf::Pose> poses)
{
	ROS_INFO("MG: Inside pub_belief");
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
    ROS_INFO("MG: Exiting pub_belief");
}

tf::Stamped<tf::Pose> Planner::getPose(const std::string target_frame,const std::string lookup_frame, ros::Time tm = ros::Time(0))
{
	ROS_INFO("MG: Inside get Pose: Returns a transform between two frames as a TF Stamped Pose");
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

    ROS_INFO("MG: Exiting get Pose");
    return ret;
}


void Planner::planRequestCallback(const tum_os::PlanRequest::ConstPtr& plan_request)
{
	//clustersDetected_ = plan_request.clusters;
	//targetCloud2_ = plan_request.target_cloud;
	objectCloud2_ = plan_request->object_cloud;
	tableHeight_ = plan_request->table_height;
	ROS_INFO("table height = %f", tableHeight_);
	
	//Euclidean segmentation
	objectCloudPub_.publish(objectCloud2_);
	tum_os::Clusters clusters_msg;
	//vector<sensor_msgs::PointCloud2> clusters;
	ROS_INFO("MG: Waiting for message on topic clusters");
	clusters_msg  = *(ros::topic::waitForMessage<tum_os::Clusters>("/clusters"));
	clustersDetected_ = clusters_msg.clusters;
	ROS_INFO("Found %zu clusters", clustersDetected_.size());

	int user_input;
	for (size_t i = 0; i < clustersDetected_.size(); i++)
	{
		clustersDetected_[i].header.frame_id = "base_link";
		clustersPub_.publish(clustersDetected_[i]);
		std::cout << "Save the screen shot for cluster. Press 1 and then 'Enter' after you are done." << std::endl;
		std::cin >> user_input;
	}

	//TODO: Need to think how a config should be passed
	//as a vector of point clouds, a TTO, a single point cloud??
	vector<Move> best_next_action_sequence;
	double total_percentage_revealed_so_far = 0.0;
	plan(MAX_HORIZON, clustersDetected_, best_next_action_sequence, total_percentage_revealed_so_far);
}

void Planner::samplePose(sensor_msgs::PointCloud2 target_cloud2, 
						 sensor_msgs::PointCloud2 other_cloud2,
						 vector<tf::Pose>& object_posterior_belief,
						 bool check_hidden,
						 bool check_visible)
{
	//This function samples the whole space for valid poses of the object_cloud.
	//Then it checks whether they lie in the free space and returns an object belief.
	if (check_hidden && !check_visible)
		ROS_INFO("Sampling in occluded space");
	else if (!check_hidden && check_visible)
		ROS_INFO("Sampling in free space");
	
	ROS_INFO("before creating samples");
    std::vector<tf::Pose> object_belief;
    for (int k =0; k < 100000; k++)
    {
        object_belief.push_back(vdc_pose_bound(BB_MIN,BB_MAX,k));
    }
    ROS_INFO("samples created");
	
    pub_belief("object_belief", object_belief);
    tf::Transform identity;
    identity.setIdentity();
    for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it!=object_belief.end(); it++)
    {
    	//First check if the pose is upright
    	//Stupid way of checking if the rotation matrix is identity
    	if ((it->getBasis())[0][0] == 1 && (it->getBasis())[0][1] == 0 && (it->getBasis())[0][2] == 0 &&
    			(it->getBasis())[1][0] == 0 && (it->getBasis())[1][1] == 1 && (it->getBasis())[1][2] == 0 &&
    			(it->getBasis())[2][0] == 0 && (it->getBasis())[2][1] == 0 && (it->getBasis())[2][2] == 1)
    	{
    		//Rotation matrix is an identity matrix
    		ROS_INFO("Pose is upright");
    		//Now check if the pose is hidden or not
    		if (checkHiddenOrVisible(target_cloud2, other_cloud2, *it, identity, check_hidden, check_visible))
    			object_posterior_belief.push_back(*it);
    	}
    }
	
    ROS_INFO("samples checked");
    std::cout << "size of object belief " << object_posterior_belief.size() << std::endl;
    pub_belief("valid_pose_samples",object_posterior_belief);

    int user_input;
  	std::cout << "Save the screen shot for sampled poses. Press 1 and then 'Enter' after you are done." << std::endl;
  	std::cin >> user_input;
}

bool Planner::checkHiddenOrVisible(sensor_msgs::PointCloud2 object_cloud2,
						  sensor_msgs::PointCloud2 other_cloud2, 
						  tf::Transform ownTransform, 
						  tf::Transform otherTransform,
						  bool check_hidden,
						  bool check_visible)
{
	ROS_INFO("Inside checkHiddenOrVisible");
	//This function checks if the object_cloud is hidden by the other_cloud.
	//Returns true if hidden, false if in free space.
	tf::Transform resultingTransform = otherTransform.inverseTimes(ownTransform);
	geometry_msgs::Transform trans;
	
	pcl::PointCloud<PointXYZRGB> cloud;
	fromROSMsg(object_cloud2, cloud);
	
	pcl::PointCloud<PointXYZRGB>::Ptr otherCloud(new pcl::PointCloud<PointXYZRGB>);
	fromROSMsg(other_cloud2, *otherCloud);
	
	ROS_INFO("Creating a TableTopObject (TTO) for other_cloud");
    TableTopObject otherCloudTTO(base_to_camera_.getOrigin(), (float)BB_MIN.z(), *otherCloud);
	
    unsigned int num_visible = 0;
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        tf::Vector3 vec(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        vec = resultingTransform * vec;
        octomath::Vector3 coord;
		
        coord.x() = vec.x();
        coord.y() = vec.y();
        coord.z() = vec.z();
		
        octomap::OcTreeKey key;
		
        // this can happen when we have an empty octree, which would of course not cover the object, so return false
        if (!otherCloudTTO.m_octoMap->octree.genKey(coord, key))
            return false;
		
        octomap::OcTreeNode *node = otherCloudTTO.m_octoMap->octree.search(key);
		
        if (!node && check_hidden)
        	//This coordinate can be seen => Not hidden
            return false;
        else if (!node && check_visible)
        	num_visible++;
        //if (node && (!otherObject.m_octoMap->octree.isNodeOccupied(node)))
        //  return false;
    }

    if (check_hidden)
    	//All coordinates of the point cloud were hidden
    	return true;
    else if (check_visible && num_visible == cloud.points.size())
    	//All coordinates of the point cloud were visible i.e. in free space
    	return true;
    else
    	return false;
}

bool Planner::generatePercentage(vector<tf::Pose> object_belief,
								 vector<sensor_msgs::PointCloud2> clusters,
								 vector<double>& percentage)
{
	ROS_INFO("Generating percentage");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr percentage_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//create tabletop representation with one cluster missing at a time
	//int max_idx = -1;
	//double max_perc = 0;
	tf::Transform identity;
	identity.setIdentity();
	percentage.resize(clusters.size());
	pcl::PointCloud<PointXYZRGB> temp_cloud;
	for (size_t i = 0; i < clusters.size(); i++) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (size_t j = 0; j < clusters.size(); j++) {
			fromROSMsg(clusters[j], temp_cloud);
			if (i == 0 && j == 1)
				*concat_cloud = temp_cloud;
			else if (i != 0 && j == 0)
				*concat_cloud = temp_cloud;
			else if (j != i) {
				*concat_cloud += temp_cloud;
			}
		}
		ROS_INFO("MG: Publishing the cloud without cluster %zu on topic sans_cluster_TTOcloud", i);
		pubCloud("sans_cluster_TTOcloud", concat_cloud, fixed_frame_);
		int user_input;
		std::cout << "Save the screen shot for sans_cluster_TTOCloud. Press 1 and then 'Enter' after you are done." << std::endl;
		std::cin >> user_input;

		size_t num_remaining = 0;
		sensor_msgs::PointCloud2 concat_cloud2;
		toROSMsg(*concat_cloud, concat_cloud2);
		for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it != object_belief.end(); it++) {
			if (checkHiddenOrVisible(targetCloud2_, concat_cloud2, *it, identity, true, false))
				num_remaining++;
		}
		
		double this_percentage = (object_belief.size() == 0 ? 1 :
							 (object_belief.size() - num_remaining)/ (double) object_belief.size());
		
		std::cout << "Removing Cluster " << i << " would reveal "
		<< object_belief.size() - num_remaining
		<< " of remaining hypotheses " << " that is "
		<< 100 * this_percentage << "%" << std::endl;
		
		percentage[i] = this_percentage;
	}
	return true;
}	

void Planner::findPossibleMoves(vector<sensor_msgs::PointCloud2> config,
		vector<sensor_msgs::PointCloud2> visible_clusters,
		vector<Move>& possible_moves)
{
	ROS_INFO("Finding possible moves");
	//Given current configuration and visible objects, find all valid moves of these objects.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	sensor_msgs::PointCloud2 concat_cloud2;
	fromROSMsg(config[0], *concat_cloud);
	for (size_t i = 1; i < config.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		fromROSMsg(config[i], temp_cloud);
		*concat_cloud += temp_cloud;
	}
	toROSMsg(*concat_cloud, concat_cloud2);
	ROS_INFO("Publishing current simulated config cloud on topic current_simulated_config");
	pubCloud("current_simulated_config", concat_cloud, fixed_frame_);

	for (size_t cluster_idx = 0; cluster_idx < visible_clusters.size(); cluster_idx++) {
		//Find destination poses
		vector<tf::Pose> destination_poses;
		samplePose(visible_clusters[cluster_idx], concat_cloud2, destination_poses, false, true);

		for (std::vector<tf::Pose>::iterator it = destination_poses.begin(); it!=destination_poses.end(); it++)
		{
			Move this_move(cluster_idx, visible_clusters[cluster_idx], *it);
			possible_moves.push_back(this_move);
		}
		//ROS_INFO("Samples checked for object %zu", cluster_idx);
		std::cout << "size of free destination poses for object "
				<< cluster_idx << " = "
				<< destination_poses.size() << std::endl;
	}
		/*
		//Step 1: Sample whole space for poses of the visible object
		//sample only upright poses
		std::vector<tf::Pose> dest_poses;
		std::vector<tf::Pose> object_belief;
		for (int k =0; k < 100000; k++)
		{
			dest_poses.push_back(vdc_pose_bound(BB_MIN,BB_MAX,k));
		}
		ROS_INFO("Samples created for object %zu", cluster_idx);

		//Step 2: Discard poses that lie in currently occupied regions
		std::vector<tf::Pose> free_dest_poses;
		ROS_INFO("MG: Creating a TableTopObject (TTO) for object %zu", cluster_idx);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		fromROSMsg(visible_clusters[cluster_idx], *cluster_cloud);
		TableTopObject visibleObject(base_to_camera_.getOrigin(), BB_MIN.z(), cluster_cloud);

		tf::Transform identity;
		identity.setIdentity();

		pub_belief("object_posterior_belief",object_posterior_belief);
		*/
}

void Planner::findVisible(vector<sensor_msgs::PointCloud2> config,
		vector<sensor_msgs::PointCloud2>& visible_clusters)
{
	ROS_INFO("Inside findVisible");
	//Find which clusters have full frontal visibility
	for (size_t i = 0; i < config.size(); i++)
	{
		pcl::PointCloud<PointXYZRGB> cloud;
		fromROSMsg(config[i], cloud);

		if (touchesTable(cloud, tableHeight_)) {
			//This cluster is in contact with the table
			ROS_INFO("Cluster %zu touches the table", i);
			//if (convexPointCloud(cloud))
				//Width is consistent throughout the height. So fully visible.
				//ROS_INFO("Cluster %zu is fully visible", i);
				visible_clusters.push_back(config[i]);
		}
	}
}

void Planner::simulateMove(vector<sensor_msgs::PointCloud2> config,
		Move move, vector<sensor_msgs::PointCloud2>& new_config)
{
	//TODO: Include rotation
	ROS_INFO("Inside simulateMove");
	new_config = config;

	//Find what the new point cloud would be after the cluster has been moved
	sensor_msgs::PointCloud2 moved_cloud2 = move.objectToMove;
	pcl::PointCloud<PointXYZRGB>::Ptr moved_cloud(new pcl::PointCloud<PointXYZRGB>);
	fromROSMsg(moved_cloud2, *moved_cloud);

	//Assuming only translation
	double translation_x = move.destPose.getOrigin().getX() - move.sourcePose.getOrigin().getX();
	double translation_y = move.destPose.getOrigin().getY() - move.sourcePose.getOrigin().getY();
	double translation_z = move.destPose.getOrigin().getZ() - move.sourcePose.getOrigin().getZ();

	for (size_t i = 0; i < moved_cloud->points.size(); i++)
	{
		moved_cloud->points[i].x += translation_x;
		moved_cloud->points[i].y += translation_y;
		moved_cloud->points[i].z += translation_z;
	}
	ROS_INFO("Publishing on topic new_simulated_config");
	pubCloud("new_simulated_config", moved_cloud, fixed_frame_);

	//Replace the point cloud for the cluster being moved with the point cloud at the destination
	toROSMsg(*moved_cloud, new_config[move.cluster_idx]);
}

void Planner::plan(int horizon,
		vector<sensor_msgs::PointCloud2> config,
		vector<Move>& best_next_action_sequence,
		double& total_percentage_revealed_so_far)
{
	//Sample target pose
	ROS_INFO("Sampling target object pose in ocluded space");
	std::vector<tf::Pose> object_posterior_belief;
	samplePose(targetCloud2_, objectCloud2_, object_posterior_belief, true, false);

	//Find visible clusters
	ROS_INFO("Finding visible clusters");
	vector<sensor_msgs::PointCloud2> visible_clusters;
	if (horizon == MAX_HORIZON)
		visible_clusters = clustersDetected_;
	else
		findVisible(config, visible_clusters);
	ROS_INFO("Found %zu visible clusters", visible_clusters.size());

	//Find % revealed by each visible object
	vector<double> percentage;
	generatePercentage(object_posterior_belief, config, percentage);

	//Find possible moves
	vector<Move> possible_moves;
	findPossibleMoves(config, visible_clusters, possible_moves);

	//std::map<std::pair<int, float> > hypothesis_revealed_map;

	double info_gain = -1.0;
	vector<Move> best_action_sequence;

	//For each possible move, do the following.
	for (size_t move_idx = 0; move_idx < possible_moves.size(); move_idx++) {
		Move this_move = possible_moves[move_idx];

		//Simulate move & find resulting config
		vector<sensor_msgs::PointCloud2> new_config;
		simulateMove(config, this_move, new_config);
		
		//Find percentage revealed by this move
		double this_percentage_revealed = percentage[this_move.cluster_idx];
		double total_percentage_revealed = this_percentage_revealed + total_percentage_revealed_so_far;
		
		if (horizon > 1) {
			//Plan recursively
			plan(horizon-1, new_config, best_next_action_sequence, total_percentage_revealed);
		} else {
			best_next_action_sequence.clear();
		}

		if (total_percentage_revealed > info_gain) {
			//This action reveals more than previously tested actions
			info_gain = total_percentage_revealed;
			best_action_sequence.insert(best_action_sequence.begin(), this_move);
		}
	}
	
	best_next_action_sequence = best_action_sequence;
	ROS_INFO("Found the best next action sequence");
	/*
	if (horizon == MAX_HORIZON)
	{
		for (size_t i = 0; i < best_next_action_sequence.size(); i++)
		{
			ROS_INFO(" ")
		}
	}
	*/

	//std::map<std::pair<int, std::vector<tf::Pose> > > cluster_to_destination_poses_map;
	//if (horizon == 0)
		//hypothesis_revealed_map.insert(std::make_pair(cluster_idx,0.0));
	//else {
		//float p = hypothesis_revealed_map[cluster_idx].second() + percentage;
	//}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle n;
	//octomap::OcTree tree(0.05);
	Planner planner(n);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //tf::Stamped<tf::Pose> fixed_to_ik;
    //tf::Stamped<tf::Pose> base_to_camera;
	
    //getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1));
	//fixed_to_ik = getPose(fixed_frame_, ik_frame_);
	//base_to_camera = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());
		
	ros::spin();
	return 0;
}


/*
void Planner::octreeCallback (const object_search_pkg::octree::ConstPtr& tree)
{
	tree_ = tree;
}

 bool Planner::generatePercentage(vector<tf::Pose> object_belief,
								 vector<sensor_msgs::PointCloud2> clusters,
								 vector<double>& percentage)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr percentage_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//create tabletop representation with one cluster missing at a time
	//int max_idx = -1;
	//double max_perc = 0;
	tf::Transform identity;
	identity.setIdentity();
	std::vector<TableTopObject*> obj_excluding;
	percentage.resize(clusters.size());
	pcl::PointCloud<PointXYZRGB> temp_cloud;
	for (size_t i = 0; i < clusters.size(); i++) {
		TableTopObject *act = new TableTopObject();
		pcl::PointCloud<pcl::PointXYZRGB> concat_cloud;

		for (size_t j = 0; j < clusters.size(); j++) {
			fromROSMsg(clusters[j], temp_cloud);
			if (i == 0 && j == 1)
				concat_cloud = temp_cloud;
			else if (i != 0 && j == 0)
				concat_cloud = temp_cloud;
			else if (j != i) {
				concat_cloud += temp_cloud;
				act->addPointCloud2(base_to_camera_.getOrigin(), BB_MIN.z(),temp_cloud);
			}
		}
		obj_excluding.push_back(act);
		size_t num_remaining = 0;
		sensor_msgs::PointCloud2 concat_cloud2;
		toROSMsg(concat_cloud, concat_cloud2);
		for (std::vector<tf::Pose>::iterator it = object_belief.begin(); it != object_belief.end(); it++) {
			if (checkHidden(targetCloud2_, concat_cloud2, *it, identity))
				num_remaining++;
		}

		double this_percentage = (object_belief.size() == 0 ? 1 :
							 (object_belief.size() - num_remaining)/ (double) object_belief.size());

		std::cout << "Removing Cluster " << i << " would reveal "
		<< object_belief.size() - num_remaining
		<< " of remaining hypotheses " << " that is "
		<< 100 * this_percentage << "%" << std::endl;

		percentage[i] = this_percentage;
	}
	return true;
}

void Planner::visibleObjectsCallback(const object_search_pkg::objLocationMap::ConstPtr& now_visible)
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
