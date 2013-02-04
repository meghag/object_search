#include "manipulator.h"

Manipulator::Manipulator(ros::NodeHandle & n): n_(n)
{
	//manipulate_service_called_ = false;
	manipulate_service_ = n_.advertiseService("execute_plan", &Manipulator::callback, this);
	marker_pub_ = n_.advertise<visualization_msgs::Marker>("manipulator_marker", 10);
	target_pub_ = n_.advertise<sensor_msgs::PointCloud2>("manipulation_target",1);
	bbx_pub_ = n_.advertise<visualization_msgs::Marker>("target_bbx",1);
	//target_object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("target_object",1);
	collision_object_pub_ = n_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
	source_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("source_pose", 1);
	dest_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("dest_pose", 1);

	shape[0] = visualization_msgs::Marker::CUBE;
	shape[1] = visualization_msgs::Marker::SPHERE;
	shape[2] = visualization_msgs::Marker::CYLINDER;
	shape[3] = visualization_msgs::Marker::CUBE;

	reset_posn_.point.x = 0.68;	reset_posn_.point.y = -0.5; reset_posn_.point.z = 1;
	reset_posn_.header.frame_id = "base_link";

	LEFT_RESET.x = 0.68;	//0.3;
	LEFT_RESET.y = 0.5; //0.7
	LEFT_RESET.z = 1.0; //1.1
	RIGHT_RESET.x = 0.68;	//0.3;
	RIGHT_RESET.y = -0.5;  //-0.7
	RIGHT_RESET.z = 1.0;   //1.1

	RED.x = 0.4;  RED.y = -0.5; RED.z = 1.0;  //Red bin
	BLUE.x = 0.5;  BLUE.y = -0.5; BLUE.z = 1.0;	//Blue bin
	GREEN.x = 0.6; GREEN.y = -0.5; GREEN.z = 1.0;  //Green bin

	active_arm_ = "right_arm";
	active_arm_sym_ = 'r';
	active_reset_ = RIGHT_RESET;

	ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
	get_planning_scene_client_ = n_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);
	get_collision_objects_client_ = n_.serviceClient<arm_navigation_msgs::GetCollisionObjects>(GET_COLLISION_OBJECTS_NAME);
	bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
}

Manipulator::~Manipulator() {}

bool Manipulator::callback(tum_os::Execute_Plan::Request &req,
		tum_os::Execute_Plan::Response &res)
{
	ROS_INFO("Received request to manipulate.");
	cluster_idx_ = req.cluster_idx;
	object_to_move_ = req.object_to_move;
	source_pose_ = req.source_pose;
	dest_pose_ = req.dest_pose;
	table_height_ = req.table_height;

	ROS_INFO("Number of moves to execute = %zu", object_to_move_.size());

	//table_extent_ = req.table_extent;
	active_arm_ = "right_arm";
	active_arm_sym_ = 'r';
	active_reset_ = RIGHT_RESET;

	geometry_msgs::PoseStamped to_pub;
	to_pub.header.frame_id = "base_link";
	to_pub.header.stamp = ros::Time::now();
	for (size_t i = 0; i < object_to_move_.size(); i++)
	{
		object_to_move_[i].header.frame_id = "base_link";
		object_to_move_[i].header.stamp = ros::Time::now();
		target_cloud2_ = object_to_move_[i];

		// 1. Remove all collision objects
		ROS_INFO("Removing all collision objects.");
		resetCollisionModels();

		// 2. Get bounding box of target cloud
		ROS_INFO("Finding bounding box of cloud");
		getClusterBoundingBox(target_cloud2_, bbx_.pose_stamped, bbx_.dimensions);

		// 3. Add this bounding box as a collision object
		ROS_INFO("Adding a collision object for cloud");
		processCollisionGeometryForBoundingBox(bbx_, collision_name_);

		to_pub.pose = source_pose_[i];
		source_pose_pub_.publish(to_pub);
		to_pub.pose = dest_pose_[i];
		dest_pose_pub_.publish(to_pub);

		res.result = true;
		if (!pick_n_place(i))
			res.result = false;
		gripper_.open(active_arm_sym_);
		move_arm(active_reset_, 1, 0, false, 2, 1);
		//move_arm(reset_posn_.point, 1, 1, 2, 1);
	}
	return true;
}

vector<geometry_msgs::Point> Manipulator::waypoints(size_t idx)
{
	vector<geometry_msgs::Point> wp;
	geometry_msgs::Point center = bbx_.pose_stamped.pose.position;
	geometry_msgs::Point temp = center;
	geometry_msgs::Point temp2 = dest_pose_[idx].position;
	temp.z += 0.2;
	wp.push_back(temp);				//Above the object
	temp.z = center.z + bbx_.dimensions.z*0.5 - 0.01;
	wp.push_back(temp);				//Around the object
	temp.z += 0.1;
	wp.push_back(temp);				//Up in the air with the object
	temp2.z += 0.2;
	wp.push_back(temp2);			//Above the destination pose with the object in hand
	//temp2.z -= 0.01;
	temp2.z = dest_pose_[idx].position.z + bbx_.dimensions.z*0.5;
	wp.push_back(temp2);			//At the destination with hand around the object
	temp2.z += 0.1;
	wp.push_back(temp2);			//Above the destination pose

	return wp;
}

bool Manipulator::pick_n_place(size_t idx)
{
	//manipulate_service_called_ = true;
	target_pub_.publish(target_cloud2_);
	visualization_msgs::Marker bbx_marker = set_marker("base_link", "bbx", 1, visualization_msgs::Marker::CUBE,
			bbx_.pose_stamped.pose, bbx_.dimensions, 1.0f, 1.0f, 0.0f, 1.0);
	bbx_pub_.publish(bbx_marker);
	//bbx_pose_pub_.publish(bbx_);

	vector<geometry_msgs::Point> wp = waypoints(idx);		//Arm waypoints
	bool success = true;
	geometry_msgs::PoseStamped pose_pub;
	pose_pub.header.frame_id = "base_link";
	pose_pub.header.stamp = ros::Time::now();
	pose_pub.pose.orientation.x = 0.0;
	pose_pub.pose.orientation.y = 0.0;
	pose_pub.pose.orientation.z = 0.0;
	pose_pub.pose.orientation.w = 1.0;

	for (int arm = 0; arm < 2; arm++)
	{
		if (arm == 1) {
			ROS_INFO("Pick up with right arm failed. Trying with left arm.");
			active_arm_ = "left_arm";
			active_arm_sym_ = 'l';
			active_reset_ = LEFT_RESET;
			success = true;
		}

		gripper_.open(active_arm_sym_);
		//move_arm(geometry_msgs::Point go_to, int cluster_id, int collision_object_operation, bool attach, int plan_id, int action);
		for (size_t i = 0; i < wp.size(); i++)
		{
			pose_pub.pose.position = wp[i];
			dest_pose_pub_.publish(pose_pub);
			breakpoint();

			if (i == 0 && (move_arm(wp[i], 1, 0, false, 1, 0) == 0))
				breakpoint();
			else if (i == 1 && (move_arm(wp[i], 1, 0, false, 1, 0) == 0)) {
				breakpoint();
				gripper_.close(active_arm_sym_);
			} else if (i == 2 && (move_arm(wp[i], 1, 0, true, 1, 0) == 0)) {
				breakpoint();
			} else if (i == 3 && (move_arm(wp[i], 100, 0, false, 1, 0) == 0)) {
				breakpoint();
			} else if (i == 4 && (move_arm(wp[i], 1, 0, false, 2, 0) == 0)) {
				gripper_.open(active_arm_sym_);
				breakpoint();
			} else if (i > 4 && (move_arm(wp[i], 1, 0, false, 2, 0) == 0)) {
				breakpoint();
			} else {
				success= false;
				ROS_ERROR("Move failed");
				breakpoint();
				if (i > 0) {
					geometry_msgs::Point temp = wp[i-1];
					temp.z = table_height_ + bbx_.dimensions.z - 0.01;
					move_arm(temp, 1, 0, false, 2, 0);
					gripper_.open(active_arm_sym_);
					temp.z += 0.2;
					move_arm(temp, 1, 0, false, 2, 0);
					move_arm(active_reset_, 1, 0, false, 2, 1);
				}
				break;
			}
		}
		move_arm(active_reset_, 1, 1, false, 2, 1);
		/*
		if (success)
		{
			for (size_t i = wp.size() - 2; i < wp.size(); i++)
			{
				pose_pub.pose.position = wp[i];
				dest_pose_pub_.publish(pose_pub);
				breakpoint();

				if (move_arm(wp[i], 1, 0, 2, 0) != 0) {
					success= false;
					geometry_msgs::Point temp = wp[i];
					temp.z = table_height_ + bbx_.dimensions.z*0.5 + 0.03;
					move_arm(temp, 1, 0, 2, 0);
					gripper_.open();
					temp.z += 0.2;
					move_arm(temp, 1, 0, 2, 0);
					move_arm(active_reset_, 1, 0, 2, 1);
					break;
				}
				breakpoint();
			}
		}
		*/
	}

		/*
		if (move_arm(temp, 1, 0, 1, 0) == 0) {
			temp.z = center.z + bbx_.dimensions.z*0.5 - 0.01;
			if (move_arm(temp, 1, 0, 1, 0) == 0) {	//Around the object
				gripper_.close(active_arm_sym_);
				temp.z += 0.1;
				if (move_arm(temp, 1, 0, 1, 0) == 0) {	//Up in the air with the object
					temp2.z += 0.2;
					if (move_arm(temp2, 1, 0, 1, 0) == 0) {		//Above the destination pose with the object in hand
						temp2.z -= 0.01;
						//temp2.z = dest_pose_[idx].position.z + bbx_.dimensions.z*0.5 + 0.01;
						if (move_arm(temp2, 1, 0, 2, 0) == 0) {	//At the destination with hand around the object
							gripper_.open(active_arm_sym_);
							temp2.z += 0.1;
							move_arm(temp2, 1, 0, 2, 0);	//Above the destination pose
							success = true;
							break;
						} else {
							temp.z -= 0.1;
							move_arm(temp, 1, 0, 1, 0);
							gripper_.open(active_arm_sym_);
							temp.z += 0.1;
							move_arm(temp, 1, 0, 1, 0);
							move_arm(active_reset_, 1, 0, 1, 0);
						}
					} else {
						move_arm(temp, 1, 0, 1, 0);
						temp.z -= 0.1;
						move_arm(temp, 1, 0, 1, 0);
						gripper_.open(active_arm_sym_);
						temp.z += 0.1;
						move_arm(temp, 1, 0, 1, 0);
						move_arm(active_reset_, 1, 0, 1, 0);
						//move_arm(reset_posn_.point, 1, 0, 2, 1);
						//return false;
					}
				} else {
					gripper_.open(active_arm_sym_);
					temp.z += 0.1;
					move_arm(temp, 1, 0, 1, 0);
					move_arm(active_reset_, 1, 0, 1, 0);
					//move_arm(reset_posn_.point, 1, 0, 2, 1);
					//return false;
				}
			} else {
				move_arm(active_reset_, 1, 0, 1, 0);
				//move_arm(reset_posn_.point, 1, 0, 2, 1);
				//return false;
			}
		} //else
		//return false;
		move_arm(active_reset_, 1, 0, 1, 0);
	}
	*/


	//		breakpoint();
	return success;
}

void Manipulator::resetCollisionModels() {
	arm_navigation_msgs::CollisionObject reset_object;
	reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
	reset_object.header.frame_id = "base_link";
	reset_object.header.stamp = ros::Time::now();
	reset_object.id = "all";
	collision_object_pub_.publish(reset_object);
	//collision_object_current_id_ = 0;
}

void Manipulator::getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
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

void Manipulator::processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box,
		std::string &collision_name) {
	ROS_INFO("Adding bounding box with dimensions %f %f %f to collision map",
			box.dimensions.x, box.dimensions.y, box.dimensions.z);

	arm_navigation_msgs::CollisionObject collision_object;
	collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

	//collision_name = getNextObjectName();
	collision_name = "object_0";
	collision_object.id = collision_name;
	//target_collision_name_ = collision_name;

	collision_object.header.frame_id = box.pose_stamped.header.frame_id;
	collision_object.header.stamp = ros::Time::now();

	arm_navigation_msgs::Shape shape;
	shape.type = arm_navigation_msgs::Shape::BOX;
	shape.dimensions.resize(3);
	shape.dimensions[0] = box.dimensions.x;
	shape.dimensions[1] = box.dimensions.y;
	shape.dimensions[2] = box.dimensions.z;
	collision_object.shapes.push_back(shape);
	collision_object.poses.push_back(box.pose_stamped.pose);

	collision_object_pub_.publish(collision_object);
}

void Manipulator::collision_op(std::string object1, std::string object2, int operation, double penetration,
		arm_navigation_msgs::CollisionOperation& cop)
{
	if (strcmp(object1.c_str(), "all") == 0 && strcmp(object2.c_str(), "all") == 0) {
		cop.object1 = cop.COLLISION_SET_ALL;
		cop.object2 = cop.COLLISION_SET_ALL;
	} else {
		cop.object1 = object1;
		cop.object2 = object2;
	}
	if (!operation)
		cop.operation = cop.DISABLE;    //0 = Disable, 1 = Enable
	else
		cop.operation = cop.ENABLE;
	cop.penetration_distance = penetration;
}

void Manipulator::add_attached_object(arm_navigation_msgs::AttachedCollisionObject& acob, int operation)
{
	//arm_navigation_msgs::CollisionObject cob;
	add_collision_object(acob.object, 0);
	//acob.link_name = "r_gripper_r_finger_tip_link";
	acob.link_name = "base_link";
	acob.touch_links.push_back("r_end_effector");
}

void Manipulator::add_collision_object(arm_navigation_msgs::CollisionObject& cob, int operation)
{
	cob.id = collision_name_;
	if (operation == 0)
		cob.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD; //0: add, 1: remove, 2: detach and add; 3: attach and remove
	else if (operation == 1)
			cob.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
	else if (operation == 2)
			cob.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
	else if (operation == 3)
		cob.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
	 //0: add, 1: remove, 2: detach and add; 3: attach and remove
	cob.header.frame_id = "base_link";
	cob.header.stamp = ros::Time::now();
	arm_navigation_msgs::Shape object;
	object.type = arm_navigation_msgs::Shape::BOX;  //0: sphere; 1: box; 2: cylinder; 3: mesh
	//arm_navigation_msgs::Shape::CYLINDER;
	object.dimensions.resize(3);
	object.dimensions[0] = bbx_.dimensions.x + 0.07; // need large padding in x direction if push is in x direction bcoz after push the hand will be in collision
	object.dimensions[1] = bbx_.dimensions.y + 0.07;
	object.dimensions[2] = bbx_.dimensions.z + 0.07;
	cob.shapes.push_back(object);

	cob.poses.push_back(bbx_.pose_stamped.pose);
}

int Manipulator::move_arm(geometry_msgs::Point go_to, int cluster_id,
		int collision_object_operation, bool attach, int plan_id, int action)
{
	//if (active_arm_sym_ == 'r') {
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_right_arm("move_right_arm",true);
	//} else
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_left_arm("move_left_arm",true);
	move_right_arm.waitForServer();
	move_left_arm.waitForServer();
	ROS_INFO("Connected to server");

	ROS_INFO("Creating move arm goal");
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = active_arm_;
	//goalA.motion_plan_request.group_name = "right_arm";
	goalA.motion_plan_request.num_planning_attempts = 3;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

	if (cluster_id != 100 && !attach) {
		arm_navigation_msgs::CollisionObject cob;
		add_collision_object(cob, collision_object_operation);
		goalA.planning_scene_diff.collision_objects.push_back(cob);
	}

	//	if (collsion_object_operation == 0) {
	arm_navigation_msgs::CollisionOperation cop;
	if (action == 0 && plan_id == 1) {
		///Setting object1 to "gripper" does not work
		//cop.object1 = "r_end_effector";
		//cop.object2 = collision_name_;
		collision_op("r_end_effector", collision_name_, 0, 1.0, cop);
	} else if (action == 0 && plan_id == 2) {
		//cop.object1 = cop.COLLISION_SET_ALL;
		//cop.object2 = cop.COLLISION_SET_ALL;
		collision_op("all", "all", 0, 1.0, cop);
	}
	//cop.operation = cop.DISABLE;
	//cop.penetration_distance = 1.0;
	goalA.operations.collision_operations.push_back(cop);
	//	}

	if (attach) {
		arm_navigation_msgs::AttachedCollisionObject acob;
		add_attached_object(acob, 0);
		goalA.planning_scene_diff.attached_collision_objects.push_back(acob);
	}

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	if (active_arm_sym_ == 'r')
		desired_pose.link_name = "r_wrist_roll_link";
	else
		desired_pose.link_name = "l_wrist_roll_link";
	desired_pose.pose.position = go_to;
	if (plan_id == 1 && action == 1)
		//desired_pose.pose.position.z = go_to.z + 0.002;
		desired_pose.pose.position.z = go_to.z + 0.19;
	else if (plan_id == 2 && action == 1)
		desired_pose.pose.position.x = go_to.x - 0.08;
	else if (plan_id < 3 && action == 2)
		desired_pose.pose.position.z = go_to.z+0.155;
	else if (plan_id >= 3 && action == 2)
		desired_pose.pose.position.x = go_to.x - 0.08;
	else if (action == 0)
	{
		desired_pose.pose.position.x -= 0.005;
		desired_pose.pose.position.y -= 0.01;
		desired_pose.pose.position.z += 0.18; //0.155;
	}

	desired_pose.pose.orientation.x = 0.0;
	desired_pose.pose.orientation.y = 0.0;
	desired_pose.pose.orientation.z = 0.0;
	desired_pose.pose.orientation.w = 1.0;
	if ((action == 1 && plan_id == 1) || (action == 2 && plan_id < 2)) {
		desired_pose.pose.orientation.x = -0.74;
		desired_pose.pose.orientation.y = -0.04; //-0.04;
		desired_pose.pose.orientation.z = 0.67;
		desired_pose.pose.orientation.w = -0.04; //-0.04;
	} else if (action == 0) {
		// specify rotation in rpy or other methods (euler)
		tf::Transform t1, t2, t3;
		tf::Quaternion q1, q3, q4;
		tf::Matrix3x3 rot_matrix1, rot_matrix2;

		tf::Quaternion q2(bbx_.pose_stamped.pose.orientation.x,
				bbx_.pose_stamped.pose.orientation.y,
				bbx_.pose_stamped.pose.orientation.z,
				bbx_.pose_stamped.pose.orientation.w);
		t2.setRotation(q2);
		t2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		double roll, pitch, yaw;
		t2.getBasis().getRPY(roll, pitch, yaw);
		ROS_INFO("bbx_pose RPY = %f %f %f", roll, pitch, yaw);

		rot_matrix1.setRPY(0.0, 1.57, yaw);
		rot_matrix1.getRotation(q4);

		desired_pose.pose.orientation.x = q4.getX();
		desired_pose.pose.orientation.y = q4.getY();
		desired_pose.pose.orientation.z = q4.getZ();
		desired_pose.pose.orientation.w = q4.getW();
	}

	desired_pose.absolute_position_tolerance.x = 0.01;
	desired_pose.absolute_position_tolerance.y = 0.01;
	desired_pose.absolute_position_tolerance.z = 0.01;  //0.02

	desired_pose.absolute_roll_tolerance = 2.0; //0.04;
	desired_pose.absolute_pitch_tolerance = 2.0; //0.04;
	desired_pose.absolute_yaw_tolerance = 2.0; //0.04;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	bool finished_within_time = false;

	ROS_INFO("Sending goal to move arm");
	//breakpoint();

	if (active_arm_sym_ == 'r')
	{
		move_right_arm.sendGoal(goalA);
		finished_within_time = move_right_arm.waitForResult(ros::Duration(10.0));
		if (!finished_within_time) {
			move_right_arm.cancelGoal();
			ROS_INFO("Timed out achieving goal A");
			return -1;
		} else {
			actionlib::SimpleClientGoalState state = move_right_arm.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success) {
				ROS_INFO("Action finished: %s",state.toString().c_str());
				return 0;
			} else {
				ROS_INFO("Action failed: %s",state.toString().c_str());
				return -1;
			}
		}
	} else {
		move_left_arm.sendGoal(goalA);
		finished_within_time = move_left_arm.waitForResult(ros::Duration(10.0));
		if (!finished_within_time) {
			move_left_arm.cancelGoal();
			ROS_INFO("Timed out achieving goal A");
			return -1;
		} else {
			actionlib::SimpleClientGoalState state = move_left_arm.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success) {
				ROS_INFO("Action finished: %s",state.toString().c_str());
				return 0;
			} else {
				ROS_INFO("Action failed: %s",state.toString().c_str());
				return -1;
			}
		}
	}
	return 0;
}

void Manipulator::breakpoint()
{
	int user_input;
	std::cout << "Save the screen shot if needed. Press 1 and then 'Enter' after you are done." << std::endl;
	std::cin >> user_input;
}

int main(int argc, char **argv)
{
	//initialize the ROS node
	ros::init(argc, argv, "manipulator");
	ros::NodeHandle n;
	Manipulator m(n);

	ROS_INFO("DUPLO: Entered manipulator");
	ros::spin();
	return 0;
}
