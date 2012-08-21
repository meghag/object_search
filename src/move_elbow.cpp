#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "elbow_mover");
    ros::NodeHandle n;

    if (argc != 4) {
        ROS_ERROR("Incorrect number of arguments");
        return -1;
    }

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("reset_position",1);

    //actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_left_arm",true);
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
    move_arm.waitForServer();
    ROS_INFO("DUPLO: Connected to server");
    arm_navigation_msgs::MoveArmGoal goalA;

    //goalA.motion_plan_request.group_name = "left_arm";
    goalA.motion_plan_request.group_name = "right_arm";
    goalA.motion_plan_request.num_planning_attempts = 1;
    goalA.motion_plan_request.planner_id = std::string("");
    goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    //desired_pose.link_name = "l_wrist_roll_link";
    desired_pose.link_name = "r_elbow_flex_link";
//	desired_pose.link_name = "r_gripper_palm_link";
    desired_pose.pose.position.x = atof(argv[1]); // = go_to;
    desired_pose.pose.position.y = atof(argv[2]);
    desired_pose.pose.position.z = atof(argv[3]);
//	desired_pose.pose.position.z = go_to.z + 0.194;
    /*
      	desired_pose.pose.position.x =  0.6; //0.75;
    	desired_pose.pose.position.y = -0.5;//-0.188;
    	desired_pose.pose.position.z = 0;
    */
    desired_pose.pose.orientation.x = 0;
    desired_pose.pose.orientation.y = 0;
    desired_pose.pose.orientation.z = 0;
    desired_pose.pose.orientation.w = 1;

    geometry_msgs::PoseStamped to_pub;
    to_pub.pose = desired_pose.pose;
    to_pub.header.frame_id = "base_link"; //desired_pose.header.frame_id;
    to_pub.header.stamp = ros::Time::now();
    pose_pub.publish(to_pub);


    /*
    	desired_pose.pose.orientation.x = -0.74;
    	desired_pose.pose.orientation.y = -0.04;
    	desired_pose.pose.orientation.z = 0.67;
    	desired_pose.pose.orientation.w = -0.04;
    */
    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;

    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;

    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);


    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
    if (!finished_within_time) {
        move_arm.cancelGoal();
        ROS_INFO("DUPLO: Timed out achieving goal A");
        return -1;
    } else {
        actionlib::SimpleClientGoalState state = move_arm.getState();
        bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success) {
            ROS_INFO("DUPLO: Action finished: %s",state.toString().c_str());
            return 0;
        } else {
            ROS_INFO("DUPLO: Action failed: %s",state.toString().c_str());
            return -1;
        }
    }

    ros::spin();

    return 0;
}


