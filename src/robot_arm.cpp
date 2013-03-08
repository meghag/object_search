

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_common_action_msgs/ArmMoveIKAction.h>
#include <tf/transform_listener.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/PositionIKRequest.h>
#include <kinematics_msgs/GetPositionFK.h>


#include "../include/robot_arm.h"


// ugly forward decl. make a class
int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues);

RobotArm *RobotArm::instance[] = {0L,0L};

RobotArm* RobotArm::getInstance(int side)
{
    if (!instance[side])
        instance[side] = new RobotArm(side);
    return instance[side];
}


RobotArm::RobotArm(int side)
{
    side_ = side;
    traj_client_ = new actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction >((side == 0) ? "/r_arm_controller/joint_trajectory_action" : "/l_arm_controller/joint_trajectory_action", true);
    ROS_INFO("WAITING FOR SERVER TO START");
    traj_client_->waitForServer();

    move_arm_client_ = new  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>((side == 0) ? "move_right_arm" : "move_left_arm",true);
    move_arm_client_->waitForServer();

    gripper_client_ = new GripperClient(side ? "l_gripper_controller/gripper_action" : "r_gripper_controller/gripper_action", true);
}

RobotArm::~RobotArm()
{
    traj_client_->cancelAllGoals();
    delete traj_client_;
}

int
RobotArm::move_arm_via_ik(tf::Pose goalPose)
{
    std::vector<double> ik_result;
    int error_code = get_ik(side_, goalPose, ik_result);
    ROS_INFO("ARM_IK ERROR CODE %i", error_code);
    move_arm_joint(ik_result);
    return 0;
}

int
RobotArm::move_arm_joint(std::vector<double> jointState)
{
    // 1 second, rather fast
    startTrajectory(createTrajectory(jointState,1), true);
    return 0;
}

pr2_controllers_msgs::JointTrajectoryGoal RobotArm::createTrajectory(std::vector<double> jointState, double dur)
{
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    if (side_==0)
    {
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else
    {
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[ind].positions[j] = jointState[j];
        goal.trajectory.points[ind].velocities[j] = 0.0;
        //std::cout << "j  : " <<  goal.trajectory.points[ind].positions[j] << std::endl;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(dur);

    //we are done; return the goal
    return goal;
}


void RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool waitForFinish)
{
    goal.trajectory.header.stamp = ros::Time::now(); // start right away

    //ROS_INFO("SENDING ARM GOAL");

    traj_client_->sendGoal(goal);

    if (waitForFinish)
    {
        try
        {
            ROS_INFO("WAITING FOR ARM TO REACH GOAL..");
            traj_client_->waitForResult();
        }
        catch ( boost::thread_interrupted ti )
        {
            traj_client_->cancelAllGoals();
            ROS_ERROR("RobotArm startTrajectory side %i Interrupted: Thread killed. Cancelling all arm ac goals", this->side_);
            throw ti;
        }
    }
}


void RobotArm::reset_arms()
{
    std::cout << "Reset arm position" << std::endl;
    std::vector<double> jstate[2];
    jstate[0].resize(7);
    jstate[1].resize(7);

    jstate[0][0] = -1.6168837569724208;
    jstate[0][1] =  0.15550442262485537;
    jstate[0][2] =  -1.25;
    jstate[0][3] =  -2.1222118472906528;
    jstate[0][4] =  -0.90575412503025221;
    jstate[0][5] =  -1.3540678462623723;
    jstate[0][6] =  1.7130631649684915;

    jstate[1][0] = 1.5371426009988673;
    jstate[1][1] =  0.15318173630933338;
    jstate[1][2] =  1.25;
    jstate[1][3] =  -2.1017963799253305;
    jstate[1][4] =  0.66713731189146697;
    jstate[1][5] =  -0.7240180626857029;
    jstate[1][6] =  -10.049029141041331;

    RobotArm::getInstance(0)->move_arm_joint(jstate[0]);
    RobotArm::getInstance(1)->move_arm_joint(jstate[1]);
}


#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int RobotArm::move_arm(tf::Pose goalPose)
{

    arm_navigation_msgs::MoveArmGoal goalA;

    goalA.motion_plan_request.group_name = side_ ? "left_arm" : "right_arm";
    goalA.motion_plan_request.num_planning_attempts = 300;
    goalA.motion_plan_request.planner_id = std::string("");
    goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalA.motion_plan_request.allowed_planning_time = ros::Duration(15.0);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "torso_lift_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = side_ ? "l_wrist_roll_link" : "r_wrist_roll_link";

    tf::poseTFToMsg(goalPose, desired_pose.pose);

    std::cout << "Move arm goal pose: " << desired_pose << std::endl;

    desired_pose.absolute_position_tolerance.x = 0.01;
    desired_pose.absolute_position_tolerance.y = 0.01;
    desired_pose.absolute_position_tolerance.z = 0.01;

    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;

    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

    bool finished_within_time = false;

    move_arm_client_->sendGoal(goalA);

    ros::Duration(.01).sleep();

    finished_within_time = move_arm_client_->waitForResult(ros::Duration(16.0));
    if (!finished_within_time) {
        move_arm_client_->cancelGoal();
        ROS_INFO("DUPLO: Timed out achieving goal A");
        return -1;
    } else {
        actionlib::SimpleClientGoalState state = move_arm_client_->getState();
        bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success) {
            ROS_INFO("DUPLO: Action finished: %s",state.toString().c_str());
            return 0;
        } else {
            ROS_INFO("DUPLO: Action failed: %s",state.toString().c_str());
            return -1;
        }
    }

    return -1;
}

int RobotArm::home_arm()
{
    tf::Pose goal;

    if (side_ == 0)
    {
        goal.setOrigin(tf::Vector3(0.244, -0.542, 0.049));
        goal.setRotation(tf::Quaternion(-0.426, 0.091, 0.645, 0.627));
    }
    else
    {
      goal.setOrigin(tf::Vector3(0.374, 0.527, 0.259));
      goal.setRotation(tf::Quaternion(-0.536, 0.384, -0.348, 0.666));
    }
    return move_arm(goal);
}


void RobotArm::open_gripper(double amount, double effort)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    //open.command.position = 0.085 * amount;
    open.command.position = amount;
    open.command.max_effort = effort;

    gripper_client_->sendGoal(open);
    //gripper_client_->waitForResult();
    if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The gripper opened!");
    else
        ROS_INFO("The gripper failed to open.");
}
