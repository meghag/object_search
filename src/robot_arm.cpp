

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_common_action_msgs/ArmMoveIKAction.h>
#include <tf/transform_listener.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/PositionIKRequest.h>
#include <kinematics_msgs/GetPositionFK.h>

#include "../include/robot_arm.h"


// ugly forward decl. make a class
int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues);


RobotArm::RobotArm(int side)
{
    side_ = side;
    traj_client_ = new actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction >((side == 0) ? "r_arm_controller/joint_trajectory_action" : "l_arm_controller/joint_trajectory_action", true);
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
    get_ik(side_, goalPose, ik_result);
    move_arm_joint(ik_result);
}

int
RobotArm::move_arm_joint(std::vector<double> jointState)
{
    // 3 seconds, thats slooow
    startTrajectory(createTrajectory(jointState,3), true);
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
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(dur);

    //we are done; return the goal
    return goal;
}


void RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool waitForFinish)
{
    goal.trajectory.header.stamp = ros::Time::now(); // start right away

    traj_client_->sendGoal(goal);

    if (waitForFinish)
    {
        try
        {
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


