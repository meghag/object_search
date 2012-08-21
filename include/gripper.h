#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper
{
private:
	GripperClient* left_gripper_client_;
	GripperClient* right_gripper_client_;

public:
	//Action client initialization
	Gripper() {

		//Initialize the client for the Action interface to the gripper controller
		//and tell the action client that we want to spin a thread by default
		right_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
		left_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);

		//wait for the gripper action server to come up
		while(!right_gripper_client_->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
		}
		while(!left_gripper_client_->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
		}
	}

	~Gripper() {
		delete right_gripper_client_;
		delete left_gripper_client_;
	}

	//Open the gripper
	bool open(int which_gripper, double position) {
		pr2_controllers_msgs::Pr2GripperCommandGoal open;
		open.command.position = position; //0.03; //0.08
		open.command.max_effort = -1.0;  // Do not limit effort (negative)

		ROS_INFO("Sending open goal");
		if (which_gripper == 0) {
			//Left gripper
			left_gripper_client_->sendGoal(open);
			left_gripper_client_->waitForResult();
			if(left_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("The left gripper opened!");
				return true;
			} else {
				ROS_INFO("The left gripper failed to open.");
				return false;
			}
		} else if (which_gripper == 1) {
			right_gripper_client_->sendGoal(open);
			right_gripper_client_->waitForResult();
			if(right_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("The right gripper opened!");
				return true;
			} else {
				ROS_INFO("The right gripper failed to open.");
				return false;
			}
		}
		return false;
	}

	//Close the gripper
	bool close(int which_gripper) {
		pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
		squeeze.command.position = 0.0;
		squeeze.command.max_effort = 50.0;  // Close gently

		ROS_INFO("Sending squeeze goal");
		if (which_gripper == 0) {
			//Left gripper
			left_gripper_client_->sendGoal(squeeze);
			left_gripper_client_->waitForResult();
			if(left_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("The left gripper closed!");
				return true;
			} else {
				ROS_INFO("The left gripper failed to close.");
				return false;
			}
		} else if (which_gripper == 1) {
			right_gripper_client_->sendGoal(squeeze);
			right_gripper_client_->waitForResult();
			if(right_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("The right gripper closed!");
				return true;
			} else {
				ROS_INFO("The right gripper failed to close.");
				return false;
			}
		}
	}
};
