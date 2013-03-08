#ifndef __ROBOTARM_H__
#define __ROBOTARM_H__

#include <tf/tf.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>

class RobotArm
{
public:

    static RobotArm* getInstance(int side = 0);

    // pose has to be in ik frame, usually torso_lift_link, then goes through ik and then joint trajectory
    int move_arm_via_ik(tf::Pose goalPose);

    // direct joint control
    int move_arm_joint(std::vector<double> jointState);

    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal,bool waitForFinish = true);

    pr2_controllers_msgs::JointTrajectoryGoal createTrajectory(std::vector<double> jointState, double dur);

    static void reset_arms();

    int move_arm(tf::Pose goalPose);

private:

    RobotArm(int side);

    ~RobotArm();

    int side_;

    actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > *traj_client_;

    static RobotArm *instance[];

    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm_client_;


};


#endif
