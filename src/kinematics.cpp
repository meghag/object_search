#include <kinematics_base/kinematics_base.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <tf/tf.h>

kinematics::KinematicsBase * inverse_kinematics[2] = {NULL, NULL};

bool inverse_kinematics_initialized = false;

void init_kinematics()
{

    if (!inverse_kinematics_initialized)
    {
        pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_("kinematics_base","kinematics::KinematicsBase");//,"pr2_arm_kinematics::PR2ArmKinematicsPlugin");

        inverse_kinematics[0] = kinematics_loader_.createClassInstance("pr2_arm_kinematics/PR2ArmKinematicsPlugin");
        inverse_kinematics[0]->initialize("right_arm",
                                          "torso_lift_link",
                                          "r_wrist_roll_link",
                                          0.1);

        inverse_kinematics[1] = kinematics_loader_.createClassInstance("pr2_arm_kinematics/PR2ArmKinematicsPlugin");
        inverse_kinematics[1]->initialize("left_arm",
                                          "torso_lift_link",
                                          "l_wrist_roll_link",
                                          0.1);

        inverse_kinematics_initialized = true;
    }

}

int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &seed ,std::vector<double> &jointValues)
{

    init_kinematics();

    arm_navigation_msgs::ArmNavigationErrorCodes error_code;

    geometry_msgs::Pose pose;
    tf::poseTFToMsg(targetPose,pose);

    int num_retries = 0;

    while ((error_code.val == -2) && (num_retries < 10)) {

        inverse_kinematics[arm]->getPositionIK(pose,
                                           seed,
                                           jointValues,
                                           error_code.val);

        if (error_code.val == -2)
            ROS_INFO("IK FAILED DUE TO TIME_OUT, retrying");

        std::string err_string = arm_navigation_msgs::armNavigationErrorCodeToString(error_code);

        std::cout << "seeded IK error" << error_code.val << " is " << err_string << std::endl;

    }


    return error_code.val;

}



int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues)
{

    init_kinematics();

    double tmp[] = {-1.6169497181369175, 0.15545771558980806, -1.250062782260915, -2.120402271101688, 11.660629107874348, -1.3540518577418752, -4.570108384006465};

    if (arm == 1)
        for (size_t k = 0; k < 7; ++k)
            tmp[k] = -tmp[k];

    std::vector<double> seed_state(tmp,tmp+7);
    //std::vector<double> solution(tmp,tmp+7);
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;

    //geometry_msgs::Pose pose;
    //tf::poseTFToMsg(targetPose,pose);

    //std::cout << "Pose " << pose << std::endl;

    //inverse_kinematics[arm]->getPositionIK(pose,
      //                                     seed_state,
        //                                   jointValues,
          //                                 error_code.val);
    error_code.val = get_ik(arm, targetPose, seed_state, jointValues);

    //std::cout << "Error code" << error_code << std::endl;
    //for (int k = 0; k < 7; k++)
      //  std::cout << "joint " << k << " : " << solution[k] << std::endl;

    return error_code.val;

}
