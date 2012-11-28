#include <kinematics_base/kinematics_base.h>

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


int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues)
{

    init_kinematics();

    double tmp[] = {-1.6169497181369175, 0.15545771558980806, -1.250062782260915, -2.120402271101688, 11.660629107874348, -1.3540518577418752, -4.570108384006465};

    std::vector<double> seed_state(tmp,tmp+7);
    //std::vector<double> solution(tmp,tmp+7);
    int error_code = 0;

    geometry_msgs::Pose pose;
    tf::poseTFToMsg(targetPose,pose);

    //std::cout << "Pose " << pose << std::endl;

    inverse_kinematics[arm]->getPositionIK(pose,
                                           seed_state,
                                           jointValues,
                                           error_code);

    //std::cout << "Error code" << error_code << std::endl;
    //for (int k = 0; k < 7; k++)
      //  std::cout << "joint " << k << " : " << solution[k] << std::endl;

    return error_code;

}
