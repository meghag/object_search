

#include "../include/grasp_planning.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "grasp_designer");
	ros::NodeHandle n;

    GraspPlanning gp;
    ros::Rate rt(1);
    while (ros::ok())
    {
        gp.visualizeGrasp(0,"r_wrist_roll_link");
        rt.sleep();
        ROS_INFO("tick");
    }
}
