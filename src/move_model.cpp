#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>

int main(int argc,char **argv)
{

    ros::init(argc, argv, "move_models");

    ros::NodeHandle *nh_ = new ros::NodeHandle();

    gazebo_msgs::GetModelState getModelState;

    getModelState.request.model_name = "green_box";
    getModelState.request.relative_entity_name = ""; // reference world


    ROS_INFO("Waiting for service to come up");
    ros::service::waitForService("gazebo/get_model_state");
    if (ros::service::call("gazebo/get_model_state", getModelState))
    {
        std::cout << (getModelState.response.success ? "sucess!" : "fail!") << std::endl;
        std::cout << getModelState.response.pose << std::endl;
        std::cout << getModelState.response.twist << std::endl;
    }
    else
    {
        ROS_ERROR("Unsucessful");
    }


    gazebo_msgs::SetModelState setModelState;
    setModelState.request.model_state.model_name = "green_box";
    setModelState.request.model_state.reference_frame = ""; // reference world
    setModelState.request.model_state.pose = getModelState.response.pose;
    if (setModelState.request.model_state.pose.position.x < .6)
        setModelState.request.model_state.pose.position.x += .1;
    else
        setModelState.request.model_state.pose.position.x -= .1;

    //setModelState.model_state.twist = getModelState.response.twist;
    tf::Vector3 nullVec(0,0,0);
    tf::vector3TFToMsg(nullVec, setModelState.request.model_state.twist.linear);
    tf::vector3TFToMsg(nullVec, setModelState.request.model_state.twist.angular);

    if (ros::service::call("gazebo/set_model_state", setModelState))
    {
        std::cout << (setModelState.response.success ? "sucess!" : "fail!") << std::endl;
        std::cout << setModelState.response.status_message << std::endl;
    }
    else
    {
        ROS_INFO("FAILURE");
    }

}
