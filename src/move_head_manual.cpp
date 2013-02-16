#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHead
{
private:
  PointHeadClient* point_head_client_;

public:
  //! Action client initialization 
  RobotHead()
  {
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
  }

  ~RobotHead()
  {
    delete point_head_client_;
  }

  //! Points the high-def camera frame at a point in a given frame  
  void lookAt(std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";
	//goal.pointing_frame = "head_mount_kinect_rgb_optical_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(2));
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotHead head;
  //head.lookAt("base_link", 0.2, -0.1, 1.36);
  head.lookAt("base_link", 0.2, -0.1, 1.3);
}
