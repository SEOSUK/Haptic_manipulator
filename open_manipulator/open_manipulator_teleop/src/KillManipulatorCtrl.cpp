#include "ros/ros.h"
#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"
#include "open_manipulator_msgs/KillManipulatorService.h"

#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KillManipulatorCtrl");

  if (argc != 2)
  {
    ROS_INFO("KillManipulatorCtrl true/false");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient KillManipulatorClient = n.serviceClient<open_manipulator_msgs::KillManipulatorService>("KillManipulatorService");
  open_manipulator_msgs::KillManipulatorService srv;
  srv.request.KillManipul = true;

  if (KillManipulatorClient.call(srv))
  {
    ROS_INFO("send");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}

