#include "ros/ros.h"
#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"
#include "open_manipulator_msgs/ManipulatorService.h"

#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ManipulatorCtrl");

  if (argc != 2)
  {
    ROS_INFO("usage: ManipulatorCtrl X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient ManipulatorClient = n.serviceClient<open_manipulator_msgs::ManipulatorService>("ManipulatorService");
  open_manipulator_msgs::ManipulatorService srv;
  srv.request.Manipul = true;

  if (ManipulatorClient.call(srv))
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

