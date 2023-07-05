#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

double x = 0;
double y = 0;
double z = 0;

void chatterCallback(const geometry_msgs::PoseStamped& msg)
{
  x = msg.pose.position.x;
  y = msg.pose.position.y;
  z = msg.pose.position.z;
  ROS_INFO("%lf, %lf, %lf", x, y, z);
}

/* void rotationmatrix(const geometry_msgs::PoseStamped& msg)
{
  x = msg.pose.position.x;
  y = msg.pose.position.y;
  z = msg.pose.position.z;

  Eigen::Matrix3d rotx(3,3);
  rotx << 1,     0,           0
          0, cos(theta), -sin(theta)
          0, sin(theta), cos(theta);
  
} */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "haptic");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/phantom/pose", 1000, chatterCallback);

  ros::spin();

  return 0;
}