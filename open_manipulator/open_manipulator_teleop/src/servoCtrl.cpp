#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include "open_manipulator_msgs/DockBatteryService.h"
#include <cstdlib> //atoll 쓰려고 넣어놈

//클라이언트로부터 값 받아서 그 값 퍼블리쉬로 서보 돌리기
int angle = 0;


bool Docking_Callback(open_manipulator_msgs::DockBatteryService::Request &req, open_manipulator_msgs::DockBatteryService::Response &res)
{
	if(req.Dock_Do){
		angle = 90;
	}
	else{
		angle = 0;
	}
	return true;
}

int main(int argc, char **argv)
 {
  ros::init(argc, argv, "servoCtrl");


    ros::NodeHandle n;
    open_manipulator_msgs::DockBatteryService Service;
    std_msgs::UInt16 cmd_msg;

	ros::ServiceServer Server = n.advertiseService("/DockBatteryService", Docking_Callback); //ASDF
	ros::Publisher publisher = n.advertise<std_msgs::UInt16>("servo", 100);   //퍼블리셔 이름은 pubname

    ros::Rate rate(100);   

	while(ros::ok())
	{
    cmd_msg.data = angle;
    ROS_INFO("angle : %d", angle);
	publisher.publish(cmd_msg);
    ros::spinOnce();
    rate.sleep();
	}

    return 0;
 }
