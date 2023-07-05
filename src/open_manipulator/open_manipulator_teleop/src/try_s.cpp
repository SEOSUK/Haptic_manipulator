#include "open_manipulator_teleop/try_s.h"
#include "open_manipulator_msgs/DockBatteryService.h"
#include "open_manipulator_msgs/ManipulatorService.h"
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include <cmath>

char ch;
//-------------------전역변수 그룹 ASDF
float endEffector_x_d = 0;
float endEffector_y_d = 0;
float endEffector_z_d = 0;

float endEffector_x = 0;
float endEffector_y = 0;
float endEffector_z = 0;
//--------------------
Eigen::Matrix4d R;
Eigen::Vector4d HapticPosition;
Eigen::Vector4d OriginalPosition;
Eigen::Vector4d Desired_EE_Position;


OpenManipulatorTeleop::OpenManipulatorTeleop()
: node_handle_(""),
  priv_node_handle_("~")
  //OpenManipulatorTeleop::OpenManipulatorTeleop(ros::NodeHandle* nodehandle):node_handle_(*nodehandle)
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT); // present_joint_angle_ : open_manipulator_teleop_keyboard.h 에 선언된 변수
  present_kinematic_position_.resize(3); // present_kinematic_position_ : open_manipulator_teleop_keyboard.h 에 선언된 변수

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();
  initService();

  disableWaitingForEnter();
  ROS_INFO("Teleoperate Manipulator & Joystick");
  //ROS_INFO("OpenManipulator teleoperation using keyboard start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_position");//ASDF    goal_task_space_path_position_only
  goal_drawing_trajectory_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("goal_drawing_trajectory"); // ADD
  DockBatteryClient = node_handle_.serviceClient<open_manipulator_msgs::DockBatteryService>("/DockBatteryService"); // ADD 

}



void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &OpenManipulatorTeleop::manipulatorStatesCallback, this);
  desired_endEffector_Position_Subscriber = node_handle_.subscribe("/Desired_endEffector_Position", 10, &OpenManipulatorTeleop::Desired_endEffector_Position_callback, this);
  HapticSubscriber = node_handle_.subscribe("/phantom/pose", 10, &OpenManipulatorTeleop::hapticCallback, this);
  desired_endEffector_Position_Publisher = node_handle_.advertise<geometry_msgs::Twist>("/Desired_endEffector_Position", 100);
  Effort_Publisher = node_handle_.advertise<sensor_msgs::JointState>("/Effort_FromManipulator", 10);

}

void OpenManipulatorTeleop::initService()
{
  ManipulatorServer = node_handle_.advertiseService("ManipulatorService", &OpenManipulatorTeleop::GUI_Manipulator_Callback, this); // ADD
}

// ADD
void OpenManipulatorTeleop::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if(msg->open_manipulator_moving_state == msg->IS_MOVING)
    {
      open_manipulator_is_moving_ = true;
    }
  else
    {
      open_manipulator_is_moving_ = false;
    }
}

//ASDF
double desired_x_fromGUI = 0;
double desired_y_fromGUI = 0;
double desired_z_fromGUI = 0;
void OpenManipulatorTeleop::Desired_endEffector_Position_callback(const geometry_msgs::Twist &pose)
{/*
  std::vector<double> kinematics_pose;
  double path_time = 0.01;

  desired_x_fromGUI = pose.linear.x;
  desired_y_fromGUI = pose.linear.y;
  desired_z_fromGUI = pose.linear.z;

  ROS_INFO("x : [%lf], y : [%lf], z : [%lf]", desired_x_fromGUI, desired_y_fromGUI, desired_z_fromGUI);



    kinematics_pose.push_back(desired_x_fromGUI);
    kinematics_pose.push_back(desired_y_fromGUI);
    kinematics_pose.push_back(desired_z_fromGUI);
    setTaskSpacePath(kinematics_pose, path_time);*/
}
double real_x = 0;
double real_y = 0;
double real_z = 0;
double start_time = 0;
double end_time = 0;
double total_time = 0;



void OpenManipulatorTeleop::hapticCallback(const geometry_msgs::PoseStamped& msg)
{
  std::vector<double> kinematics_pose;
  geometry_msgs::Twist desired_position;  
  double path_time = 0.001;
/*  desired_x_fromGUI = msg.pose.position.x + 0.134;
  desired_y_fromGUI = msg.pose.position.y - 0.0855954 + 0.000;
  desired_z_fromGUI = msg.pose.position.z + 0.0979452 + 0.241;

  desired_x_fromGUI = + msg.pose.position.y - 0.0855954 + 0.286;
  desired_y_fromGUI = -msg.pose.position.x + 0.000;
  desired_z_fromGUI = msg.pose.position.z + 0.0979452 + 0.202;  //default : home pose
  start_time = msg.pose.orientation.x;
*/

  HapticPosition << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1;

  Desired_EE_Position = R*HapticPosition + OriginalPosition;

  kinematics_pose.push_back(Desired_EE_Position[0]);
  kinematics_pose.push_back(Desired_EE_Position[1]);
  kinematics_pose.push_back(Desired_EE_Position[2]);
  
  setTaskSpacePath(kinematics_pose, path_time);

  desired_position.linear.x = Desired_EE_Position[0];
  desired_position.linear.y = Desired_EE_Position[1];
  desired_position.linear.z = Desired_EE_Position[2];

  desired_endEffector_Position_Publisher.publish(desired_position);
  
  ROS_INFO("%lf, %lf, %lf", Desired_EE_Position[0], Desired_EE_Position[1], Desired_EE_Position[2]);

  
}


double Effort_Joint0 = 0;
void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  sensor_msgs::JointState effort_vector;
  temp_angle.resize(NUM_OF_JOINT);
  effort_vector.effort.resize(NUM_OF_JOINT);

  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    
    if (!msg->name.at(i).compare("joint1"))  
    {temp_angle.at(0) = (msg->position.at(i));
    effort_vector.effort[1] = msg->effort.at(0);
    }
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }

  //effort_vector.effort[0] = msg->effort.at(0);
  //Effort.linear.x = Effort_Joint0;
//  ROS_INFO("Joint 0 : %lf", effort_vector.effort[1]);
//  Effort_Publisher.publish(effort_vector);

  present_joint_angle_ = temp_angle;

}


//KinematicsPose는 현재 위치임
void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  present_kinematic_position_ = temp_position;

  kinematics_pose_.pose = msg->pose;

  endEffector_x = msg->pose.position.x;
  endEffector_y = msg->pose.position.y;
  endEffector_z = msg->pose.position.z;

//--------------------init
//ROS_WARN("haptic x :  %lf, y : %lf, z : %lf",desired_x_fromGUI, desired_y_fromGUI, desired_z_fromGUI);
//ROS_FATAL("hand x : %lf, y : %lf, z : %lf", endEffector_x, endEffector_y, endEffector_z);
}
// bool boolean = true;
// ADD
bool OpenManipulatorTeleop::GUI_Manipulator_Callback(open_manipulator_msgs::ManipulatorService::Request &req, open_manipulator_msgs::ManipulatorService::Response &res)
{
  
  //int result = req.isChange;
  if(req.Manipul == 1)
  {
    printf("isChange = 1\n");
    change_battery = true;
  }
  else change_battery = false;
  
  ROS_INFO("service callback activated");
  //printf("%d",result);
  // if(req.isChange == 1) ch = 's';
  
  /* if(result == 1) change_battery = true;
  else change_battery = false; */
  //return true;
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

// ADD
bool OpenManipulatorTeleop::getOpenManipulatorMovingState()
{
  return open_manipulator_is_moving_;
}

bool OpenManipulatorTeleop::getManipulatorService()
{
  return change_battery;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
 //ASDFASDF srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}


bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;


  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

// ADD
bool OpenManipulatorTeleop::setDrawingTrajectory(std::string name, std::vector<double> arg, double path_time)
{
  open_manipulator_msgs::SetDrawingTrajectory srv;
  srv.request.end_effector_name = "gripper";
  srv.request.drawing_trajectory_name = name;
  srv.request.path_time = path_time;
  for(int i = 0; i < arg.size(); i ++)
    srv.request.param.push_back(arg.at(i));

  if(goal_drawing_trajectory_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("i : init pose\n");
  printf("h : home pose\n");
  printf("s : continuous motion\n");
  printf("r : ready\n");
  printf("v : ready 2\n");
  printf("1 : pose #1\n");
  printf("2 : pose #1 - gripper close\n");
  printf("3 : pose #2\n");
  printf("4 : pose #3\n");
  printf("5 : pose #4\n");
  printf("6 : pose #4 - gripper open\n");
  printf("7 : pose #5\n");
  printf("8 : pose #6 - home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");
}



void OpenManipulatorTeleop::initEndEffector()
{
  endEffector_x = getPresentKinematicsPose().at(0);
  endEffector_y = getPresentKinematicsPose().at(1);
  endEffector_z = getPresentKinematicsPose().at(2);
}
/////////////////
//-------------------목표위치----------------//

bool OpenManipulatorTeleop::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose service;

  service.request.end_effector_name = "gripper";

  service.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  service.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  service.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  service.request.kinematics_pose.pose.orientation.w = kinematics_pose_.pose.orientation.w;
  service.request.kinematics_pose.pose.orientation.x = kinematics_pose_.pose.orientation.x;
  service.request.kinematics_pose.pose.orientation.y = kinematics_pose_.pose.orientation.y;
  service.request.kinematics_pose.pose.orientation.z = kinematics_pose_.pose.orientation.z;

  service.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(service))
  {
  end_time = ros::Time::now().toSec();
  total_time = end_time - start_time;
  //  ROS_INFO("start_time : %lf, end_time : %lf, total_time : %lf", start_time, end_time, total_time);

    return service.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::setGoal()
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.822);
    joint_name.push_back("joint2"); joint_angle.push_back(0.231);
    joint_name.push_back("joint3"); joint_angle.push_back(0.185);
    joint_name.push_back("joint14"); joint_angle.push_back(1.151);
    setJointSpacePath(joint_name, joint_angle, path_time);


   // joint_angle.push_back(-0.023);
   // setToolControl(joint_angle);

  ros::spinOnce();

  ros::Rate loop(100);

  loop.sleep();
  
}

void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "try_s");
  OpenManipulatorTeleop openManipulatorTeleop;
  char ch;
  int move = 0;
  //openManipulatorTeleop.printText();


//---------publisher 선언------------//
ros::NodeHandle su;
ros::Publisher publisher = su.advertise<geometry_msgs::Twist>("Now_endEffector_Position", 10);
geometry_msgs::Twist endPose_d;
/////////////////////////////////////

R << 0, 1, 0, -0.0855954,
    -1,  0, 0, 0,
     0, 0, 1, 0.0979452,
     0, 0, 0, 1;
OriginalPosition << 0.200, 0, 0.200, 0;


  ros::Rate loop(100);

  while(ros::ok())
  {
         openManipulatorTeleop.initEndEffector();
         endPose_d.linear.x = endEffector_x;
         endPose_d.linear.y = endEffector_y;
         endPose_d.linear.z = endEffector_z;
         


         publisher.publish(endPose_d);
        ros::spinOnce(); 
        loop.sleep();
  }
  return 0;
}