#include "open_manipulator_teleop/s.h"
#include "open_manipulator_msgs/DockBatteryService.h"
#include "open_manipulator_msgs/ManipulatorService.h"
#include <geometry_msgs/Twist.h>

char ch;
//-------------------전역변수 그룹 ASDF
float endEffector_x_d = 0;
float endEffector_y_d = 0;
float endEffector_z_d = 0;

float endEffector_x = 0;
float endEffector_y = 0;
float endEffector_z = 0;
//--------------------



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
  ROS_INFO("OpenManipulator teleoperation using keyboard start");
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
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");//ASDF
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
  Effort_Publisher = node_handle_.advertise<geometry_msgs::Twist>("/Effort_FromManipulator", 10);

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
  double path_time = 0.01;
  /*desired_x_fromGUI = msg.pose.position.x + 0.134;
  desired_y_fromGUI = msg.pose.position.y - 0.0855954 + 0.000;
  desired_z_fromGUI = msg.pose.position.z + 0.0979452 + 0.241;
*/

  desired_x_fromGUI = - msg.pose.position.y + 0.0855954 + 0.134;
  desired_y_fromGUI = -msg.pose.position.x + 0.000;
  desired_z_fromGUI = msg.pose.position.z + 0.0979452 + 0.241;  //default : home pose
  start_time = msg.pose.orientation.x;
//  ROS_INFO("x : [%lf], y : [%lf], z : [%lf]", desired_x_fromGUI, desired_y_fromGUI, desired_z_fromGUI);

  kinematics_pose.push_back(desired_x_fromGUI);
  kinematics_pose.push_back(desired_y_fromGUI);
  kinematics_pose.push_back(desired_z_fromGUI);
  
  setTaskSpacePath(kinematics_pose, path_time);

  desired_position.linear.x = desired_x_fromGUI;
  desired_position.linear.y = desired_y_fromGUI;
  desired_position.linear.z = desired_z_fromGUI;


  desired_endEffector_Position_Publisher.publish(desired_position);
}


double Effort_Joint0 = 0;
void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }

  Effort_Joint0 = msg->effort.at(0);  //이걸봐!
  Effort.linear.x = Effort_Joint0;
  ROS_INFO("Joint 0 : %lf", Effort_Joint0);
  Effort_Publisher.publish(Effort);

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
  
  /*
  std::vector<double> kinematics_pose;
  double path_time = 2.0;
  double x_pose = 0;
  double y_pose = 0;
  double z_pose = 0;

  if(ch == '1' && open_manipulator_is_moving_ == false)
  {
    printf("x pose"); std::cin >> x_pose; kinematics_pose.push_back(0.134);
    printf("y pose"); std::cin >> y_pose; kinematics_pose.push_back(0.000);
    printf("z pose"); std::cin >> z_pose; kinematics_pose.push_back(0.241);
    setTaskSpacePath(kinematics_pose, path_time);
  }*/
  /*
  if(ch == '1' && open_manipulator_is_moving_ == false)
  {
    printf("x pose"); kinematics_pose.push_back(0.134);
    printf("y pose"); kinematics_pose.push_back(0.000);
    printf("z pose"); kinematics_pose.push_back(0.241);
    setTaskSpacePath(kinematics_pose, path_time);
  }

    if(ch == '2' && open_manipulator_is_moving_ == false)
  {
    printf("x pose"); kinematics_pose.push_back(0.125);
    printf("y pose"); kinematics_pose.push_back(0.122);
    printf("z pose"); kinematics_pose.push_back(0.060);
    setTaskSpacePath(kinematics_pose, path_time);
  }

  if(ch == '9' && open_manipulator_is_moving_ == false)
  {
    printf("x pose"); kinematics_pose.push_back(0.125);
    printf("y pose"); kinematics_pose.push_back(0.122);
    printf("z pose"); kinematics_pose.push_back(0.020);
    setTaskSpacePath(kinematics_pose, path_time);
  }
//home pose : 0.134, 0.000, 0.241

  
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);
  open_manipulator_msgs::DockBatteryService Service;
  if (ch == 'i' && open_manipulator_is_moving_ == false)
  {
    printf("input : i \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if ((ch == 'h' || ch == '8') && open_manipulator_is_moving_ == false)
  {
    printf("input : h \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '1' && open_manipulator_is_moving_== false)
  {
    printf("input : 1 \tpose #1\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.822);//0.822
    joint_name.push_back("joint2"); joint_angle.push_back(0.150);//0.150
    joint_name.push_back("joint3"); joint_angle.push_back(-0.038);
    joint_name.push_back("joint4"); joint_angle.push_back(1.454);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '2' && open_manipulator_is_moving_== false)
  {
    printf("input : 2 \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.023);
    Service.request.Dock_Do = true;
    DockBatteryClient.call(Service);
    setToolControl(joint_angle);
  }
  else if (ch == '9' && open_manipulator_is_moving_== false)
  {
    printf("input : 9 \tdown\n");

    std::string name; name = "line";
    std::vector<double> arg;
    double path_time = 2.0;
    arg.push_back(0);
    arg.push_back(0);
    arg.push_back(-0.04);
    setDrawingTrajectory(name, arg, path_time);
  }
  else if (ch == '3' && open_manipulator_is_moving_== false)
  {
    printf("input : 3 \tpose #2\n");

    std::string name; name = "line";
    std::vector<double> arg;
    double path_time = 2.0;
    arg.push_back(0);
    arg.push_back(0);
    arg.push_back(0.04);
    setDrawingTrajectory(name, arg, path_time);
  }
  else if (ch == 'p' && open_manipulator_is_moving_== false)
  {
    printf("input : p \tsdf\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.000);//0.822
    joint_name.push_back("joint2"); joint_angle.push_back(0.000);
    joint_name.push_back("joint3"); joint_angle.push_back(0.096);
    joint_name.push_back("joint4"); joint_angle.push_back(1.270);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '4' && open_manipulator_is_moving_== false)
  {
    printf("input : 4 \tpose #3\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(getPresentJointAngle().at(0)-1);
    joint_name.push_back("joint2"); joint_angle.push_back(getPresentJointAngle().at(1));
    joint_name.push_back("joint3"); joint_angle.push_back(getPresentJointAngle().at(2));
    joint_name.push_back("joint4"); joint_angle.push_back(getPresentJointAngle().at(3));
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '5' && open_manipulator_is_moving_== false)
  {
    printf("input : 5 \tpose #4\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(getPresentJointAngle().at(0));
    joint_name.push_back("joint2"); joint_angle.push_back(0.649);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.268);
    joint_name.push_back("joint4"); joint_angle.push_back(1.144);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '6' && open_manipulator_is_moving_== false)
  {
    printf("input : 6 \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.02);
    setToolControl(joint_angle);
  }
  else if (ch == '7' && open_manipulator_is_moving_== false)
  {
    printf("input : 7 \tpose #5\n");

    std::string name; name = "line";
    std::vector<double> arg;
    double path_time = 2.0;
    arg.push_back(0);
    arg.push_back(0);
    arg.push_back(0.05);
    setDrawingTrajectory(name, arg, path_time);
    Service.request.Dock_Do = false;
    DockBatteryClient.call(Service);
  }
  else if (ch == 'r' && open_manipulator_is_moving_== false)
  {
    printf("input : r \tready\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-1.583);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.563);
    joint_name.push_back("joint3"); joint_angle.push_back(1.564);
    joint_name.push_back("joint4"); joint_angle.push_back(-1.422);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'v' && open_manipulator_is_moving_== false)
  {
    printf("input : v \tready 2\n");

    std::string name; name = "line";
    std::vector<double> arg;
    double path_time = 2.0;
    arg.push_back(0);
    arg.push_back(0);
    arg.push_back(-0.04);
    setDrawingTrajectory(name, arg, path_time);
  }
  */
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

// char ch = 'N';

/* int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "try2");
  
  //ros::NodeHandle seuk;
  //auto openManipulatorTeleop = OpenManipulatorTeleop();
  OpenManipulatorTeleop openManipulatorTeleop;
  //ros::ServiceServer srv = seuk.advertiseService("ChangeService", &OpenManipulatorTeleop::GUI_Change_Callback, &openManipulatorTeleop);
  //char ch;
  int move = 0;
  openManipulatorTeleop.printText();
  while (ros::ok())//&& (ch = std::getchar()) != 'q')
  {
    if(openManipulatorTeleop.getChangeService() == true) printf("true\n");
    /* if(openManipulatorTeleop.getChangeService() == true)
    {       
      ros::Rate rate(2);
      while(ros::ok())
      {
        if(openManipulatorTeleop.getOpenManipulatorMovingState() == false) 
        {
          ROS_INFO("stopping");
          {
            move++; 
            switch(move)
            {
              case 1: openManipulatorTeleop.setGoal('r'); printf("r send\n"); break;
              case 2: openManipulatorTeleop.setGoal('h'); printf("h send\n"); break;
              case 3: openManipulatorTeleop.setGoal('1'); printf("1 send\n"); break;
              case 4: openManipulatorTeleop.setGoal('2'); printf("2 send\n"); break;
              case 5: openManipulatorTeleop.setGoal('9'); printf("9 send\n"); break;
              case 6: openManipulatorTeleop.setGoal('3'); printf("3 send\n"); break;
              case 7: openManipulatorTeleop.setGoal('4'); printf("4 send\n"); break;
              case 8: openManipulatorTeleop.setGoal('5'); printf("5 send\n"); break;
              case 9: openManipulatorTeleop.setGoal('6'); printf("6 send\n"); break;
              case 10: openManipulatorTeleop.setGoal('7'); printf("7 send\n"); break;
              case 11: openManipulatorTeleop.setGoal('h'); printf("h send\n"); break;
              case 12: openManipulatorTeleop.setGoal('r'); printf("r send\n"); break;
              case 13: printf("finish\n"); break;
              default : printf("skip\n"); break;
            }
          }
        }
        else ROS_INFO("moving");

        if(move > 12) { move = 0; break; }
        rate.sleep();
        ros::spinOnce(); 
      }          
    //}
    } */
/*     else {printf("here\n"); ros::spin();}
    //ros::spinOnce();
    return 0;
  }
  ros::spin();
} */


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "try");
  OpenManipulatorTeleop openManipulatorTeleop;
  char ch;
  int move = 0;
  openManipulatorTeleop.printText();


//---------publisher 선언------------//
ros::NodeHandle su;
ros::Publisher publisher = su.advertise<geometry_msgs::Twist>("Now_endEffector_Position", 10);
geometry_msgs::Twist endPose_d;
/////////////////////////////////////




/*  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    openManipulatorTeleop.printText();
    ros::spinOnce();
    openManipulatorTeleop.setGoal(ch); 
//init pose : 0.286, 0.000, 0.205  
//home pose : 0.134, 0.000, 0.241
    if(ch == 's')
    {       
      ros::Rate rate(10);
      while(ros::ok())
      {
        if(openManipulatorTeleop.getOpenManipulatorMovingState() == false) 
        {
          ROS_INFO("stopping");
          {
            move++; 
            switch(move)
            {
              case 1: printf("r send\n"); break;
              case 2: printf("h send\n"); break;
              case 3: openManipulatorTeleop.setGoal('1'); printf("1 send\n"); break; //배터리 위치까지 감 (0.125, 0.122, 0.060)
              case 4: openManipulatorTeleop.setGoal('2'); printf("2 send\n"); break; //엔드이펙터 각도조절
              case 5: openManipulatorTeleop.setGoal('9'); printf("9 send\n"); break; //내려가는거 (0.125, 0.122, 0.020)
              case 6: openManipulatorTeleop.setGoal('p'); printf("p send\n"); break; //오조준 (안씀)
              case 7: openManipulatorTeleop.setGoal('3'); printf("3 send\n"); break; //z방향으로 올라오는거 (0.125, 0.122, 0.060)
              case 8: openManipulatorTeleop.setGoal('v'); printf("v send\n"); break; //내려가기
              case 9: openManipulatorTeleop.setGoal('3'); printf("3 send\n"); break; //
              case 10: openManipulatorTeleop.setGoal('v'); printf("v send\n"); break; 
              case 12: openManipulatorTeleop.setGoal('3'); printf("3 send\n"); break;
              case 13: openManipulatorTeleop.setGoal('v'); printf("v send\n"); break;
              case 14: printf("finish\n"); break;
              default : printf("skip\n"); break;
            }
          }
        }
        else ROS_INFO("moving");

        if(move > 13) { move = 0; break; }
//----------------publish init 하는곳--------------//

         openManipulatorTeleop.initEndEffector();
         endPose_d.linear.x = endEffector_x;
         endPose_d.linear.y = endEffector_y;
         endPose_d.linear.z = endEffector_z;

         publisher.publish(endPose_d);    
        rate.sleep();
        ros::spinOnce(); 
      }          
    }
  }*/


  ros::Rate loop(100);
  //openManipulatorTeleop.setGoal();
/*
  while(openManipulatorTeleop.getOpenManipulatorMovingState())
  {
    ROS_INFO("MOVING");
    loop.sleep();
    ros::spinOnce();
  }
*/
  
  
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