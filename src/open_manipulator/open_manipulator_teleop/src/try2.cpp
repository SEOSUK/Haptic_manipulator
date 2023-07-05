#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"
#include "open_manipulator_msgs/DockBatteryService.h"
#include "open_manipulator_msgs/ManipulatorService.h"

double endEffector_x = 0;
double endEffector_y = 0;
double endEffector_z = 0;


char ch;


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
  goal_drawing_trajectory_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("goal_drawing_trajectory"); // ADD
  DockBatteryClient = node_handle_.serviceClient<open_manipulator_msgs::DockBatteryService>("/DockBatteryService"); // ADD 
}

void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &OpenManipulatorTeleop::manipulatorStatesCallback, this);
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
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

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
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
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
// open_manipulator_msgs::DockService Service; 
// Service.request.Dock_Do = true;
// DockClient.call(Service);
// ros::spinOnce();

void OpenManipulatorTeleop::initEndEffector()
{
  endEffector_x = getPresentKinematicsPose().at(0);
  endEffector_y = getPresentKinematicsPose().at(1);
  endEffector_z = getPresentKinematicsPose().at(2);
}

void OpenManipulatorTeleop::setGoal(char ch)
{
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
    joint_name.push_back("joint2"); joint_angle.push_back(0.150);//0.150 //0.031 horizontal
    joint_name.push_back("joint3"); joint_angle.push_back(-0.038);
    joint_name.push_back("joint4"); joint_angle.push_back(1.454);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '2' && open_manipulator_is_moving_== false)
  {
    printf("input : 2 \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.023);
    //Service.request.Dock_Do = true;
    //DockBatteryClient.call(Service);
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
    joint_name.push_back("joint1"); joint_angle.push_back(0.822);//0.822
    joint_name.push_back("joint2"); joint_angle.push_back(0.031);//0.150 //0.031 horizontal
    joint_name.push_back("joint3"); joint_angle.push_back(-0.038);
    joint_name.push_back("joint4"); joint_angle.push_back(1.454);
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
  else if (ch == 'a' && open_manipulator_is_moving_== false)
  {
    printf("input : a \t111111111111111\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.779);
    joint_name.push_back("joint2"); joint_angle.push_back(0.084);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.028);
    joint_name.push_back("joint4"); joint_angle.push_back(1.465);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'b' && open_manipulator_is_moving_== false)
  {
    printf("input : b \t2222222222222\n");

    std::string name; name = "line";
    std::vector<double> arg;
    double path_time = 2.0;
    arg.push_back(0);
    arg.push_back(0);
    arg.push_back(-0.07);
    setDrawingTrajectory(name, arg, path_time);
  }
  else if (ch == 'c' && open_manipulator_is_moving_== false)
  {
    printf("input : c \t33333333333333\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.781);
    joint_name.push_back("joint2"); joint_angle.push_back(0.025);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.233);
    joint_name.push_back("joint4"); joint_angle.push_back(1.608);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'd' && open_manipulator_is_moving_== false)
  {
    printf("input : d \t44444444444444\n");

    std::string name; name = "line";
    std::vector<double> arg;
    double path_time = 2.0;
    arg.push_back(0);
    arg.push_back(0);
    arg.push_back(0.01);
    setDrawingTrajectory(name, arg, path_time);
  }
  else if (ch == 'e' && open_manipulator_is_moving_== false)
  {
    printf("input : e \t5555555555555\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.822);
    joint_name.push_back("joint2"); joint_angle.push_back(0.192);//0.080
    joint_name.push_back("joint3"); joint_angle.push_back(-0.368);
    joint_name.push_back("joint4"); joint_angle.push_back(1.782);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'f' && open_manipulator_is_moving_== false)
  {
    printf("input : f \tdfghjkl;\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.822);
    joint_name.push_back("joint2"); joint_angle.push_back(0.080);//0.080
    joint_name.push_back("joint3"); joint_angle.push_back(-0.038);
    joint_name.push_back("joint4"); joint_angle.push_back(1.454);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'g' && open_manipulator_is_moving_== false)
  {
    printf("input : g \tkjhgfdxgh\n");

    std::vector<double> joint_angle;
    joint_angle.push_back(-0.0465);
    Service.request.Dock_Do = true;
    DockBatteryClient.call(Service);
    setToolControl(joint_angle);
  }
  else if (ch == 'j' && open_manipulator_is_moving_== false)
  {
    printf("input : j \tkjhgfdxgh\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(2.321);
    joint_name.push_back("joint2"); joint_angle.push_back(0.125);//0.080
    joint_name.push_back("joint3"); joint_angle.push_back(0.066);
    joint_name.push_back("joint4"); joint_angle.push_back(1.335);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'l' && open_manipulator_is_moving_== false)
  {
    printf("input : l \tkjhgfdxgh\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(2.338);
    joint_name.push_back("joint2"); joint_angle.push_back(0.261);//0.080
    joint_name.push_back("joint3"); joint_angle.push_back(0.239);
    joint_name.push_back("joint4"); joint_angle.push_back(1.049);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'm' && open_manipulator_is_moving_== false)
  {
    printf("input : m \t111111111111111\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 5.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.779);
    joint_name.push_back("joint2"); joint_angle.push_back(0.103);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.178);
    joint_name.push_back("joint4"); joint_angle.push_back(1.626);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == 'z' && open_manipulator_is_moving_== false)
  {
    printf("input : z \tdocking\n");

    Service.request.Dock_Do = true;
    DockBatteryClient.call(Service);
  }
  else if (ch == 'x' && open_manipulator_is_moving_== false)
  {
    printf("input : x \tundocking\n");

    Service.request.Dock_Do = false;
    DockBatteryClient.call(Service);
  }
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
  ros::init(argc, argv, "try");
  OpenManipulatorTeleop openManipulatorTeleop;

  ros::NodeHandle su;
  ros::Publisher publisher = su.advertise<geometry_msgs::Twist>("Now_endEffector_Pose",100);
  geometry_msgs::Twist endPose_d;

  char ch;
  int move = 0;
  openManipulatorTeleop.printText();

  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    openManipulatorTeleop.printText();
    ros::spinOnce();
    openManipulatorTeleop.setGoal(ch);

    



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
              case 3: openManipulatorTeleop.setGoal('1'); printf("1 send\n"); break;
              case 4: openManipulatorTeleop.setGoal('2'); printf("2 send\n"); break;
              case 5: openManipulatorTeleop.setGoal('9'); printf("9 send\n"); break;
              case 6: openManipulatorTeleop.setGoal('p'); printf("p send\n"); break;
              case 7: openManipulatorTeleop.setGoal('3'); printf("3 send\n"); break;
              case 8: openManipulatorTeleop.setGoal('3'); printf("3 send\n"); break;
              case 9: openManipulatorTeleop.setGoal('1'); printf("3 send\n"); break;
              case 10: openManipulatorTeleop.setGoal('9'); printf("9 send\n"); break;
              case 11: printf("finish\n"); break;
              default : printf("skip\n"); break;
            }
          }
        }
        else 
        {
          ROS_INFO("moving");
          openManipulatorTeleop.initEndEffector();
          endPose_d.linear.x = endEffector_x;
          endPose_d.linear.y = endEffector_y;
          endPose_d.linear.z = endEffector_z;

          publisher.publish(endPose_d);
          ros::spinOnce();
        }
        if(move > 10) { move = 0; break; }

        rate.sleep();
        ros::spinOnce(); 
      }          
    }
  }
  return 0;
}

