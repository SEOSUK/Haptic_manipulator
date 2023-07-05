#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h> // ADD

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include <pthread.h>

float prev_time;
int calibrationStyle;
double Effort_Joint0 = 0; // ADD
double velocityX=0; // ADD
// ADD
double earlier_positionX = 0;
double later_positionX = 0;
double now_positionX = 0;
double velocity = 0;

double estimated_force_x = 0;
double estimated_force_y = 0;
double estimated_force_z = 0;

double estimated_force_x_LPF_f = 0;
double estimated_force_y_LPF_f = 0;
double estimated_force_z_LPF_f = 0;

double estimated_force_x_LPF_i = 0;
double estimated_force_y_LPF_i = 0;
double estimated_force_z_LPF_i = 0;

double WL = 0.1;

geometry_msgs::Twist haptic_tester;



struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
};

class PhantomROS {

public:
  ros::NodeHandle n;
  ros::Publisher state_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher button_publisher;
  ros::Publisher joint_publisher;
  ros::Publisher now_haptic_endEffector_publisher; // ADD - 현재 : test용
  ros::Publisher haptic_test_pub; // ADD
  ros::Subscriber haptic_sub;
  ros::Subscriber effort_sub; // ADD
  ros::Subscriber force_sub; // ADD
  std::string omni_name, ref_frame, units;

  OmniState *state;

  void init(OmniState *s) {
    ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));

    //Publish button state on NAME/button
    std::ostringstream stream1;
    stream1 << omni_name << "/button";
    std::string button_topic = std::string(stream1.str());
    button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100);

    //Publish on NAME/state
    std::ostringstream stream2;
    stream2 << omni_name << "/state";
    std::string state_topic_name = std::string(stream2.str());
    state_publisher = n.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1);

    haptic_test_pub = n.advertise<geometry_msgs::Twist>("/haptic_tester",10); // ADD

    //Subscribe to NAME/force_feedback
    std::ostringstream stream3;
    stream3 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

    //Publish on NAME/pose
    std::ostringstream stream4;
    stream4 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1);

    //Publish on NAME/joint_states
    std::ostringstream stream5;
    stream5 << omni_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);

    // ADD
    now_haptic_endEffector_publisher = n.advertise<geometry_msgs::Twist>("/now_haptic_endEffector_publisher",10);

    // ADD(subscribe manipulator effort)
    effort_sub = n.subscribe("/Effort_FromManipulator",10,&PhantomROS::effort_callback,this);

    // ADD
    force_sub = n.subscribe("/estimated_force",10,&PhantomROS::estimated_force_callback,this);

    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
    state->lock = false;
    state->close_gripper = false;
    state->lock_pos = zeros;
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHaNTOM position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/

  // ADD
  void effort_callback(const sensor_msgs::JointState& effortfeed) {
    Effort_Joint0 = effortfeed.effort.at(1);
   // ROS_INFO("effort_joint0 : %lf",Effort_Joint0);
    //geometry_msgs::Twist now_haptic; // ADD
    //now_haptic.linear.y = Effort_Joint0;
    //now_haptic_endEffector_publisher.publish(now_haptic); // ADD
  }

  // ADD
  void estimated_force_callback(const geometry_msgs::Twist& estimated_force)
  {
    ros::NodeHandle n;
    ros::Publisher haptic_tester_pub = n.advertise<geometry_msgs::Twist>("/haptic_tester", 10);

    if(state->close_gripper == false)
    {
      estimated_force_x = estimated_force.linear.x;
      estimated_force_y = estimated_force.linear.y;
      estimated_force_z = estimated_force.linear.z;
      ROS_WARN("without M,C,G");
    }
    else
    {
      estimated_force_x = estimated_force.angular.x;
      estimated_force_y = estimated_force.angular.y;
      estimated_force_z = estimated_force.angular.z;
      ROS_WARN("with M,C,G");
    }

    estimated_force_x_LPF_f = WL * estimated_force_x + (1 - WL) * estimated_force_x_LPF_i;
    estimated_force_y_LPF_f = WL * estimated_force_y + (1 - WL) * estimated_force_y_LPF_i;
    estimated_force_z_LPF_f = WL * estimated_force_z + (1 - WL) * estimated_force_z_LPF_i;

    //ROS_ERROR("F_x = %lf, F_y = %lf, F_z = %lf", estimated_force_x, estimated_force_y, estimated_force_z);
    //ROS_WARN("F_LPF_x = %lf, F_LPF_y = %lf, F_LPF_z = %lf", estimated_force_x_LPF_f, estimated_force_y_LPF_f, estimated_force_z_LPF_f);


  ////////////////////////////////////////////////////////////////////////////////////////////////
    estimated_force_x_LPF_i = estimated_force_x_LPF_f;
    estimated_force_y_LPF_i = estimated_force_y_LPF_f;
    estimated_force_z_LPF_i = estimated_force_z_LPF_f;

    haptic_tester.linear.x = estimated_force_x;
    haptic_tester.linear.y = estimated_force_y;
    haptic_tester.linear.z = estimated_force_z;
    haptic_tester.angular.x = -estimated_force_x_LPF_f/4;
    haptic_tester.angular.y = estimated_force_y_LPF_f/2;
    haptic_tester.angular.z = estimated_force_z_LPF_f;

    haptic_tester_pub.publish(haptic_tester);
  }

  void force_callback(const omni_msgs::OmniFeedbackConstPtr& omnifeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0]; // 0.001
    state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];
    velocityX = state->velocity[0]; // ADD
    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
  }

  double start_time = 0;
  
  void publish_omni_state() {
    // Build the state msg
    omni_msgs::OmniState state_msg;
    // Locked
    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;
    // Position
    
    //ADD
    earlier_positionX = now_positionX;
    state_msg.pose.position.x = -state->position[0]; // ADD(Change SIGN)
    now_positionX = state->position[0];
    later_positionX = now_positionX; 
    
    
    state_msg.pose.position.y = state->position[1];
    state_msg.pose.position.z = state->position[2];
    // Orientation
    start_time = ros::Time::now().toSec();
    state_msg.pose.orientation.x = start_time;
    state_msg.pose.orientation.y = state->rot.v()[1];
    state_msg.pose.orientation.z = state->rot.v()[2];
    state_msg.pose.orientation.w = state->rot.s();
    // Velocity
    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];

    // TODO: Append Current to the state msg
    state_msg.header.stamp = ros::Time::now();
    state_publisher.publish(state_msg);

    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = state->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = state->thetas[6] - M_PI; // -state->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = state->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = state->thetas[4] + M_PI; // state->thetas[6] - M_PI;
    joint_publisher.publish(joint_state);

    // Build the pose msg
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::Twist now_haptic; // ADD
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = ref_frame;
    pose_msg.pose = state_msg.pose;
    now_haptic.linear.x = state_msg.pose.position.x*10.0; // ADD
    now_haptic.linear.y = Effort_Joint0; // ADD
    now_haptic.linear.z = velocityX*100.0;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    pose_publisher.publish(pose_msg);
    now_haptic_endEffector_publisher.publish(now_haptic); // ADD

    if ((state->buttons[0] != state->buttons_prev[0])
        or (state->buttons[1] != state->buttons_prev[1]))
    {
      if (state->buttons[0] == 1) {
        state->close_gripper = !(state->close_gripper);
      }
      if (state->buttons[1] == 1) {
        state->lock = !(state->lock);
      }
      omni_msgs::OmniButtonEvent button_event;
      button_event.grey_button = state->buttons[0];
      button_event.white_button = state->buttons[1];
      state->buttons_prev[0] = state->buttons[0];
      state->buttons_prev[1] = state->buttons[1];
      button_publisher.publish(button_event);
    }
  }
};
HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  // Notice that we are inverting the Z-position value and changing Y <---> Z
  // Position
  omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  omni_state->position /= omni_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  // Force feedback
  //~ // Set forces if locked
  //~ if (omni_state->lock == true) {
    //~ omni_state->force = 0.04 * omni_state->units_ratio * (omni_state->lock_pos - omni_state->position)
        //~ - 0.001 * omni_state->velocity;
  //~ }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback

  //ROS_WARN("F_LPF_x = %lf, F_LPF_y = %lf, F_LPF_z = %lf", estimated_force_x_LPF_f, estimated_force_y_LPF_f, estimated_force_z_LPF_f);

  //feedback[0] = omni_state->force[0]; // -> 방향
  //feedback[1] = omni_state->force[2]; // | 아래 방향
  //feedback[2] = -omni_state->force[1]; // 햅틱 안으로 들어가는 방향

  // lock 버튼(아래 버튼) true/false에 따라 force feedback 유무
  if(omni_state->lock == false)
  {
    feedback[0] = estimated_force_y_LPF_f/2;
    feedback[1] = estimated_force_z_LPF_f;
    feedback[2] = -estimated_force_x_LPF_f;
    //ROS_WARN("force feedback unlocked");
  }
  else ROS_WARN("force feedback locked");

  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  //ROS_INFO("F0 = %lf, F1 = %lf, F2 = %lf", feedback[0],feedback[1],feedback[2]);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    ROS_INFO("Calibration complete.");
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      ROS_FATAL("Unknown calibration status");
  }
}

void *ros_publish(void *ptr) {
  PhantomROS *omni_ros = (PhantomROS *) ptr;
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 200);
  ROS_INFO("Publishing PHaNTOM state at [%d] Hz", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok()) {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
    return -1;
  }

  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to start the scheduler"); //, &error);
    return -1;
  }
  HHD_Auto_Calibration();

  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "omni_haptic_node");
  OmniState state;
  PhantomROS omni_ros;

  omni_ros.init(&state);
  hdScheduleAsynchronous(omni_state_callback, &state,
      HD_MAX_SCHEDULER_PRIORITY);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}