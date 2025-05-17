#pragma once
#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// for gait switching
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"

// constant define
// joy stick command interprate
#define JOY_CMD_BODY_HEIGHT_MAX 0.32  // m
#define JOY_CMD_BODY_HEIGHT_MIN 0.1   // m
#define JOY_CMD_BODY_HEIGHT_VEL 0.04  // m/s
#define JOY_CMD_VELX_MAX 1.0          // m/s
#define JOY_CMD_VELY_MAX 0.5          // m/s
#define JOY_CMD_YAW_MAX 0.9           // rad
#define JOY_CMD_PITCH_MAX 0.4         // rad
#define JOY_CMD_ROLL_MAX 0.4          // rad

#define JOY_STATE_TOTAL_STATES 3
// state list
// follow gait.info
// 0 stance             (default)
// 1 trot
// 2 flying_trot

class ROSJoyProcessor {
 public:
  ROSJoyProcessor(ros::NodeHandle& _nh, const std::string& gaitFile, const std::string& robotName) {
    nh = _nh;
    sub_joy_msg = nh.subscribe("/joy", 1000, &ROSJoyProcessor::joy_callback, this);

    //
    switch_ctrl_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    // joystick mapping
    _nh.param("/joystick_left_updown_axis", joystick_left_updown_axis, 1);
    _nh.param("/joystick_left_horiz_axis", joystick_left_horiz_axis, 0);
    _nh.param("/joystick_right_updown_axis", joystick_right_updown_axis, 4);
    _nh.param("/joystick_right_horiz_axis", joystick_right_horiz_axis, 3);
    _nh.param("/joystick_mode_switch_button", joystick_mode_switch_button, 0);
    _nh.param("/joystick_start_button", joystick_start_button, 4);
    _nh.param("/joystick_stop_button", joystick_stop_button, 5);

    // joystick parameters
    _nh.param("/joystick_velx_scale", joystick_velx_scale, 1.5);
    _nh.param("/joystick_vely_scale", joystick_vely_scale, 0.4);
    _nh.param("/joystick_height_vel", joystick_height_vel, 0.1);
    _nh.param("/joystick_max_height", joystick_max_height, 0.3);
    _nh.param("/joystick_min_height", joystick_min_height, 0.03);
    _nh.param("/joystick_yaw_rate_scale", joystick_yaw_rate_scale, 0.8);
    _nh.param("/joystick_roll_rate_scale", joystick_roll_rate_scale, 0.4);
    _nh.param("/joystick_pitch_rate_scale", joystick_pitch_rate_scale, 0.4);

    _nh.param("/joystick_smooth_coeffi", joystick_smooth_coeffi, 0.8);

    ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, false);
    gaitMap_.clear();
    for (const auto& gaitName : gaitList_) {
      gaitMap_.insert({gaitName, ocs2::legged_robot::loadModeSequenceTemplate(gaitFile, gaitName, false)});
    }
    modeSequenceTemplatePublisher_ = nh.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);
  }

  ~ROSJoyProcessor() {
    // destruct = true;
    // thread_.join();
  }

  // bool isExit()
  // {
  //     return joy_cmd_exit;
  // }

  // this should be called in the control loop
  void processJoy(double dt) {
    if (joy_cmd_ctrl_state_change_request) {
      // toggle joy_cmd_ctrl_state
      joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
      joy_cmd_ctrl_state = joy_cmd_ctrl_state % JOY_STATE_TOTAL_STATES;  // TODO: how to toggle more states?
      joy_cmd_ctrl_state_change_request = false;                         // erase this change request;
      // publish one gait switch command
      std::string gaitCommand = "stance";
      if (joy_cmd_ctrl_state == 0) {
        gaitCommand = "stance";
      } else if (joy_cmd_ctrl_state == 1) {
        gaitCommand = "trot";
      } else if (joy_cmd_ctrl_state == 2) {
        gaitCommand = "flying_trot";
      } else {
        gaitCommand = "stance";
      }

      ocs2::legged_robot::ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
      modeSequenceTemplatePublisher_.publish(ocs2::legged_robot::createModeSequenceTemplateMsg(modeSequenceTemplate));
    }
    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;
  }

  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    // A - mode switch
    if (joy_msg->buttons[joystick_mode_switch_button] == 1) {
      joy_cmd_ctrl_state_change_request = true;
    }

    // right updown
    double tmp = joy_msg->axes[joystick_right_updown_axis] * joystick_velx_scale;
    joy_cmd_velx = joystick_smooth_coeffi * joy_cmd_velx + (1.0 - joystick_smooth_coeffi) * tmp;
    // right horiz
    tmp = joy_msg->axes[joystick_right_horiz_axis] * joystick_vely_scale;
    joy_cmd_vely = joystick_smooth_coeffi * joy_cmd_vely + (1.0 - joystick_smooth_coeffi) * tmp;
    // left updown
    joy_cmd_velz = joy_msg->axes[joystick_left_updown_axis] * joystick_height_vel;
    // left horiz
    joy_cmd_yaw_rate = joy_msg->axes[joystick_left_horiz_axis] * joystick_yaw_rate_scale;

    // send controller start and stop command
    if (joy_msg->buttons[joystick_start_button] == 1) {
      std::cout << "You have pressed the start button!!!!" << std::endl;
      // set the string start_controllers to controllers/legged_controller
      switch_ctrl_srv.request.start_controllers = {"controllers/legged_controller"};
      switch_ctrl_srv.request.stop_controllers = {""};
      switch_ctrl_srv.request.strictness = switch_ctrl_srv.request.BEST_EFFORT;
      switch_ctrl_srv.request.start_asap = true;
      switch_ctrl_srv.request.timeout = 0.0;
      switch_ctrl_client.call(switch_ctrl_srv);
    }
    if (joy_msg->buttons[joystick_stop_button] == 1) {
      std::cout << "You have pressed the stop button!!!!" << std::endl;
      // set the string stop_controllers to controllers/legged_controller
      switch_ctrl_srv.request.start_controllers = {""};
      switch_ctrl_srv.request.stop_controllers = {"controllers/legged_controller"};
      switch_ctrl_srv.request.strictness = switch_ctrl_srv.request.BEST_EFFORT;
      switch_ctrl_srv.request.start_asap = true;
      switch_ctrl_srv.request.timeout = 0.0;
      switch_ctrl_client.call(switch_ctrl_srv);
    }
  }

  // public joystick command
  double joy_cmd_velx = 0.0;
  double joy_cmd_vely = 0.0;
  double joy_cmd_velz = 0.0;
  double joy_cmd_body_height = 0.28;  // unitree Go1 default height
  double joy_cmd_yaw_rate = 0.0;

  int joy_cmd_ctrl_state = 0;

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub_joy_msg;
  ros::ServiceClient switch_ctrl_client;
  controller_manager_msgs::SwitchController switch_ctrl_srv;

  // private joystick command, intermediate
  double joy_cmd_roll_rate = 0.0;
  double joy_cmd_pitch_rate = 0.0;
  double joy_cmd_pitch_ang = 0.0;
  double joy_cmd_roll_ang = 0.0;

  bool joy_cmd_ctrl_state_change_request = false;
  int prev_joy_cmd_ctrl_state = 0;
  // bool joy_cmd_exit = false;

  // for switching gait mode
  ros::Publisher modeSequenceTemplatePublisher_;
  std::vector<std::string> gaitList_;
  std::map<std::string, ocs2::legged_robot::ModeSequenceTemplate> gaitMap_;

  // joystick mapping
  int joystick_left_updown_axis;
  int joystick_left_horiz_axis;
  int joystick_right_updown_axis;
  int joystick_right_horiz_axis;
  int joystick_mode_switch_button;
  int joystick_start_button;
  int joystick_stop_button;

  // joystick parameters
  double joystick_velx_scale;
  double joystick_vely_scale;
  double joystick_height_vel;
  double joystick_max_height;
  double joystick_min_height;

  double joystick_yaw_rate_scale;
  double joystick_roll_rate_scale;
  double joystick_pitch_rate_scale;

  double joystick_smooth_coeffi;
};