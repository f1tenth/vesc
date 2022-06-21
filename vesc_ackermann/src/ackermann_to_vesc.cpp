// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used
//    to endorse or promote products derived from this software without specific prior
//    written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.h"

#include <cmath>
#include <sstream>
#include <string>

#include <std_msgs/Float64.h>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, const std::string& name, T* value);

AckermannToVesc::AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // get conversion parameters
  if (!getRequiredParam(nh, "speed_to_erpm_gain", &speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", &speed_to_erpm_offset_))
    return;
  if (!getRequiredParam(nh, "accel_to_current_gain", &accel_to_current_gain_))
    return;
  if (!getRequiredParam(nh, "accel_to_brake_gain", &accel_to_brake_gain_))
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_gain", &steering_to_servo_gain_))
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_offset", &steering_to_servo_offset_))
    return;

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
  current_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/current", 10);
  brake_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/brake", 10);
  servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = nh.subscribe("ackermann_cmd", 10, &AckermannToVesc::ackermannCmdCallback, this);
}

typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;
void AckermannToVesc::ackermannCmdCallback(const AckermannMsgPtr& cmd)
{
  // calc vesc electric RPM (speed)
  std_msgs::Float64::Ptr erpm_msg(new std_msgs::Float64);
  erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc vesc current/brake (acceleration)
  bool is_positive_accel = true;
  std_msgs::Float64::Ptr current_msg(new std_msgs::Float64);
  std_msgs::Float64::Ptr brake_msg(new std_msgs::Float64);
  current_msg->data = 0;
  brake_msg->data = 0;
  if (cmd->drive.acceleration < 0)
  {
    brake_msg->data = accel_to_brake_gain_ * cmd->drive.acceleration;
    is_positive_accel = false;
  }
  else
  {
    current_msg->data = accel_to_current_gain_ * cmd->drive.acceleration;
  }

  // calc steering angle (servo)
  std_msgs::Float64::Ptr servo_msg(new std_msgs::Float64);
  servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (ros::ok())
  {
    // The below code attempts to stick to the previous mode until a new message forces a mode switch.
    if (erpm_msg->data != 0 || previous_mode_speed_)
    {
      erpm_pub_.publish(erpm_msg);
    }
    else
    {
      if (is_positive_accel)
      {
        current_pub_.publish(current_msg);
      }
      else
      {
        brake_pub_.publish(brake_msg);
      }
    }
    servo_pub_.publish(servo_msg);
  }

  // The lines below are to determine which mode we are in so we can hold until new messages force a switch.
  if (erpm_msg->data != 0)
  {
    previous_mode_speed_ = true;
  }
  else if (current_msg->data != 0 || brake_msg->data != 0)
  {
    previous_mode_speed_ = false;
  }
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, const std::string& name, T* value)
{
  if (nh.getParam(name, *value))
    return true;

  ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

}  // namespace vesc_ackermann
