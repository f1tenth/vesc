// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  u_int8_t operation_mode_;
  // bool previous_mode_speed_ = true;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double speed_to_braking_gain_, speed_to_braking_center_, speed_to_braking_max_, speed_to_braking_min_;  
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double current_vel_, brake_deadzone_;
  double accel_to_current_gain_, accel_to_brake_gain_;

  enum operation_modes {
    ACCEL_TO_CURRENT,
    VEL_TO_CURRENT,
    VEL_TO_ERPM
  };

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<Float64>::SharedPtr brake_pub_;
  rclcpp::Publisher<Float64>::SharedPtr current_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
