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

#ifndef VESC_DRIVER__VESC_DRIVER_HPP_
#define VESC_DRIVER__VESC_DRIVER_HPP_

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state.hpp>

namespace vesc_driver
{

using std_msgs::msg::Float64;
using vesc_msgs::msg::VescState;

class VescDriver
: public autoware::drivers::serial_driver::SerialDriverNode<VescDriver, VescPacket, VescState>
{
public:
  VescDriver(const rclcpp::NodeOptions & options);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(
      const std::string& str,
      const boost::optional<double>& min_lower = boost::optional<double>(),
      const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };

  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  rclcpp::PublisherBase::SharedPtr state_pub_;
  rclcpp::PublisherBase::SharedPtr servo_sensor_pub_;
  rclcpp::SubscriptionBase::SharedPtr duty_cycle_sub_;
  rclcpp::SubscriptionBase::SharedPtr current_sub_;
  rclcpp::SubscriptionBase::SharedPtr brake_sub_;
  rclcpp::SubscriptionBase::SharedPtr speed_sub_;
  rclcpp::SubscriptionBase::SharedPtr position_sub_;
  rclcpp::SubscriptionBase::SharedPtr servo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // driver modes (possible states)
  typedef enum
  {
    MODE_INITIALIZING,
    MODE_OPERATING
  }
  driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  // ROS callbacks
  void brakeCallback(const Float64::SharedPtr & brake);
  void currentCallback(const Float64::SharedPtr & current);
  void dutyCycleCallback(const Float64::SharedPtr & duty_cycle);
  void positionCallback(const Float64::SharedPtr & position);
  void servoCallback(const Float64::SharedPtr & servo);
  void speedCallback(const Float64::SharedPtr & speed);
  void timerCallback();
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_DRIVER_HPP_
