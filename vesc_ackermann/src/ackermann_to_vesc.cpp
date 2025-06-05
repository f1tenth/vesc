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

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>	

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

	using ackermann_msgs::msg::AckermannDriveStamped;
	using nav_msgs::msg::Odometry;
	using std::placeholders::_1;
	using std_msgs::msg::Float64;

	AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions &options)
			: Node("ackermann_to_vesc_node", options)
	{
		// Declare parameters, initialized with default 0
		// These four are what we started with:
		declare_parameter("speed_to_erpm_gain", 0.0);
		declare_parameter("speed_to_erpm_offset", 0.0);
		declare_parameter("steering_angle_to_servo_gain", 0.0);
		declare_parameter("steering_angle_to_servo_offset", 0.0);

		// I added this one (originally just a regular variable) as part of the first braking test:
		// Renaming to brake_deadzone
		declare_parameter("brake_deadzone", 0.1);

		// Adding braking parameters for VEL_TO_ERPM mode
		declare_parameter("speed_to_braking_gain", 0.0);
		declare_parameter("speed_to_braking_center", 0.0);
		declare_parameter("speed_to_braking_max", 20000.0);
		declare_parameter("speed_to_braking_min", 0.0);

		// I am adding these two in as part of the conversion to acceleration based control:
		declare_parameter("accel_to_current_gain", 0.0);
		declare_parameter("accel_to_brake_gain", 0.0);

		// Get conversion parameters from config file, in our case 'vesc.yaml'
		// Originals:
		speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
		speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
		steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
		steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();

		// Braking Test:
		brake_deadzone_ = get_parameter("brake_deadzone").get_value<double>();

		// Braking parameters for VEL_TO_ERPM mode
		speed_to_braking_gain_ = get_parameter("speed_to_braking_gain").get_value<double>();
		speed_to_braking_center_ = get_parameter("speed_to_braking_center").get_value<double>();
		speed_to_braking_max_ = get_parameter("speed_to_braking_max").get_value<double>();
		speed_to_braking_min_ = get_parameter("speed_to_braking_min").get_value<double>();

		// Acceleration Test:
		accel_to_current_gain_ = get_parameter("accel_to_current_gain").get_value<double>();
		accel_to_brake_gain_ = get_parameter("accel_to_brake_gain").get_value<double>();

		// Create publishers to VESC electric-RPM (speed) and servo commands
		// The ERPM publisher should only be used if the acceleration parameter is 0
		erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
		servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

		// Create brake publisher
		brake_pub_ = create_publisher<Float64>("commands/motor/brake", 10);
		// Initialize current velocity storage variable
		current_vel_ = 0.0;

		// Create current publisher
		current_pub_ = create_publisher<Float64>("commands/motor/current", 10);

		// Subscribe to Ackermann topic
		// This is what supplies the input information for the rest of the code.
		// I don't care to change the disparityExtender code, so we'll just work with the speed commands therein.
		ackermann_sub_ = create_subscription<AckermannDriveStamped>(
				"ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

		// Subscribe to odom topic
		// Because we are just listening to the speed commands, we need a tracker of the existing velocity
		odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
				"odom", 10, std::bind(&AckermannToVesc::odomCallback, this, _1));
	}

	void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
	{
		// Initialize all messages at the start
		Float64 servo_msg;
		Float64 brake_msg;
		Float64 current_msg;
		Float64 erpm_msg;
		bool publish_brake = false;
		bool publish_ERPM = false;

		// Zero-initialize message data
		brake_msg.data = 0.0;
		current_msg.data = 0.0;
		erpm_msg.data = 0.0;

		// Calculate steering angle (servo)
		servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

		// Calculate BLDC command
		// Case 1: We are supplied the acceleration-to-current gain and acceleration-to-brake gain
		if (accel_to_current_gain_ != 0 && accel_to_brake_gain_ != 0)
		{
			bool is_positive_accel = true;
			// 1.1 We have an incoming acceleration command. 
			if (cmd->drive.acceleration != 0)
			{
				operation_mode_ = ACCEL_TO_CURRENT; // For deciding which message to publish
				if (cmd->drive.acceleration < 0)
				{
					// Apply controlled braking
					brake_msg.data = accel_to_brake_gain_ * cmd->drive.acceleration;
					// is_positive_accel = false;
				} else
				{
					// Apply motor current (throttle)
					current_msg.data = accel_to_current_gain_ * cmd->drive.acceleration;
				}
				// previous_mode_speed_ = false;
			}
			// 1.2 We do not have an incoming acceleration command. We have an incoming velocity command.
			else if (cmd->drive.speed != 0 || operation_mode_ == VEL_TO_CURRENT) {
				// Calculate the required acceleration:
				// TODO: Calculate dt 
				operation_mode_ = VEL_TO_CURRENT; // For deciding which message to publish
				double commanded_vel = cmd->drive.speed;
				double acceleration = 10 * (commanded_vel - current_vel_); // . / 0.1;  // Desired acceleration
				if (acceleration > 0) 
				{
					// Apply motor current (throttle)
					current_msg.data = acceleration * accel_to_current_gain_;
				} else 
				{
					// Apply controlled braking
					brake_msg.data = -acceleration * accel_to_brake_gain_;
					// is_positive_accel = false;
				}
			}
		}
		// Case 2: If the acceleration-to-current gain and acceleration-to-brake gain is 0, then we operate entirely with the velocity message
		else if (accel_to_current_gain_ == 0 && accel_to_brake_gain_ == 0)
		{
			operation_mode_ = VEL_TO_ERPM; // For deciding which message to publish
			double commanded_vel = cmd->drive.speed;

			// Case A: We have a postive commanded velocity:
			if (commanded_vel >= 0)
			{
				// I. The commanded velocity is greater than the current velocity
				if (commanded_vel >= current_vel_)
				{
					// If the current velocity is negative apply the brake
					if (current_vel_ < 0)
					{
						// Calculate the brake value
						brake_msg.data = (
							(speed_to_braking_max_) 
							/ (1 + exp(- speed_to_braking_gain_ * (abs(current_vel_ - commanded_vel) - speed_to_braking_center_))) 
							);
						publish_brake = true;
					}
					// If the current velocity is zero we need to go slow to get a rotor position reading before we can accelerate
					else if (current_vel_ < 1 && commanded_vel > 1)
					{
						// Send a (0.5 m/s) ERPM message
						// Calculate the ERPM using the speed-to-ERPM gain and offset:
						erpm_msg.data = speed_to_erpm_gain_ * (current_vel_ + 0.4) + speed_to_erpm_offset_;
							// Hopefully the compiler will pick this up and pre-evaluate
						publish_ERPM = true;
					}
					// The current velocity is greater than zero
					else
					{
						// Calculate the ERPM using the speed-to-ERPM gain and offset:
						erpm_msg.data = speed_to_erpm_gain_ * commanded_vel + speed_to_erpm_offset_;
						publish_ERPM = true;
					}
				}
				// II. The commanded velocity is less than the current velocity
				else if (commanded_vel < current_vel_)
				{
					// Apply the BRAKE
					brake_msg.data = (
						(speed_to_braking_max_) 
						/ (1 + exp(- speed_to_braking_gain_ * (abs(current_vel_ - commanded_vel) - speed_to_braking_center_))) 
						);
					// is_positive_accel = false;
					publish_brake = true;
				}
			}					

			// Case B: We have a negative commanded velocity:
			else if (commanded_vel < 0)
			{
				// I. The commanded velocity is less (faster) than the current velocity
				if (commanded_vel <= current_vel_)
				{
					// If the current velocity is positive
					if (current_vel_ > 0)
					{
						// Apply the brake
						brake_msg.data = (
							(speed_to_braking_max_) 
							/ (1 + exp(- speed_to_braking_gain_ * (abs(current_vel_ - commanded_vel) - speed_to_braking_center_))) 
							);
						publish_brake = true;
					}
					// If the current velocity is zero we need to go slow to get a rotor position reading
					else if (current_vel_ == 0 && commanded_vel < -0.5)
					{
						// Send a (-0.5 m/s) ERPM message
						erpm_msg.data = speed_to_erpm_gain_ * -0.5 + speed_to_erpm_offset_;
							// Hopefully the compiler will pick this up and pre-evaluate
						publish_ERPM = true;
					}
					// The current velocity is negative
					else 
					{
						// Calculate the ERPM using the speed-to-ERPM gain and offset:
						erpm_msg.data = speed_to_erpm_gain_ * commanded_vel + speed_to_erpm_offset_;
						publish_ERPM = true;
					}
				}
				// II. The commanded velocity is greater (slower) than the current velocity
				else if (commanded_vel > current_vel_)
				{
					// Apply the BRAKE
					// Apply the BRAKE
					brake_msg.data = (
						(speed_to_braking_max_) 
						/ (1 + exp(- speed_to_braking_gain_ * (abs(current_vel_ - commanded_vel) - speed_to_braking_center_))) 
						);
					// is_positive_accel = false;
					publish_brake = true;
				}
			}
		}
			// ----- OLD CODE ------

		// 	double vel_diff = current_vel_ - commanded_vel;
		// 	// 2.1 The commanded velocity has increased:
		// 	if (vel_diff < 0) 
		// 	{
		// 		// Calculate the ERPM using the speed-to-ERPM gain and offset:
		// 		erpm_msg.data = speed_to_erpm_gain_ * commanded_vel + speed_to_erpm_offset_;
		// 	}
		// 	// 2.2 The commanded velocity has decreased:
		// 	if (vel_diff > 0) 
		// 	{
		// 		// Calculate the brake value:
		// 		// brake_msg.data = (vel_diff) * speed_to_braking_gain_ + speed_to_braking_center_;
        // 		brake_msg.data = (
        //         (speed_to_braking_max_) 
        //         / (1 + exp(- speed_to_braking_gain_ * (vel_diff - speed_to_braking_center_))) 
      	// 		);
		// 		// brake_msg.data = std::clamp(brake_msg.data, 0.0, 20000.0);
		// 		is_positive_accel = false;
		// 	}
		// }

		// Publish motor commands:
		if (rclcpp::ok()) {
			if (publish_brake) {
				if (brake_msg.data != 0.0) {  // Only publish if we actually set a brake value
					brake_pub_->publish(brake_msg);
				}
			} else if (publish_ERPM) {
				if (operation_mode_ == ACCEL_TO_CURRENT || operation_mode_ == VEL_TO_CURRENT) {
					if (current_msg.data != 0.0) {
						current_pub_->publish(current_msg);
					}
				} else if (operation_mode_ == VEL_TO_ERPM) {
					if (erpm_msg.data != 0.0) {
						erpm_pub_->publish(erpm_msg);
					}
				}
			}
			servo_pub_->publish(servo_msg);
		}
	}


	void AckermannToVesc::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
	{
		// Everytime there is a new publish to /odom, we update the current velocity
		current_vel_ = odom_msg->twist.twist.linear.x;
		// RCLCPP_INFO(get_logger(), "Current velocity: %f", current_vel_);
	}

} // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp" // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
