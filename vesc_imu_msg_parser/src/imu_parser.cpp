#include "vesc_imu_msg_parser/imu_parser.hpp"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
// #include "vesc_msgs/VescImuStamped.h"
namespace vesc_imu_parser
{

ImuParser::ImuParser(ros::NodeHandle& nh)
{
  ImuParser::imu_data_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);
  ImuParser::imu_mag_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 10);
  ImuParser::imu_data_sub = nh.subscribe("/sensors/imu", 10, &ImuParser::imuDataCallback, this);
}

void ImuParser::imuDataCallback(const vesc_msgs::VescImuStamped& imu_msg)
{
  auto parsed_imu_msg = new sensor_msgs::Imu();
  auto parsed_imu_mag_msg = new sensor_msgs::MagneticField();

  parsed_imu_msg->header.frame_id = "imu";
  parsed_imu_mag_msg->header.frame_id = "imu";

  // imu data message
  parsed_imu_mag_msg->header.stamp = ros::Time::now();
  parsed_imu_msg->header.stamp = ros::Time::now();

  parsed_imu_msg->orientation.x = imu_msg.imu.orientation.x;
  parsed_imu_msg->orientation.y = imu_msg.imu.orientation.y;
  parsed_imu_msg->orientation.z = imu_msg.imu.orientation.z;
  parsed_imu_msg->orientation.w = imu_msg.imu.orientation.w;

  parsed_imu_msg->linear_acceleration.x = imu_msg.imu.linear_acceleration.x;
  parsed_imu_msg->linear_acceleration.y = imu_msg.imu.linear_acceleration.y;
  parsed_imu_msg->linear_acceleration.z = imu_msg.imu.linear_acceleration.z;

  parsed_imu_msg->angular_velocity.x = imu_msg.imu.angular_velocity.x;
  parsed_imu_msg->angular_velocity.y = imu_msg.imu.angular_velocity.y;
  parsed_imu_msg->angular_velocity.z = imu_msg.imu.angular_velocity.z;

  // magnetic field message
  parsed_imu_mag_msg->magnetic_field.x = imu_msg.imu.compass.x;
  parsed_imu_mag_msg->magnetic_field.y = imu_msg.imu.compass.y;
  parsed_imu_mag_msg->magnetic_field.z = imu_msg.imu.compass.z;

  imu_data_pub.publish(*parsed_imu_msg);
  imu_mag_pub.publish(*parsed_imu_mag_msg);
}

}  // namespace vesc_imu_parser

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_imu_msg_parser_node");
  ROS_INFO("VESC IMU Message Parser Node Started");
  ros::NodeHandle n;

  vesc_imu_parser::ImuParser obj = vesc_imu_parser::ImuParser(n);

  ros::Rate loop_rate(60);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}