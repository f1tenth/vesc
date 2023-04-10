#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vesc_msgs/VescImuStamped.h"

namespace vesc_imu_parser
{
class ImuParser
{
public:
  ImuParser(ros::NodeHandle& nh);

private:
  ros::Publisher imu_data_pub;
  ros::Publisher imu_mag_pub;
  ros::Subscriber imu_data_sub;

  void imuDataCallback(const vesc_msgs::VescImuStamped& imu_msg);
};
}  // namespace vesc_imu_parser