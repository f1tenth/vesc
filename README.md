# Ray-Quasar: Veddar VESC Interface 

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## Ackermann to VESC Node has been reconfigured to publish braking messages.
This functionality existed in the ROS1 version of this package but was not included in the port to ROS2. I have added it back in, and improved it somewhat.

If you are an F1Tenth/Roboracer team using Traxxas' Velineon 3351R 3500 BLDC motor [this motor configuration XML](https://github.com/ray-quasar/vesc/blob/main/velineon3500_HFI.xml) will get you HFI for smoother starts as well. 

## How to test

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
