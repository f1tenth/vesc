# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

# Development Log
Haoru Xue 5/16/2021

## Completed
- Ported the VESC package to ROS2
- Sensor reading tested
- Speed and steerign commands tested

## Potential Improvements
- C++ 17 is currently required to replace `boost::optional` with `std::optional`
- Speed sensor reading is inverted
- Not made LifeCycleNode yet

## How to test
1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src` folder.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Check the device name, typically `/dev/ttyACM0`.
5. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
6. Build the package `colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug' --packages-up-to vesc`
7. `ros2 run vesc_driver vesc_driver_node --ros-args -r __node:=vesc_node -r __ns:=/ --params-file /root/ros2_ws/src/vesc/vesc_driver/params/vesc_config.yaml`