# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

# Development Log
Haoru Xue 8/6/2021

Completed:

- Fall back to C++ 14
- Enabled error handler


Haoru Xue 5/16/2021

Completed:

- Ported the VESC package to ROS2
- Sensor reading tested
- Speed and steerign commands tested

Potential Improvements:

- C++ 17 is currently required to replace `boost::optional` with `std::optional`
- Speed sensor reading is inverted
- Not made LifeCycleNode yet

## How to test

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the package `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
