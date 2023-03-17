^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vesc_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2023-03-17)
------------------
* Expose VESC config from VESC driver launch file.
* Fix issue with if statements.
* Merge pull request `#20 <https://github.com/f1tenth/vesc/issues/20>`_ from Triton-AI/ros2
  [Bug Fix] Use synchronous serial read
* reduce print severity to debug
* Merge pull request `#19 <https://github.com/f1tenth/vesc/issues/19>`_ from anscipione/add_imu_support_rebase_v2
  Add imu support and vesc udev enumeration tools (rebase upstream ros2) v2
* fixed some test issues
* publish sensors_msgs::Imu
* replaced  boost::bind with std::bind
* fix sensor
* fixed timing wait for imu polling
* added imu polling
* fixed test for udev
* added script for udev
* added program to enumerate vesc serial ports based on their uuid
* Fix linter errors
* Merge pull request `#14 <https://github.com/f1tenth/vesc/issues/14>`_ from Triton-AI/ros2
  Pull ros2 experimental vesc driver back into f1tenth
* remove serial driver from cmake
* removed debug codes from 654ba11
* update main branch changes
* update cmake
* Merge branch 'ros2' of https://github.com/Triton-AI/vesc into ros2
* node naving convention; removed unused variable
* rebased to upstream
* working ros2 driver
* removed old methods
* fixed test results
* fix test results
* protocol update to FW 5.2
* resolved inbound serial comunication problems
* removed old error handler
* update readme
* modified launch file
* std::optional -> std::experimental::optional
* force c++17
* minor coding practice updates
* Merge pull request `#12 <https://github.com/f1tenth/vesc/issues/12>`_ from anscipione/protocol_update_to_FW_5.2
  Protocol update to fw 5.2
* Merge pull request `#11 <https://github.com/f1tenth/vesc/issues/11>`_ from anscipione/Ros2_bug_fixing_serial_comunication
  resolved inbound serial comunication problems
* Merge remote-tracking branch 'origin/Ros2_bug_fixing_serial_comunication' into protocol_update_to_FW_5.2
* removed commented-out code
* removed old methods
* fixed test results
* Merge branch 'Ros2_bug_fixing_serial_comunication' into protocol_update_to_FW_5.2
* fix test results
* protocol update to FW 5.2
* resolved inbound serial comunication problems
* working ros2 driver
* [vesc_driver] Optimizing mutex utilization.
* [vesc_driver] Changing default port to F1TENTH symlink name.
* Add mutex and wait in read thread.
* [vesc_driver] Re-adding missing join() for thread.
* [vesc_driver] Adding all optional params to launch.
* [vesc_driver] Set launch file to typical device for F1TENTH.
* [vesc_driver] Node finally starts.
* [vesc_driver] Fixing copyrights and finishing transition to boost::asio.
* [vesc_driver] Finished port - applied uncrustify.
* [vesc_driver] Working on implementation of rclcpp::Node.
* [vesc_driver] Starting port of vesc_driver.
* [vesc_driver] Removing unnecessary files and renaming header files.
* [vesc_driver] Removing nodelet XML and renaming header files.
* Porting all package.xml and CMakeList.txt files to ROS2.
* Contributors: Alessandro Celeste, Andrea Scipione, Haoru, Haoru Xue, Hongrui (Billy) Zheng, Joshua Whitley

1.1.0 (2020-12-12)
------------------
* Merge pull request `#1 <https://github.com/f1tenth/vesc/issues/1>`_ from f1tenth/melodic-devel
  Updating for Melodic
* Exclude crc.h from roslint.
* Replacing boost::crc with CRCPP.
* Replacing boost::begin, boost::end, and boost::distance with Standard Library equivalents.
* Replacing boost::bind with Standard Library equivalent.
* Replaing boost::noncopyable with C++ equivalent.
* Replacing boost::function with Standard Library version.
* Replacing Boost smart pointers with Standard Library equivalents.
* Removing unnecessary v8stdint.h.
* Updating package.xml to format 3 and setting C++ standard to 11.
* Contributors: Joshua Whitley

1.0.0 (2020-12-02)
------------------
* Applying roslint and replacing registration macro with templated class.
* Adding roslint.
* Adding licenses.
* Updating maintainers, authors, and URLs.
* added onboard car
* Contributors: Joshua Whitley, billyzheng
