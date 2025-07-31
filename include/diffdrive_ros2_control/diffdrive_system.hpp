// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIFFDRIVE_ROS2_CONTROL__DIFFDRIVE_SYSTEM_HPP_
#define DIFFDRIVE_ROS2_CONTROL__DIFFDRIVE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffdrive_ros2_control/visibility_control.h"
#include "diffdrive_ros2_control/base_comms.hpp"

namespace diffdrive_ros2_control
{
  class DiffDriveSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    struct Config
    {
      std::string device_name = "";
      std::string port = "";
      int baud_rate = 0;
      int timeout_ms = 0;
      float loop_rate = 0.0;
    };

    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveSystemHardware)

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // serial communication obj
    BaseComm *comm_;
    Config cfg_;

    // these will hold the rpm converted from cmd_vel
    double left_wheel_cmd_rpm_;
    double right_wheel_cmd_rpm_;

    // these will hold the arbiterate values of encoder rpm per loop rate,
    // which is used to calculate odom if position_feedback param is true
    double left_hw_positions_;
    double right_hw_positions_;
    
    // these hold the rpm from motor encoder used to calc odom when
    // position_feedback param is false
    double left_encoder_rpm_;
    double right_encoder_rpm_;
  };

} // namespace diffdrive_ros2_control

#endif // DIFFDRIVE_ROS2_CONTROL__DIFFDRIVE_SYSTEM_HPP_
