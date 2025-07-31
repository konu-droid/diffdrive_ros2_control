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

#include "diffdrive_ros2_control/diffdrive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Custom motor controller libs
#include "diffdrive_ros2_control/roboteq_comms.hpp"
#include "diffdrive_ros2_control/moons_comms.hpp"
#include "diffdrive_ros2_control/zlac8015_comms.hpp"

namespace diffdrive_ros2_control
{
  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.device_name = info_.hardware_parameters["device_name"];
    cfg_.port = info_.hardware_parameters["port"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffDriveSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_hw_positions_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_encoder_rpm_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_hw_positions_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_encoder_rpm_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_cmd_rpm_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_cmd_rpm_));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Configuring ...please wait...");

    if (cfg_.device_name == "roboteq")
      comm_ = new RoboteqComm();
    else if (cfg_.device_name == "moons")
      comm_ = new MoonsComm();
    else if (cfg_.device_name == "zlac8015")
      comm_ = new ZLAC8015Comm();

    bool success = comm_->ConnectComm(cfg_.port, cfg_.baud_rate, cfg_.timeout_ms);

    if (success)
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully configured!");

    return success ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Cleaning up ...please wait...");
    comm_->DisconnectComm();
    delete comm_;
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Activating ...please wait...");

    // set some default values
    left_wheel_cmd_rpm_ = 0.0;
    right_wheel_cmd_rpm_ = 0.0;
    left_hw_positions_ = 0.0;
    right_hw_positions_ = 0.0;
    left_encoder_rpm_ = 0.0;
    right_encoder_rpm_ = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comm_->connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // get rpm from motor
    bool success = comm_->ReadRPM(left_encoder_rpm_, right_encoder_rpm_);

    // calc
    left_hw_positions_ = left_hw_positions_ + period.seconds() * left_encoder_rpm_;
    right_hw_positions_ = right_hw_positions_ + period.seconds() * right_encoder_rpm_;

    // RCLCPP_INFO(
    //     rclcpp::get_logger("DiffDriveSystemHardware"),
    //     "Got position state %.5f and velocity state %.5f for '%s'!", left_hw_positions_,
    //     left_encoder_rpm_, info_.joints[0].name.c_str());
    // RCLCPP_INFO(
    //     rclcpp::get_logger("DiffDriveSystemHardware"),
    //     "Got position state %.5f and velocity state %.5f for '%s'!", right_hw_positions_,
    //     right_encoder_rpm_, info_.joints[0].name.c_str());

    return success ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
  }

  hardware_interface::return_type diffdrive_ros2_control ::DiffDriveSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Writing...");
    if (!comm_->connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    comm_->DriveCommand(left_wheel_cmd_rpm_, right_wheel_cmd_rpm_);

    // RCLCPP_INFO(
    //     rclcpp::get_logger("DiffDriveSystemHardware"), "Got command %.5f for '%s'!", left_wheel_cmd_rpm_,
    //     info_.joints[0].name.c_str());
    // RCLCPP_INFO(
    //     rclcpp::get_logger("DiffDriveSystemHardware"), "Got command %.5f for '%s'!", right_wheel_cmd_rpm_,
    //     info_.joints[1].name.c_str());

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Joints successfully written!");

    return hardware_interface::return_type::OK;
  }

} // namespace diffdrive_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdrive_ros2_control::DiffDriveSystemHardware, hardware_interface::SystemInterface)
