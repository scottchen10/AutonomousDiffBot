#include "differential_drive_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <string>
 
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial.h"

std::vector<std::string> split(const char *str, char c = ' ')
{
    std::vector<std::string> result;

    do
    {
        const char *begin = str;

        while(*str != c && *str)
            str++;

        result.push_back(std::string(begin, str));
    } while (0 != *str++);

    return result;
}

namespace differential_drive_interface
{

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DifferentialDriveInterface::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  for (int joint_index = 0; joint_index < 1; ++joint_index) {
    Wheel* wheel = joint_index == 0 ? &left_wheel: &right_wheel;

    state_interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(
        info_.joints[joint_index].name, hardware_interface::HW_IF_POSITION, &wheel->position
      )
    );
    state_interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(
        info_.joints[joint_index].name, hardware_interface::HW_IF_VELOCITY, &wheel->velocity
      )
    );  
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn DifferentialDriveInterface::on_init(
  const hardware_interface::HardwareInfo &info)
{
  if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_wheel = Wheel("left_wheel_joint", 44);
  right_wheel = Wheel("right_wheel_joint", 44);

  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDriveInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto &[name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto &[name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  serial_port.setPort("/dev/ttyAMA1");
  serial_port.setBaudrate(115200);
  serial_port.open();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDriveInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

  // command and state should be equal when starting
  for (const auto &[name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDriveInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  serial_port.close(); 
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DifferentialDriveInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration &period)
{
  int lineCount = 0;
  while (serial_port.available() && lineCount < 1000) {
      std::string line = serial_port.readline(1024, "\n");
      std::vector<std::string> tokens = split(line.c_str(), ' ');

      if (tokens.size() < 4)
        continue;

      std::string response_title = tokens[0];
      std::string side = tokens[1];
      std::string angle = tokens[2];
      std::string angular_vel = tokens[3];


      if (response_title == "resp:angle_angular_vel") 
      {
        Wheel wheel = side == "r" ? right_wheel: left_wheel;
        wheel.position = std::stod(angle);
        wheel.velocity = std::stod(angular_vel);
      }
      lineCount++;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DifferentialDriveInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    serial_port.write("cmd:get_motor r angle_angular_vel" + '\n');
    serial_port.write("cmd:get_motor l angle_angular_vel" + '\n');
    set_state(name, get_command(name));
  }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    differential_drive_interface::DifferentialDriveInterface, hardware_interface::SystemInterface)