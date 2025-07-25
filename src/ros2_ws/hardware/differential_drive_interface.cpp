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

#include "differential_drive_interface.hpp"
#include "libserial/SerialPortConstants.h"

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

hardware_interface::CallbackReturn DifferentialDriveInterface::on_init(
  const hardware_interface::HardwareInfo &info)
{
  if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDriveInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("DifferentialDriveInterface");

  for (const auto &[name, descr] : joint_state_interfaces_)
  {
    RCLCPP_INFO(logger, name.c_str());
    set_state(name, 0.0);

  }
  for (const auto &[name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  try {
    serial_port.Open("/dev/ttyAMA1");
  } catch (const LibSerial::OpenFailed& e) {
    RCLCPP_ERROR(logger, "Failed to open serial port %s: %s", "/dev/ttyAMA1", e.what());
    throw;
  }
  serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
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

  serial_port.Close(); 
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DifferentialDriveInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration &/*period*/)
{
  int lineCount = 0;
  size_t bytesAvailable = serial_port.GetNumberOfBytesAvailable();
  
  while (bytesAvailable > 0 && lineCount < 1000) {
      std::string line;
      serial_port.ReadLine(line, '\n', 10);
      std::vector<std::string> tokens = split(line.c_str(), ' ');
      bytesAvailable = serial_port.GetNumberOfBytesAvailable();
      lineCount++;

      if (tokens.size() < 4)
        continue;
      
      std::string response_title = tokens[0];
      std::string side = tokens[1];
      std::string angle = tokens[2];
      std::string angular_vel = tokens[3];
        
      if (response_title == "resp:angle_angular_vel") 
      {
        Wheel* wheel = side == "r" ? &right_wheel: &left_wheel;
        try {
          wheel->position = std::stod(angle);
          wheel->velocity = std::stod(angular_vel);
        } catch(const std::invalid_argument&) {
          RCLCPP_INFO(this->get_logger(), "Invalid angle_angular_vel received, skipping: %s %s", angle.c_str(),  angular_vel.c_str());
        }
      }
  }

  set_state("left_wheel_joint/position", left_wheel.position);
  set_state("left_wheel_joint/velocity", left_wheel.velocity);
  set_state("right_wheel_joint/position", right_wheel.position);
  set_state("right_wheel_joint/velocity", right_wheel.velocity);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DifferentialDriveInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration &/*period*/)
{
  std::string command = "";

  double left_speed = get_command("left_wheel_joint/velocity");
  double right_speed = get_command("right_wheel_joint/velocity");

  command.append("cmd:set_motor l angular_vel " + std::to_string(left_speed) + "\n");
  command.append("cmd:set_motor r angular_vel " + std::to_string(right_speed) + "\n");
  command.append("cmd:get_motor r angle_angular_vel\n");
  command.append("cmd:get_motor l angle_angular_vel\n");

  serial_port.Write(command);
  // RCLCPP_INFO(this->get_logger(), "Left speed: %f", left_speed);
  // RCLCPP_INFO(this->get_logger(), "Right speed: %f", left_speed);

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    differential_drive_interface::DifferentialDriveInterface, hardware_interface::SystemInterface)