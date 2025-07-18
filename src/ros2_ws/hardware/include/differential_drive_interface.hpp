#pragma once

#include <memory>
#include <string>
#include <vector>


#include "wheel.hpp"
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
#include "libserial/SerialPort.h"

using namespace LibSerial;

namespace differential_drive_interface
{
  class DifferentialDriveInterface : public hardware_interface::SystemInterface
  {
  public:

    DifferentialDriveInterface() = default;
    RCLCPP_SHARED_PTR_DEFINITIONS(DifferentialDriveInterface)
    
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private: 
    Wheel left_wheel = Wheel("left_wheel", 100);
    Wheel right_wheel = Wheel("right_wheel", 100);
    
    SerialPort serial_port;
  };

}