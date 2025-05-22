#ifndef ARDUINO_HARDWARE_INTERFACE__ARDUINO_SYSTEM_HPP_
#define ARDUINO_HARDWARE_INTERFACE__ARDUINO_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <string>
#include <vector>

namespace arduino_hardware_interface
{

class ArduinoSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

  hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Serial communication
  int serial_fd_;

  // Hardware state
  double hw_cmd_left_;
  double hw_cmd_right_;
  double hw_pos_left_;
  double hw_pos_right_;
  double hw_vel_left_;
  double hw_vel_right_;

  // Internal helpers
  bool sendVelocityCommand(int left_rpm, int right_rpm);
  bool readEncoderData();
};

}  // namespace arduino_hardware_interface

#endif  // ARDUINO_HARDWARE_INTERFACE__ARDUINO_SYSTEM_HPP_
