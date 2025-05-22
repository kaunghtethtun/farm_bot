#include "arduino_hardware_interface/arduino_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <unistd.h>     // for ::read(), ::write()
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <sstream>
#include <iostream>

using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

namespace arduino_hardware_interface
{

hardware_interface::CallbackReturn ArduinoSystemHardware::on_init(
  const hardware_interface::HardwareInfo &info)
{
  std::string port = info_.hardware_parameters["port"];
  //std::string port = info.hardware_parameters.at("port");
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_cmd_left_ = 50.0;
  hw_cmd_right_ = 50.0;
  hw_pos_left_ = 0.0;
  hw_pos_right_ = 0.0;
  hw_vel_left_ = 0.0;
  hw_vel_right_ = 0.0;

  serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoSystemHardware"), "Failed to open serial port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoSystemHardware"), "Failed to get terminal attributes");
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;
  tcsetattr(serial_fd_, TCSANOW, &tty);
  RCLCPP_INFO(rclcpp::get_logger("ArduinoSystemHardware"), "Arduino interface initialized");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_pos_left_);
  state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_vel_left_);
  state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_pos_right_);
  state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_vel_right_);

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ArduinoSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_cmd_left_);
  command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_cmd_right_);

  return command_interfaces;
}

hardware_interface::return_type ArduinoSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return readEncoderData() ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type ArduinoSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return sendVelocityCommand(static_cast<int>(hw_cmd_left_), static_cast<int>(hw_cmd_right_))
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}

bool ArduinoSystemHardware::sendVelocityCommand(int left_rpm, int right_rpm)
{
  std::stringstream ss;
  ss << "(1," << right_rpm << "," << left_rpm << ";)";
  std::string msg = ss.str();
  RCLCPP_INFO(rclcpp::get_logger("ArduinoSystemHardware"), "Sending command: %s", msg.c_str());
  return ::write(serial_fd_, msg.c_str(), msg.size()) > 0;
}


bool ArduinoSystemHardware::readEncoderData()
{
  char buffer[128];
  int n = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (n <= 0) return false;

  buffer[n] = '\0';
  std::string data(buffer);

  if (data.find("3,") != 0) {
    RCLCPP_WARN(rclcpp::get_logger("ArduinoSystemHardware"), "Unexpected prefix in data: %s", data.c_str());
    return false;
  }

  size_t end = data.find(';');
  if (end == std::string::npos) {
    RCLCPP_WARN(rclcpp::get_logger("ArduinoSystemHardware"), "Missing semicolon in data: %s", data.c_str());
    return false;
  }

  std::string payload = data.substr(2, end - 2);  // Skip "3," and trim before ';'
  std::replace(payload.begin(), payload.end(), ',', ' ');
  std::stringstream ss(payload);
 

  int right_enc, left_enc;
  double right_vel, left_vel;
  if (!(ss >> right_enc >> left_enc >> right_vel >> left_vel))
  {
    return false;
  }

  hw_pos_right_ = static_cast<double>(right_enc) / 1000.0;
  hw_pos_left_ = static_cast<double>(left_enc) / 1000.0;
  
  // hw_vel_right_ = static_cast<double>(right_vel) / 100.0;
  // hw_vel_left_ = static_cast<double>(left_vel) / 100.0;

  hw_vel_right_ = right_vel;
  hw_vel_left_ = left_vel;

  RCLCPP_INFO(rclcpp::get_logger("ArduinoSystemHardware"), "Read encoders: %d %d %f %f",
    right_enc, left_enc, right_vel, left_vel);
  return true;
}


}  // namespace arduino_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  arduino_hardware_interface::ArduinoSystemHardware,
  hardware_interface::SystemInterface)
