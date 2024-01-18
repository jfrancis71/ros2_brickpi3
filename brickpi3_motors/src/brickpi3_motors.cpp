#include "brickpi3_motors/brickpi3_motors.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "BrickPi3.cpp"


BrickPi3 brickpi3;
std::map<std::string, int> lego_port_map = {
  {"PORT_A", PORT_A}, {"PORT_B", PORT_B},
  {"PORT_C", PORT_C}, {"PORT_D", PORT_D}};


const double MATH_PI = 3.141592653589793;


namespace brickpi3_motors
{
hardware_interface::CallbackReturn BrickPi3MotorsHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_start_sec_ = std::stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_lego_ports_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DifferentialDrive has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

  }

  // Map the hardware command interface onto lego port numbers
  for (auto i = 0u; i < info.joints.size(); i++)
  {
    std::map<std::string, int>::iterator pos = lego_port_map.find(info.joints[i].name);
    if (pos != lego_port_map.end())
      hw_lego_ports_[i] = pos->second;
    else
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Unknown lego port '%s'", info.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BrickPi3MotorsHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BrickPi3MotorsHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn BrickPi3MotorsHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BrickPi3MotorsHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("BrickPi3MotorsHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    brickpi3.reset_motor_encoder(hw_lego_ports_[i]);
    hw_positions_[i] = 0;
    hw_velocities_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("BrickPi3MotorsHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BrickPi3MotorsHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BrickPi3MotorsHardware"), "Deactivating ...please wait...");
  for (auto i = 0u; i < hw_commands_.size(); i++)
    brickpi3.set_motor_dps(hw_lego_ports_[i], 0.0);

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("BrickPi3MotorsHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("BrickPi3MotorsHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BrickPi3MotorsHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  uint8_t state;
  int8_t power;
  int32_t position_degrees;
  int16_t dps;
  double radps;
  double position;

  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    brickpi3.get_motor_status(hw_lego_ports_[i], state, power, position_degrees, dps);
    position = position_degrees*(2.0*MATH_PI/360.0);
    radps = dps*(2.0*MATH_PI/360.0);
    hw_positions_[i]= position;
    hw_velocities_[i] = radps;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BrickPi3MotorsHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & /*period*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    double dps = hw_commands_[i]*360.0/(2.0*MATH_PI);
    brickpi3.set_motor_dps(hw_lego_ports_[i], dps);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace brickpi3_motors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  brickpi3_motors::BrickPi3MotorsHardware, hardware_interface::SystemInterface)
