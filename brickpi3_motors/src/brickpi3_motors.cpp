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
  hw_gear_ratios_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_interface_type_.resize(info_.joints.size());
  hw_min_position_limits_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_max_position_limits_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_zero_start_power_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_zero_start_wait_time_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_zero_stall_dps_threshold_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_zero_relative_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_zero_relative_wait_time_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    //We assume BrickPi3 models have exactly two states and one command interface on each joint
  
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
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
  const std::string gear_ratio_str = std::string("gear_ratio");
  for (auto i = 0u; i < info.joints.size(); i++)
  {
    // Map the hardware command interface onto lego port numbers
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

    // Set gear_ratio
    float gear_ratio = 1.0;
    auto parameters = info.joints[i].parameters;
    auto gear_param = parameters.find(gear_ratio_str);
    if (gear_param != parameters.end())
      gear_ratio = std::stod(gear_param->second);
    hw_gear_ratios_[i] = gear_ratio;

    // Set joint command type
    if (info.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY || info.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
      hw_commands_interface_type_[i] = info.joints[i].command_interfaces[0].name;
    else
      RCLCPP_FATAL(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Joint '%s' has unknown command interface %s.", info.joints[i].name.c_str(),
        info.joints[i].command_interfaces[0].name.c_str());

    if (info.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
      // Calibrating and zero's position. EV3 Motors lose encoder information on power reset.
      // Parameters for move joint to physical stop, adjust physical position and then reset encoder.
      set_parameter(info.joints[i].name, parameters, "hw_min_position_limit", hw_min_position_limits_[i]);
      set_parameter(info.joints[i].name, parameters, "hw_max_position_limit", hw_max_position_limits_[i]);
      set_parameter(info.joints[i].name, parameters, "hw_position_zero_start_power",
        hw_position_zero_start_power_[i]);
      set_parameter(info.joints[i].name, parameters, "hw_position_zero_start_wait_time",
        hw_position_zero_start_wait_time_[i]);
      set_parameter(info.joints[i].name, parameters, "hw_position_zero_stall_dps_threshold",
        hw_position_zero_stall_dps_threshold_[i]);
      set_parameter(info.joints[i].name, parameters, "hw_position_zero_relative", hw_position_zero_relative_[i]);
      set_parameter(info.joints[i].name, parameters, "hw_position_zero_relative_wait_time",
        hw_position_zero_relative_wait_time_[i]);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void
BrickPi3MotorsHardware::set_parameter(const std::string& joint_name,
  const std::unordered_map<std::string, std::string>& parameters,
  const std::string& parameter_name, int& parameter)
{
  auto param = parameters.find(parameter_name);
  if (param != parameters.end())
    parameter = std::stod(param->second);
  else
    RCLCPP_WARN(
      rclcpp::get_logger("BrickPi3MotorsHardware"),
      "Joint '%s' has position command interface, but no '%s' set.", joint_name.c_str(),
      parameter_name.c_str());
}

void
BrickPi3MotorsHardware::set_parameter(const std::string& joint_name,
  const std::unordered_map<std::string, std::string>& parameters,
  const std::string& parameter_name, double& parameter)
{
  auto param = parameters.find(parameter_name);
  if (param != parameters.end())
    parameter = std::stod(param->second);
  else
    RCLCPP_WARN(
      rclcpp::get_logger("BrickPi3MotorsHardware"),
      "Joint '%s' has position command interface, but no '%s' set.", joint_name.c_str(),
      parameter_name.c_str());
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
      info_.joints[i].name, hw_commands_interface_type_[i], &hw_commands_[i]));
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

  // for any position command interfaces, perform encoder procedure reset
  // EV3 motors reset encoder information on power loss, so this resets to known position.
  for (auto i = 0u; i < hw_positions_.size(); i++)
    if (hw_commands_interface_type_[i] == hardware_interface::HW_IF_POSITION) {
      RCLCPP_INFO(
        rclcpp::get_logger("BrickPi3MotorsHardware"),
        "Resetting position joint on interface '%d'.", i);
      int stalled = 0;
      brickpi3.set_motor_power(hw_lego_ports_[i], hw_position_zero_start_power_[i]);
      rclcpp::sleep_for(std::chrono::milliseconds(hw_position_zero_start_wait_time_[i]));
      while (stalled==0) {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        uint8_t state;
        int8_t power = 0;
        int32_t position = 0;
        int16_t dps = 0;
        brickpi3.get_motor_status(hw_lego_ports_[i], state, power, position, dps);
        if (dps < hw_position_zero_stall_dps_threshold_[i]) { //30
          stalled = 1;
          brickpi3.set_motor_power(hw_lego_ports_[i], 0.0);
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      brickpi3.set_motor_position_relative(hw_lego_ports_[i], hw_position_zero_relative_[i]);
      rclcpp::sleep_for(std::chrono::milliseconds(hw_position_zero_relative_wait_time_[i]));
      brickpi3.set_motor_power(hw_lego_ports_[i], 0);
      brickpi3.offset_motor_encoder(hw_lego_ports_[i], brickpi3.get_motor_encoder(hw_lego_ports_[i]));
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
  int32_t motor_position_degrees;
  int16_t motor_dps;
  int32_t position_degrees;
  int16_t dps;
  double radps;
  double position;

  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    brickpi3.get_motor_status(hw_lego_ports_[i], state, power, motor_position_degrees, motor_dps);
    position_degrees = motor_position_degrees * hw_gear_ratios_[i];
    dps = motor_dps * hw_gear_ratios_[i];
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
    RCLCPP_DEBUG(
      rclcpp::get_logger("BrickPi3MotorsHardware"),
      "Command interface %d has value %f.", i,
      hw_commands_[i]);
    if (hw_commands_interface_type_[i] == hardware_interface::HW_IF_VELOCITY) {
      double dps = hw_commands_[i]*360.0/(2.0*MATH_PI);
      double motor_dps = dps / hw_gear_ratios_[i];
      brickpi3.set_motor_dps(hw_lego_ports_[i], motor_dps);
    }
    else { // Must be hardware_interface::HW_IF_POSITION
      double degrees = hw_commands_[i]*360.0/(2.0*MATH_PI);
      double motor_degrees = degrees / hw_gear_ratios_[i];
      double limit_motor_degrees = std::max(std::min(motor_degrees, hw_max_position_limits_[i]), hw_min_position_limits_[i]);
      brickpi3.set_motor_position(hw_lego_ports_[i], limit_motor_degrees);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace brickpi3_motors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  brickpi3_motors::BrickPi3MotorsHardware, hardware_interface::SystemInterface)
