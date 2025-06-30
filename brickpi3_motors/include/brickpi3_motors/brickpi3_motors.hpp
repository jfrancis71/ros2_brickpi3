#ifndef BRICKPI3_MOTORS__BRICKPI3_MOTORS_HPP_
#define BRICKPI3_MOTORS__BRICKPI3_MOTORS_HPP_

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

#include "brickpi3_motors/visibility_control.h"

namespace brickpi3_motors
{
class BrickPi3MotorsHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BrickPi3MotorsHardware);

  BRICKPI3_MOTORS_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  BRICKPI3_MOTORS_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  BRICKPI3_MOTORS_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  BRICKPI3_MOTORS_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  BRICKPI3_MOTORS_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  BRICKPI3_MOTORS_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  BRICKPI3_MOTORS_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  void set_parameter(const std::string& joint_name, const std::unordered_map<std::string, std::string>& parameters,
    const std::string& parameter_name, int& parameter);
  void set_parameter(const std::string& joint_name, const std::unordered_map<std::string, std::string>& parameters,
    const std::string& parameter_name, double& parameter);

  double hw_start_sec_;
  double hw_stop_sec_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<int> hw_lego_ports_;
  std::vector<double> hw_gear_ratios_;  // ratio of joint rotation to motor rotation
  std::vector<std::string> hw_commands_interface_type_;  // velocity/position interface

  // Below are applicable to Position command interface.
  std::vector<double> hw_min_position_limits_;
  std::vector<double> hw_max_position_limits_;
  std::vector<double> hw_position_zero_start_power_;
  std::vector<int> hw_position_zero_start_wait_time_;  // ms
  std::vector<double> hw_position_zero_stall_dps_threshold_;
  std::vector<double> hw_position_zero_relative_;
  std::vector<int> hw_position_zero_relative_wait_time_;  // ms
};

}  // namespace brickpi3_motors

#endif  // BRICKPI3_MOTORS__BRICKPI3_MOTORS_HPP_
