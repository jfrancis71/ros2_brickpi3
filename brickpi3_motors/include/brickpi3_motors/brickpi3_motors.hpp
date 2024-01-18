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
  double hw_start_sec_;
  double hw_stop_sec_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<int> hw_lego_ports_;
};

}  // namespace brickpi3_motors

#endif  // BRICKPI3_MOTORS__BRICKPI3_MOTORS_HPP_
