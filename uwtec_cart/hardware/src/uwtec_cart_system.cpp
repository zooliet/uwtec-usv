
#include "uwtec_cart/uwtec_cart_system.hpp"

// #include <chrono>
// #include <cstddef>
// #include <iomanip>
// #include <limits>
// #include <memory>
// #include <sstream>
// #include <vector>

#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"       // ROS 2 C++ client library
#include "std_msgs/msg/string.hpp" // Standard message type for strinmygs

namespace uwtec_cart {
  hardware_interface::CallbackReturn UwtecCartSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {

    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_start_sec_ = hardware_interface::stod(
        info_.hardware_parameters["hw_start_duration_sec"]);
    hw_stop_sec_ = hardware_interface::stod(
        info_.hardware_parameters["hw_stop_duration_sec"]);

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
      // UwtecCartSystem has two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1) {
        RCLCPP_FATAL(get_logger(),
                    "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(
            get_logger(),
            "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2) {
        RCLCPP_FATAL(get_logger(),
                    "Joint '%s' has %zu state interface. 2 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(
            get_logger(),
            "Joint '%s' have '%s' as first state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(
            get_logger(),
            "Joint '%s' have '%s' as second state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn UwtecCartSystemHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    // RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    //
    // for (int i = 0; i < hw_start_sec_; i++) {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
    // }

    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_) {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_) {
      set_command(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    if (get_node()) {
      uros_publisher_ =
          get_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn UwtecCartSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    // RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    //
    // for (auto i = 0; i < hw_start_sec_; i++) {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
    // }

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_command_interfaces_) {
      set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn UwtecCartSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    // RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    //
    // for (auto i = 0; i < hw_stop_sec_; i++) {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
    // }

    // RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type
  UwtecCartSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {

    std::stringstream ss;
    ss << "Reading states:";
    ss << std::fixed << std::setprecision(2);
    for (const auto &[name, descr] : joint_state_interfaces_) {
      if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
        set_state(name, get_state(name) + period.seconds() * velo);

        ss << std::endl
          << "\t position " << get_state(name) << " and velocity " << velo
          << " for '" << name << "'!";
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  UwtecCartSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    std::stringstream ss;
    ss << "Writing commands:";
    for (const auto &[name, descr] : joint_command_interfaces_) {
      // Simulate sending commands to the hardware
      set_state(name, get_command(name));

      ss << std::fixed << std::setprecision(2) << std::endl
        << "\t" << "command " << get_command(name) << " for '" << name << "'!";
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    x_speed = get_command("left_wheel_joint/velocity")/3.3;
    y_speed = get_command("right_wheel_joint/velocity")/3.3;
    if (x_speed < 0.05) // reverse direction or too slow
      x_speed = 0.0;
    else if (x_speed < 0.2)
      x_speed = 0.2;


    if (y_speed < 0.05) // reverse direction or too slow
      y_speed = 0.0;
    else if (y_speed < 0.2)
      y_speed = 0.2;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = x_speed; // get_command("left_wheel_joint/velocity")/3.3;
    msg.linear.y = y_speed; // get_command("right_wheel_joint/velocity")/3.3;
    uros_publisher_->publish(msg);

    return hardware_interface::return_type::OK;
  }
} // namespace uwtec_cart

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  uwtec_cart::UwtecCartSystemHardware,
  hardware_interface::SystemInterface
)
