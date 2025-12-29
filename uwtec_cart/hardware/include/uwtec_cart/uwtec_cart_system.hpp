#ifndef UWTEC_CART__UWTEC_CART_SYSTEM_HPP_
#define UWTEC_CART__UWTEC_CART_SYSTEM_HPP_

// #include <memory>
// #include <string>
// #include <vector>
// #include "std_msgs/msg/string.hpp" // Standard message type for strinmygs

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
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp" // Standard message type for strinmygs

namespace uwtec_cart {
class UwtecCartSystemHardware : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(UwtecCartSystemHardware)

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type 
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type
    write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // Parameters
    double hw_start_sec_;
    double hw_stop_sec_;
    double x_speed;
    double y_speed;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uros_publisher_;
  };
} // namespace uwtec_cart
#endif // UWTEC_CART__UWTEC_CART_SYSTEM_HPP_ 

