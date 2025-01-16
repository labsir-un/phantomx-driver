#pragma once

// C++
#include <memory>
#include <string>

// ROS
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

// #include <rclcpp/publisher.hpp>
// #include <rclcpp/subscription.hpp>


#include <sensor_msgs/msg/joint_state.hpp>

namespace phantom_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PhantomPincherSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

private:
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_states_subscriber_;
//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  rclcpp::Node::SharedPtr node_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool sum_wrapped_joint_states_{ false };

  /// Use standard interfaces for joints because they are relevant for dynamic behavior
  std::array<std::string, 4> standard_interfaces_ = { hardware_interface::HW_IF_POSITION,
                                                      hardware_interface::HW_IF_VELOCITY,
                                                      hardware_interface::HW_IF_ACCELERATION,
                                                      hardware_interface::HW_IF_EFFORT };

  struct MimicJoint
  {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
  };
  std::vector<MimicJoint> mimic_joints_;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;

  // If the difference between the current joint state and joint command is less than this value,
  // the joint command will not be published.
  double trigger_joint_command_threshold_ = 1e-5;

  // Dynamixel control table regs 
  int ADDR_TORQUE_EN_;
  int ADDR_POSITION_GOAL_; 
  int ADDR_VELOLICTY_GOAL_; 
  int ADDR_OPERATING_MODE_; 
  int ADDR_PRESENT_POSITION_; 
  int ADDR_PRESENT_VELOCITY_; 
  int BAUDRATE_; 

  int present_position; 
  
  
  dynamixel::PortHandler * portHandler;
  dynamixel::PacketHandler * packetHandler;

  uint8_t dxl_error = 0;
  uint8_t dxl_id = BROADCAST_ID; 
  uint32_t goal_position = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  std::string DEVICE_NAME_ ; 

  double PROTOCOL_VERSION_; 


  template <typename HandleType>
  bool getInterface(const std::string& name, const std::string& interface_name, const size_t vector_index,
                    std::vector<std::vector<double>>& values, std::vector<HandleType>& interfaces);
};

}  // namespace topic_based_ros2_control