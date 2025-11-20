#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

namespace gazebo
{
namespace wheelspeed
{
class WheelSpeedPlugin : public ModelPlugin
{
public:
  WheelSpeedPlugin() = default;
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnUpdate(const common::UpdateInfo & info);

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  event::ConnectionPtr update_connection_;

  std::string joint_name_;
  std::string topic_name_;
  std::string twist_topic_;
  std::string twist_frame_;
  double wheel_radius_{0.0325};
  double publish_period_{0.02};
  double twist_noise_stddev_{0.02};
  double deadband_{0.001};
  common::Time last_publish_time_;
};
}  // namespace wheelspeed
}  // namespace gazebo
