#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <string>

namespace gazebo
{
namespace steering
{
class SteeringAnglePlugin : public ModelPlugin
{
public:
  SteeringAnglePlugin() = default;
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnUpdate(const common::UpdateInfo & info);

  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  event::ConnectionPtr update_connection_;

  physics::JointPtr left_joint_;
  physics::JointPtr right_joint_;
  std::string topic_name_{"/automobile/steering_angle"};
  double publish_period_{0.02};
  common::Time last_publish_{0};
};
}  // namespace steering
}  // namespace gazebo
