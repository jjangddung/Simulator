#include "wheel_speed_plugin.hpp"

#include <gazebo/common/Exception.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>

#include <cmath>
#include <functional>
#include <string>

namespace gazebo
{
namespace wheelspeed
{

void WheelSpeedPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  if (!sdf)
  {
    gzerr << "[wheel_speed_plugin] No SDF element provided." << std::endl;
    return;
  }

  joint_name_ = sdf->Get<std::string>("joint_name", "").first;
  if (joint_name_.empty())
  {
    gzerr << "[wheel_speed_plugin] Parameter <joint_name> is required." << std::endl;
    return;
  }

  wheel_radius_ = sdf->Get<double>("wheel_radius", wheel_radius_).first;
  if (wheel_radius_ <= 0.0)
  {
    gzerr << "[wheel_speed_plugin] <wheel_radius> must be positive." << std::endl;
    wheel_radius_ = 0.0325;
  }

  topic_name_ = sdf->Get<std::string>("topic", "/automobile/wheel_speed").first;
  twist_topic_ = sdf->Get<std::string>("twist_topic", "/automobile/wheel_twist").first;
  twist_frame_ = sdf->Get<std::string>("twist_frame", "base_link").first;

  const double update_rate = sdf->Get<double>("update_rate", 30.0).first;
  publish_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.0;
  twist_noise_stddev_ = sdf->Get<double>("twist_noise_stddev", twist_noise_stddev_).first;
  deadband_ = sdf->Get<double>("deadband", deadband_).first;

  joint_ = model_->GetJoint(joint_name_);
  if (!joint_)
  {
    gzerr << "[wheel_speed_plugin] Joint '" << joint_name_ << "' not found in model '"
          << model_->GetName() << "'." << std::endl;
    return;
  }

  ros_node_ = gazebo_ros::Node::Get(sdf);
  if (!ros_node_)
  {
    gzerr << "[wheel_speed_plugin] Failed to get ROS node." << std::endl;
    return;
  }

  speed_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32>(topic_name_, rclcpp::QoS(10));
  twist_pub_ = ros_node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    twist_topic_, rclcpp::QoS(10));

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&WheelSpeedPlugin::OnUpdate, this, std::placeholders::_1));

  RCLCPP_INFO(ros_node_->get_logger(),
    "[wheel_speed_plugin] Publishing joint '%s' linear speed on %s (radius=%.4fm, rate=%.1f Hz). Twist topic: %s",
    joint_name_.c_str(), topic_name_.c_str(), wheel_radius_, update_rate, twist_topic_.c_str());
}

void WheelSpeedPlugin::OnUpdate(const common::UpdateInfo & info)
{
  if ((!speed_pub_ && !twist_pub_) || !joint_)
  {
    return;
  }

  if (publish_period_ > 0.0 &&
      (info.simTime - last_publish_time_).Double() < publish_period_)
  {
    return;
  }

  last_publish_time_ = info.simTime;

  const double angular_speed = joint_->GetVelocity(0);  // rad/s
  double linear_speed = angular_speed * wheel_radius_;  // m/s

  if (std::fabs(linear_speed) < deadband_)
  {
    linear_speed = 0.0;
  }

  if (speed_pub_)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(linear_speed);
    speed_pub_->publish(msg);
  }

  if (twist_pub_)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = ros_node_->now();
    twist_msg.header.frame_id = twist_frame_;
    twist_msg.twist.twist.linear.x = linear_speed;

    const double var = twist_noise_stddev_ * twist_noise_stddev_;
    twist_msg.twist.covariance = {
      var, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1e6};

    twist_pub_->publish(twist_msg);
  }
}

GZ_REGISTER_MODEL_PLUGIN(WheelSpeedPlugin)

}  // namespace wheelspeed
}  // namespace gazebo
