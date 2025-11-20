#include "steering_angle_plugin.hpp"

#include <gazebo/common/Exception.hh>
#include <gazebo/physics/Joint.hh>

namespace gazebo
{
namespace steering
{

void SteeringAnglePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  if (!sdf)
  {
    gzerr << "[steering_angle_plugin] No SDF parameters provided." << std::endl;
    return;
  }

  const std::string left_name = sdf->Get<std::string>("left_joint", "").first;
  const std::string right_name = sdf->Get<std::string>("right_joint", "").first;
  topic_name_ = sdf->Get<std::string>("topic", topic_name_).first;
  const double update_rate = sdf->Get<double>("update_rate", 50.0).first;
  publish_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.0;

  ros_node_ = gazebo_ros::Node::Get(sdf);
  if (!ros_node_)
  {
    gzerr << "[steering_angle_plugin] Failed to get ROS node." << std::endl;
    return;
  }

  if (!left_name.empty())
  {
    left_joint_ = model_->GetJoint(left_name);
    if (!left_joint_)
    {
      gzerr << "[steering_angle_plugin] Left joint '" << left_name << "' not found." << std::endl;
    }
  }

  if (!right_name.empty())
  {
    right_joint_ = model_->GetJoint(right_name);
    if (!right_joint_)
    {
      gzerr << "[steering_angle_plugin] Right joint '" << right_name << "' not found." << std::endl;
    }
  }

  if (!left_joint_ && !right_joint_)
  {
    gzerr << "[steering_angle_plugin] No valid joints provided." << std::endl;
    return;
  }

  angle_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32>(topic_name_, rclcpp::QoS(10));

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&SteeringAnglePlugin::OnUpdate, this, std::placeholders::_1));

  RCLCPP_INFO(ros_node_->get_logger(),
    "[steering_angle_plugin] Publishing steering angle on %s", topic_name_.c_str());
}

void SteeringAnglePlugin::OnUpdate(const common::UpdateInfo & info)
{
  if (!angle_pub_)
  {
    return;
  }

  if (publish_period_ > 0.0 && (info.simTime - last_publish_).Double() < publish_period_)
  {
    return;
  }

  last_publish_ = info.simTime;

  double angle = 0.0;
  int count = 0;

  if (left_joint_)
  {
    angle += left_joint_->Position(0);
    ++count;
  }
  if (right_joint_)
  {
    angle += right_joint_->Position(0);
    ++count;
  }

  if (count == 0)
  {
    return;
  }

  angle /= static_cast<double>(count);

  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(angle);
  angle_pub_->publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(SteeringAnglePlugin)

}  // namespace steering
}  // namespace gazebo
