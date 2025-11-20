#include "bno055_plugin.hpp"

#include <cmath>

#define DEBUG true

namespace gazebo
{
    namespace bno055
    {   
        BNO055::BNO055():ModelPlugin() {}

        void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            this->m_model = model_ptr;
            this->m_ros_node = gazebo_ros::Node::Get(sdf_ptr);
            this->timer = this->m_ros_node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BNO055::OnUpdate, this));
      			
           	std::string topic_name = "/automobile/IMU";
          	this->m_pubBNO = this->m_ros_node->create_publisher<utils::msg::IMU>(topic_name, 2);

            if (sdf_ptr)
            {
                this->m_roll_noise_stddev = sdf_ptr->Get<double>("roll_noise_stddev", 0.0).first;
                this->m_pitch_noise_stddev = sdf_ptr->Get<double>("pitch_noise_stddev", 0.0).first;
                this->m_yaw_noise_stddev = sdf_ptr->Get<double>("yaw_noise_stddev", 0.0).first;
                this->m_ang_vel_noise_stddev = sdf_ptr->Get<double>("angular_velocity_noise_stddev", 0.0).first;
                this->m_lin_acc_noise_stddev = sdf_ptr->Get<double>("linear_acceleration_noise_stddev", 0.0).first;
                this->m_imu_topic = sdf_ptr->Get<std::string>("imu_topic", this->m_imu_topic).first;
                this->m_imu_frame = sdf_ptr->Get<std::string>("imu_frame", this->m_imu_frame).first;
            }

            this->m_pubImuStd = this->m_ros_node->create_publisher<sensor_msgs::msg::Imu>(this->m_imu_topic, rclcpp::QoS(10));

            std::random_device rd;
            this->m_rng.seed(rd());

            if(DEBUG)
            {
                auto logger = this->m_ros_node->get_logger();
                std::cerr << "\n\n";
                RCLCPP_INFO_STREAM(logger, "====================================================================");
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] attached to: " << this->m_model->GetName());
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] publish to: "  << topic_name);
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] noise stddev (roll, pitch, yaw): "
                    << this->m_roll_noise_stddev << ", "
                    << this->m_pitch_noise_stddev << ", "
                    << this->m_yaw_noise_stddev);
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] imu topic: " << this->m_imu_topic
                    << " frame: " << this->m_imu_frame);
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] Usefull data: linear z, angular x, angular y, angular z");
                RCLCPP_INFO_STREAM(logger, "====================================================================");
            }
        }

        // Publish the updated values
        void BNO055::OnUpdate()
        {        
          
            // TODO: accel fields are missing, is this intentional?
            auto pose = this->m_model->RelativePose();
            auto rot = pose.Rot();

            const double noisy_roll = rot.Roll() + this->SampleNoise(this->m_roll_noise_stddev);
            const double noisy_pitch = rot.Pitch() + this->SampleNoise(this->m_pitch_noise_stddev);
            const double noisy_yaw = rot.Yaw() + this->SampleNoise(this->m_yaw_noise_stddev);

           	this->m_bno055_pose.roll = noisy_roll;
            this->m_bno055_pose.pitch = noisy_pitch;
           	this->m_bno055_pose.yaw = noisy_yaw;
            this->m_pubBNO->publish(this->m_bno055_pose);

            if (this->m_pubImuStd)
            {
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header.stamp = this->m_ros_node->now();
                imu_msg.header.frame_id = this->m_imu_frame;

                ignition::math::Quaterniond noisy_quat(noisy_roll, noisy_pitch, noisy_yaw);
                imu_msg.orientation.x = noisy_quat.X();
                imu_msg.orientation.y = noisy_quat.Y();
                imu_msg.orientation.z = noisy_quat.Z();
                imu_msg.orientation.w = noisy_quat.W();

                const double roll_var = std::pow(this->m_roll_noise_stddev, 2);
                const double pitch_var = std::pow(this->m_pitch_noise_stddev, 2);
                const double yaw_var = std::pow(this->m_yaw_noise_stddev, 2);
                imu_msg.orientation_covariance = {
                    roll_var, 0.0,      0.0,
                    0.0,      pitch_var,0.0,
                    0.0,      0.0,      yaw_var};

                ignition::math::Vector3d ang_vel = this->m_model->RelativeAngularVel();
                imu_msg.angular_velocity.x = ang_vel.X() + this->SampleNoise(this->m_ang_vel_noise_stddev);
                imu_msg.angular_velocity.y = ang_vel.Y() + this->SampleNoise(this->m_ang_vel_noise_stddev);
                imu_msg.angular_velocity.z = ang_vel.Z() + this->SampleNoise(this->m_ang_vel_noise_stddev);

                const double ang_var = std::pow(this->m_ang_vel_noise_stddev, 2);
                imu_msg.angular_velocity_covariance = {
                    ang_var, 0.0,    0.0,
                    0.0,    ang_var, 0.0,
                    0.0,    0.0,    ang_var};

                ignition::math::Vector3d lin_acc = this->m_model->RelativeLinearAccel();
                imu_msg.linear_acceleration.x = lin_acc.X() + this->SampleNoise(this->m_lin_acc_noise_stddev);
                imu_msg.linear_acceleration.y = lin_acc.Y() + this->SampleNoise(this->m_lin_acc_noise_stddev);
                imu_msg.linear_acceleration.z = lin_acc.Z() + this->SampleNoise(this->m_lin_acc_noise_stddev);

                const double lin_var = std::pow(this->m_lin_acc_noise_stddev, 2);
                imu_msg.linear_acceleration_covariance = {
                    lin_var, 0.0,   0.0,
                    0.0,   lin_var, 0.0,
                    0.0,   0.0,   lin_var};

                this->m_pubImuStd->publish(imu_msg);
            }
        };

        double BNO055::SampleNoise(double stddev)
        {
            if (stddev <= 0.0)
            {
                return 0.0;
            }
            return stddev * this->m_unit_normal(this->m_rng);
        }
    }; //namespace trafficLight
    GZ_REGISTER_MODEL_PLUGIN(bno055::BNO055)
}; // namespace gazebo
