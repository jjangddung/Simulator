#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"
#include <utils/msg/imu.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>

namespace gazebo
{
    namespace bno055
    {   
        class BNO055: public ModelPlugin
    	{
        private: 
            physics::ModelPtr m_model;
            rclcpp::TimerBase::SharedPtr timer;
            rclcpp::Node::SharedPtr m_ros_node;
            rclcpp::Publisher<utils::msg::IMU>::SharedPtr m_pubBNO;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pubImuStd;
            utils::msg::IMU m_bno055_pose;
            std::default_random_engine m_rng;
            std::normal_distribution<double> m_unit_normal{0.0, 1.0};
            double m_roll_noise_stddev{0.0};
            double m_pitch_noise_stddev{0.0};
            double m_yaw_noise_stddev{0.0};
            double m_ang_vel_noise_stddev{0.0};
            double m_lin_acc_noise_stddev{0.0};
            std::string m_imu_topic{"/automobile/imu_raw"};
            std::string m_imu_frame{"base_link"};

        // Default constructor
        public: BNO055();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();
        private: double SampleNoise(double stddev);
        };
    };
};
