#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace gazebo
{
    namespace bno055
    {   
        class BNO055: public ModelPlugin
    	{
        private: 
            physics::ModelPtr                   m_model;
            rclcpp::Node::SharedPtr             nh;
            rclcpp::TimerBase::SharedPtr        timer;
            rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
            std::thread executor_thread;

            /** ----------------------------------For ROS integration----------------------------------------------------**/
            // A ROS publisher
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  m_pubBNO;
            
            sensor_msgs::msg::Imu               m_bno055_data;

        // Default constructor
        public: BNO055();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();        
        };
    };    
};