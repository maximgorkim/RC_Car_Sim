#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo
{
    namespace gps
    {   
        class GPS: public ModelPlugin
    	{
        private: 
            physics::ModelPtr                   m_model;
            rclcpp::Node::SharedPtr             nh;
            rclcpp::TimerBase::SharedPtr        timer;
            rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
            std::thread executor_thread;

	    /** ROS INTEGRATION **/
            // A ROS publisher
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  m_pubGPS;

            // The odometry message
            nav_msgs::msg::Odometry             m_odom_data;
            
        // Default constructor
        public: GPS();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();        
        };
    };    
};