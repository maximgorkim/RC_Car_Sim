#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <random>
#include <rclcpp/qos.hpp>
#include "carlikerobot.hpp"
#include "rapidjson/document.h"

namespace gazebo
{
	namespace carlikerobot
    {

        class IRobotCommandSetter
        {
            public:
                virtual void setCommand()=0;
                float f_steer;
                float f_speed;
        };       
        
        typedef std::shared_ptr<IRobotCommandSetter> IRobotCommandSetterPtr;

        class CMessageHandler
        {
            public:
                CMessageHandler(std::string, IRobotCommandSetter*, rclcpp::Node::SharedPtr, std::string _speedTopic, std::string _steerTopic);
                ~CMessageHandler();
                
            private:
                void OnSpeedCommand(const std_msgs::msg::Float64::SharedPtr _msg);
                void OnSteerCommand(const std_msgs::msg::Float64::SharedPtr _msg);
                
                // private variable 
                IRobotCommandSetter*                _robotSetter;

                rclcpp::Node::SharedPtr             _rosNode;
                rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _speedSubscriber;
                rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _steerSubscriber;
        };
        
        
        class CCarLikeRobotRosPlugin: public ModelPlugin, public IRobotCommandSetter
        {
            
            public:
                CCarLikeRobotRosPlugin();
                bool LoadParameterJoints(sdf::ElementPtr _sdf);
                void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
                void setCommand();
                void OnUpdate();

            private:
                // Private variable
                IWheelsSpeedPtr     _rearWheelsSpeedPtr;
                IWheelsSpeedPtr     _frontWheelSpeedPtr;
                ISteerWheelsPtr     _steerWheelsAnglePtr;
                
                physics::ModelPtr   _model;                

                std::shared_ptr<CMessageHandler>      _messageHandler;
                
                // ROS2 elements
                rclcpp::Node::SharedPtr               _rosNode;
                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odomPublisher;
                std::unique_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;
                
                event::ConnectionPtr _updateConnection;
                common::Time _lastOdomTime;

                // Gaussian noise for ZED odometry simulation
                std::mt19937 _rng{std::random_device{}()};
                std::normal_distribution<double> _posNoise{0.0, 0.01};   // σ = 1 cm
                std::normal_distribution<double> _angNoise{0.0, 0.001};  // σ ≈ 0.057 deg

            public:
                // Mapping parameters
                double _speedToMsGain;
                double _steerToRadGain;
                double _steerToRadOffset;
                std::string _odomTopicName;
        };	
	}; // namespace carlikerobot
}; // namespace gazebo
