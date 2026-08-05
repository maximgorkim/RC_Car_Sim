#include "bno055_plugin.hpp"
#include <thread>

#define DEBUG false

namespace gazebo
{
    namespace bno055
    {   
        BNO055::BNO055():ModelPlugin() {}

        void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            if(!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }

            nh = rclcpp::Node::make_shared("bno055_plugin_node");
            timer = nh->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BNO055::OnUpdate, this));
            
            // Create executor and spin in separate thread
            executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor->add_node(nh);
            executor_thread = std::thread([this]() { executor->spin(); });
	    	
      			// Save a pointer to the model for later use
      			this->m_model = model_ptr;
      			
          	// Create topic name        	
          	std::string topic_name = "/automobile/imu";

            this->m_pubBNO = this->nh->create_publisher<sensor_msgs::msg::Imu>(topic_name, 10);


            if(DEBUG)
            {
                std::cerr << "\n\n";
                gzdbg << "====================================================================" << std::endl;
                gzdbg << "[bno055_plugin] attached to: " << this->m_model->GetName() << std::endl;
                gzdbg << "[bno055_plugin] publish to: "  << topic_name << std::endl;
                gzdbg << "[bno055_plugin] Usefull data: linear z, angular x, angular y, angular z" << std::endl;
                gzdbg << "====================================================================" << std::endl;
            }
        }

        // Publish the updated values
        void BNO055::OnUpdate()
        {        
            // Get orientation from Gazebo
            auto rot = this->m_model->RelativePose().Rot();
            
            // Fill header
            m_bno055_data.header.stamp = nh->get_clock()->now();
            m_bno055_data.header.frame_id = "imu_link";
            
            // Fill orientation (quaternion)
            m_bno055_data.orientation.x = rot.X();
            m_bno055_data.orientation.y = rot.Y();
            m_bno055_data.orientation.z = rot.Z();
            m_bno055_data.orientation.w = rot.W();
            
            // Set orientation covariance (assuming good accuracy)
            m_bno055_data.orientation_covariance = {
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01
            };
            
            // Angular velocity (from Gazebo)
            auto angular_vel = this->m_model->RelativeAngularVel();
            m_bno055_data.angular_velocity.x = angular_vel.X();
            m_bno055_data.angular_velocity.y = angular_vel.Y();
            m_bno055_data.angular_velocity.z = angular_vel.Z();
            
            m_bno055_data.angular_velocity_covariance = {
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01
            };
            
            // Linear acceleration (from Gazebo)
            auto linear_accel = this->m_model->RelativeLinearAccel();
            m_bno055_data.linear_acceleration.x = linear_accel.X();
            m_bno055_data.linear_acceleration.y = linear_accel.Y();
            m_bno055_data.linear_acceleration.z = linear_accel.Z() + 9.81; // Add gravity
            
            m_bno055_data.linear_acceleration_covariance = {
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01
            };
           	
            this->m_pubBNO->publish(this->m_bno055_data);
        };      
    }; //namespace trafficLight
    GZ_REGISTER_MODEL_PLUGIN(bno055::BNO055)
}; // namespace gazebo
