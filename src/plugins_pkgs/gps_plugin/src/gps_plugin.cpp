
#include "gps_plugin.hpp"
#include <thread>

#define DEBUG false

namespace gazebo
{
    namespace gps
    {   
        GPS::GPS():ModelPlugin() {}
     
        void GPS::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
          if(!rclcpp::ok())
          {
              rclcpp::init(0, nullptr);
          }

          nh = rclcpp::Node::make_shared("gps_plugin_node");
          timer = nh->create_wall_timer(std::chrono::milliseconds(250), std::bind(&GPS::OnUpdate, this));
          
          // Create executor and spin in separate thread
          executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
          executor->add_node(nh);
          executor_thread = std::thread([this]() { executor->spin(); });

    // Save a pointer to the model for later use
    this->m_model = model_ptr;
  
        // Create topic name      
        std::string topic_name = "/automobile/odom";

        this->m_pubGPS = this->nh->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
        
          if(DEBUG)
          {
              std::cerr << "\n\n";
              gzdbg << "====================================================================" << std::endl;
              gzdbg << "[gps_plugin] attached to: " << this->m_model->GetName() << std::endl;
              gzdbg << "[gps_plugin] publish to: "  << topic_name << std::endl;
              gzdbg << "[gps_plugin] Publishing odometry data" << std::endl;
              gzdbg << "====================================================================\n\n" << std::endl;
          }
        }

        // Publish the updated values
        void GPS::OnUpdate()
        {
            // Get pose from Gazebo
            auto pose = this->m_model->WorldPose();
            auto linear_vel = this->m_model->RelativeLinearVel();
            auto angular_vel = this->m_model->RelativeAngularVel();
            
            // Fill header
            m_odom_data.header.stamp = nh->get_clock()->now();
            m_odom_data.header.frame_id = "odom";
            m_odom_data.child_frame_id = "base_link";
            
            // Fill position
            m_odom_data.pose.pose.position.x = pose.Pos().X();
            m_odom_data.pose.pose.position.y = pose.Pos().Y();
            m_odom_data.pose.pose.position.z = pose.Pos().Z();
            
            // Fill orientation (quaternion)
            m_odom_data.pose.pose.orientation.x = pose.Rot().X();
            m_odom_data.pose.pose.orientation.y = pose.Rot().Y();
            m_odom_data.pose.pose.orientation.z = pose.Rot().Z();
            m_odom_data.pose.pose.orientation.w = pose.Rot().W();
            
            // Pose covariance (6x6 = 36 elements)
            for (int i = 0; i < 36; i++) {
                m_odom_data.pose.covariance[i] = 0.0;
            }
            m_odom_data.pose.covariance[0] = 0.01;   // x
            m_odom_data.pose.covariance[7] = 0.01;   // y
            m_odom_data.pose.covariance[14] = 0.01;  // z
            m_odom_data.pose.covariance[21] = 0.01;  // roll
            m_odom_data.pose.covariance[28] = 0.01;  // pitch
            m_odom_data.pose.covariance[35] = 0.01;  // yaw
            
            // Fill twist (velocity)
            m_odom_data.twist.twist.linear.x = linear_vel.X();
            m_odom_data.twist.twist.linear.y = linear_vel.Y();
            m_odom_data.twist.twist.linear.z = linear_vel.Z();
            
            m_odom_data.twist.twist.angular.x = angular_vel.X();
            m_odom_data.twist.twist.angular.y = angular_vel.Y();
            m_odom_data.twist.twist.angular.z = angular_vel.Z();
            
            // Twist covariance (6x6 = 36 elements)
            for (int i = 0; i < 36; i++) {
                m_odom_data.twist.covariance[i] = 0.0;
            }
            m_odom_data.twist.covariance[0] = 0.01;
            m_odom_data.twist.covariance[7] = 0.01;
            m_odom_data.twist.covariance[14] = 0.01;
            m_odom_data.twist.covariance[21] = 0.01;
            m_odom_data.twist.covariance[28] = 0.01;
            m_odom_data.twist.covariance[35] = 0.01;
            
            this->m_pubGPS->publish(this->m_odom_data);
        };      
    }; //namespace gps
    GZ_REGISTER_MODEL_PLUGIN(gps::GPS)
}; // namespace gazebo
