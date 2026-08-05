#include "carlikerobot_ros_plugin.hpp"
#include <gazebo_ros/node.hpp>


#define DEBUG false

namespace gazebo
{
    namespace carlikerobot
    {
        CMessageHandler::CMessageHandler(std::string _modelName, IRobotCommandSetter *_setter, rclcpp::Node::SharedPtr _node, std::string _speedTopic, std::string _steerTopic)
        {
            this->_robotSetter = _setter;
            this->_rosNode = _node;

            rclcpp::QoS command_qos(rclcpp::KeepLast(1));
            command_qos.best_effort();
            command_qos.durability_volatile();

            this->_speedSubscriber =
                this->_rosNode->create_subscription<std_msgs::msg::Float64>(
                    _speedTopic,
                    command_qos,
                    std::bind(
                        &CMessageHandler::OnSpeedCommand,
                        this,
                        std::placeholders::_1));

            this->_steerSubscriber =
                this->_rosNode->create_subscription<std_msgs::msg::Float64>(
                    _steerTopic,
                    command_qos,
                    std::bind(
                        &CMessageHandler::OnSteerCommand,
                        this,
                        std::placeholders::_1));
        }

        CMessageHandler::~CMessageHandler()
        {
        }

        void CMessageHandler::OnSpeedCommand(const std_msgs::msg::Float64::SharedPtr _msg)
        {
            CCarLikeRobotRosPlugin *plugin = static_cast<CCarLikeRobotRosPlugin *>(_robotSetter);
            // speed_ms = (erpm_command - speed_to_erpm_offset) / speed_to_erpm_gain
            // Assuming offset is 0 as per real car config
            _robotSetter->f_speed = _msg->data / plugin->_speedToMsGain;
            _robotSetter->setCommand();
        }

        void CMessageHandler::OnSteerCommand(const std_msgs::msg::Float64::SharedPtr _msg)
        {
            CCarLikeRobotRosPlugin *plugin = static_cast<CCarLikeRobotRosPlugin *>(_robotSetter);
            // steering_angle_rad = (servo_command - offset) / gain
            // steering_angle_deg = steering_angle_rad * (180.0 / PI)
            double steeringAngleRad = (_msg->data - plugin->_steerToRadOffset) / plugin->_steerToRadGain;
            _robotSetter->f_steer = steeringAngleRad * (180.0 / PI);
            _robotSetter->setCommand();
        }

        CCarLikeRobotRosPlugin::CCarLikeRobotRosPlugin()
        {
            _speedToMsGain = 4000.0;
            _steerToRadGain = -1.2135;
            _steerToRadOffset = 0.5304;
            _odomTopicName = "/zed2/zed_node/odom";
        }

        bool CCarLikeRobotRosPlugin::LoadParameterJoints(sdf::ElementPtr f_sdf)
        {
            double l_wheelbase = 0;
            double l_axletrack = 0;
            double l_wheelradius = 0;

            if (f_sdf->HasElement("wheelbase"))
            {
                l_wheelbase = f_sdf->Get<double>("wheelbase");
            }

            if (f_sdf->HasElement("axletrack"))
            {
                l_axletrack = f_sdf->Get<double>("axletrack");
            }

            if (f_sdf->HasElement("wheelradius"))
            {
                l_wheelradius = f_sdf->Get<double>("wheelradius");
            }

            sdf::ElementPtr l_speed_wheel_sdf = NULL;

            if (f_sdf->HasElement("speed_wheel_joints"))
            {
                double l_kp_speed = 0;
                double l_ki_speed = 0;
                double l_kd_speed = 0;

                l_speed_wheel_sdf = f_sdf->GetElement("speed_wheel_joints");

                if (l_speed_wheel_sdf->HasElement("kp"))
                    l_kp_speed = l_speed_wheel_sdf->Get<double>("kp");
                if (l_speed_wheel_sdf->HasElement("kd"))
                    l_kd_speed = l_speed_wheel_sdf->Get<double>("kd");
                if (l_speed_wheel_sdf->HasElement("ki"))
                    l_ki_speed = l_speed_wheel_sdf->Get<double>("ki");

                if (l_speed_wheel_sdf->HasElement("front_wheel_joints"))
                {
                    sdf::ElementPtr l_front_wheel_sdf = l_speed_wheel_sdf->GetElement("front_wheel_joints");
                    std::string l_left = l_front_wheel_sdf->Get<std::string>("leftjoint");
                    std::string l_right = l_front_wheel_sdf->Get<std::string>("rightjoint");

                    physics::JointPtr l_rightJoint = this->_model->GetJoint(l_right);
                    physics::JointPtr l_leftJoint = this->_model->GetJoint(l_left);

                    if (l_rightJoint && l_leftJoint)
                    {
                        common::PID l_rightPID = common::PID(l_kp_speed, l_ki_speed, l_kd_speed);
                        common::PID l_leftPID = common::PID(l_kp_speed, l_ki_speed, l_kd_speed);

                        this->_frontWheelSpeedPtr = IWheelsSpeedPtr(new CFrontWheelsSpeed(l_axletrack,
                                                                                          l_wheelbase,
                                                                                          l_wheelradius,
                                                                                          l_rightJoint,
                                                                                          l_leftJoint,
                                                                                          l_rightPID,
                                                                                          l_leftPID,
                                                                                          this->_model));
                    }
                }

                if (l_speed_wheel_sdf->HasElement("rear_wheel_joints"))
                {
                    sdf::ElementPtr l_rear_wheel_sdf = l_speed_wheel_sdf->GetElement("rear_wheel_joints");
                    std::string l_left = l_rear_wheel_sdf->Get<std::string>("leftjoint");
                    std::string l_right = l_rear_wheel_sdf->Get<std::string>("rightjoint");

                    physics::JointPtr l_rightJoint = this->_model->GetJoint(l_right);
                    physics::JointPtr l_leftJoint = this->_model->GetJoint(l_left);

                    if (l_rightJoint && l_leftJoint)
                    {
                        common::PID l_rightPID = common::PID(l_kp_speed, l_ki_speed, l_kd_speed);
                        common::PID l_leftPID = common::PID(l_kp_speed, l_ki_speed, l_kd_speed);

                        this->_rearWheelsSpeedPtr = IWheelsSpeedPtr(new CRearWheelsSpeed(l_axletrack, l_wheelbase, l_wheelradius, l_rightJoint, l_leftJoint, l_rightPID, l_leftPID, this->_model));
                    }
                }
            }

            if (f_sdf->HasElement("steer_wheel_joints"))
            {
                sdf::ElementPtr l_steering_joints_sdf = f_sdf->GetElement("steer_wheel_joints");

                double l_kp_position = 0;
                double l_ki_position = 0;
                double l_kd_position = 0;

                if (l_steering_joints_sdf->HasElement("kp"))
                    l_kp_position = l_steering_joints_sdf->Get<double>("kp");
                if (l_steering_joints_sdf->HasElement("kd"))
                    l_kd_position = l_steering_joints_sdf->Get<double>("kd");
                if (l_steering_joints_sdf->HasElement("ki"))
                    l_ki_position = l_steering_joints_sdf->Get<double>("ki");

                std::string l_left = l_steering_joints_sdf->Get<std::string>("leftjoint");
                std::string l_right = l_steering_joints_sdf->Get<std::string>("rightjoint");

                physics::JointPtr l_rightJoint = this->_model->GetJoint(l_right);
                physics::JointPtr l_leftJoint = this->_model->GetJoint(l_left);

                if (l_leftJoint && l_rightJoint)
                {
                    common::PID l_rightPID = common::PID(l_kp_position, l_ki_position, l_kd_position);
                    common::PID l_leftPID = common::PID(l_kp_position, l_ki_position, l_kd_position);
                    this->_steerWheelsAnglePtr = ISteerWheelsPtr(new CSteerWheelsAngle(l_axletrack, l_wheelbase, l_rightJoint, l_leftJoint, l_rightPID, l_leftPID, this->_model));
                }
            }

            return true;
        }

        void CCarLikeRobotRosPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            if (_parent->GetJointCount() == 0)
            {
                return;
            }
            this->_model = _parent;

            // Initialize ROS 2 Node
            _rosNode = gazebo_ros::Node::Get(_sdf);

            bool isLoaded = this->LoadParameterJoints(_sdf);
            if (!isLoaded)
            {
                return;
            }

            this->f_steer = _sdf->Get<float>("initial_steer");
            this->f_speed = _sdf->Get<float>("initial_speed");
            this->setCommand();

            // Get mapping parameters from SDF or use defaults
            if (_sdf->HasElement("speed_to_ms_gain"))
                _speedToMsGain = _sdf->Get<double>("speed_to_ms_gain");
            if (_sdf->HasElement("steer_to_rad_gain"))
                _steerToRadGain = _sdf->Get<double>("steer_to_rad_gain");
            if (_sdf->HasElement("steer_to_rad_offset"))
                _steerToRadOffset = _sdf->Get<double>("steer_to_rad_offset");
            if (_sdf->HasElement("odom_topic"))
                _odomTopicName = _sdf->Get<std::string>("odom_topic");

            // Get topics from SDF or use defaults
            std::string speedTopic = "/commands/motor/speed";
            if (_sdf->HasElement("speed_topic"))
                speedTopic = _sdf->Get<std::string>("speed_topic");

            std::string steerTopic = "/commands/servo/position";
            if (_sdf->HasElement("steer_topic"))
                steerTopic = _sdf->Get<std::string>("steer_topic");

            // Initialize publishers and broadcasters
            _odomPublisher = _rosNode->create_publisher<nav_msgs::msg::Odometry>(_odomTopicName, 10);
            _tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(_rosNode);

            _messageHandler = std::shared_ptr<CMessageHandler>(new CMessageHandler(this->_model->GetName(), this, _rosNode, speedTopic, steerTopic));

            // Connect to Gazebo update event
            _lastOdomTime = _model->GetWorld()->SimTime();
            _updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&CCarLikeRobotRosPlugin::OnUpdate, this));
        }

        void CCarLikeRobotRosPlugin::OnUpdate()
        {
            common::Time currentTime = _model->GetWorld()->SimTime();

            // Rate-limit to 8 Hz (ZED2 odometry rate) — skip if < 125 ms elapsed
            double dt = (currentTime - _lastOdomTime).Double();
            if (dt < 0.115)
                return;

            _lastOdomTime = currentTime;

            // Pose and velocity
            ignition::math::Pose3d pose = _model->WorldPose();
            ignition::math::Vector3d linVel = _model->WorldLinearVel();
            ignition::math::Vector3d angVel = _model->WorldAngularVel();

            // ── Gaussian noise (ZED2 simulation) ─────────────────────────────
            // Position noise: XY, σ = 1 cm
            double nx = _posNoise(_rng);
            double ny = _posNoise(_rng);

            // Yaw noise: apply as a small quaternion rotation around Z
            double nYaw = _angNoise(_rng);
            ignition::math::Quaterniond qNoise(ignition::math::Vector3d(0, 0, 1), nYaw);
            ignition::math::Quaterniond qNoisy = pose.Rot() * qNoise;
            // ─────────────────────────────────────────────────────────────────

            // Publish Odometry
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = _rosNode->get_clock()->now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = pose.Pos().X() + nx;
            odom.pose.pose.position.y = pose.Pos().Y() + ny;
            odom.pose.pose.position.z = pose.Pos().Z();
            odom.pose.pose.orientation.x = qNoisy.X();
            odom.pose.pose.orientation.y = qNoisy.Y();
            odom.pose.pose.orientation.z = qNoisy.Z();
            odom.pose.pose.orientation.w = qNoisy.W();

            odom.twist.twist.linear.x = linVel.X();
            odom.twist.twist.linear.y = linVel.Y();
            odom.twist.twist.linear.z = linVel.Z();
            odom.twist.twist.angular.x = angVel.X();
            odom.twist.twist.angular.y = angVel.Y();
            odom.twist.twist.angular.z = angVel.Z();

            _odomPublisher->publish(odom);

            // Broadcast TF: odom -> base_link (noisy pose)
            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = odom.child_frame_id;
            tf.transform.translation.x = odom.pose.pose.position.x;
            tf.transform.translation.y = odom.pose.pose.position.y;
            tf.transform.translation.z = odom.pose.pose.position.z;
            tf.transform.rotation = odom.pose.pose.orientation;

            _tfBroadcaster->sendTransform(tf);
        }

        void CCarLikeRobotRosPlugin::setCommand()
        {
            if (this->_rearWheelsSpeedPtr != NULL)
            {
                this->_rearWheelsSpeedPtr->update(this->f_steer, this->f_speed);
            }

            if (this->_frontWheelSpeedPtr != NULL)
            {
                this->_frontWheelSpeedPtr->update(this->f_steer, this->f_speed);
            }

            if (this->_steerWheelsAnglePtr != NULL)
            {
                this->_steerWheelsAnglePtr->update(this->f_steer);
            }
        }

    }; // namespace CarLikeRobot
    GZ_REGISTER_MODEL_PLUGIN(carlikerobot::CCarLikeRobotRosPlugin)
} // namespace gazebo
