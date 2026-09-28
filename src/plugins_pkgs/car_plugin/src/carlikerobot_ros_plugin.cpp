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
            if (plugin->carLike())
            {
                plugin->OnRawErpm(_msg->data);
                return;
            }
            // speed_ms = (erpm_command - speed_to_erpm_offset) / speed_to_erpm_gain
            // Assuming offset is 0 as per real car config
            _robotSetter->f_speed = _msg->data / plugin->_speedToMsGain;
            _robotSetter->setCommand();
        }

        void CMessageHandler::OnSteerCommand(const std_msgs::msg::Float64::SharedPtr _msg)
        {
            CCarLikeRobotRosPlugin *plugin = static_cast<CCarLikeRobotRosPlugin *>(_robotSetter);
            if (plugin->carLike())
            {
                plugin->OnRawServo(_msg->data);
                return;
            }
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

            // Car-like mode: the real car's identified actuators and sensors (car_like.hpp)
            const char *clPath = std::getenv("CAR_LIKE_CONFIG");
            if (clPath != nullptr && std::string(clPath).size() > 0)
            {
                std::string err;
                if (!LoadCarLikeConfig(clPath, _cl, err))
                {
                    RCLCPP_FATAL(_rosNode->get_logger(), "car-like config %s: %s", clPath, err.c_str());
                    throw std::runtime_error("car-like config: " + err);
                }
                _carLike = true;
                _odomTopicName = _cl.pose_topic;
                _clRng.seed(_cl.seed);
                _nLat.configure(_cl.lat_ou_sigma, _cl.lat_ou_tau, _cl.lat_white);
                _nLon.configure(_cl.lon_ou_sigma, _cl.lon_ou_tau, _cl.lon_white);
                _nHdg.configure(_cl.hdg_ou_sigma, _cl.hdg_ou_tau, _cl.hdg_white);
                _nSteer.configure(_cl.steer_dist_sigma, _cl.steer_dist_tau, 0.0);
                _imuPub = _rosNode->create_publisher<sensor_msgs::msg::Imu>(_cl.imu_topic, rclcpp::SensorDataQoS());
                _vescPub = _rosNode->create_publisher<std_msgs::msg::Float64>(_cl.vesc_topic, 10);
                _truthPub = _rosNode->create_publisher<nav_msgs::msg::Odometry>(_cl.truth_topic, 10);
                RCLCPP_INFO(_rosNode->get_logger(),
                            "car-like mode (%s): steer delay %.3f s tau %.3f s disturbance %.4f rad / %.3f s, speed delay %.3f s tau %.3f s (Gazebo's own %.3f s subtracted), "
                            "pose %s %.2f Hz latency %.3f s heading bias %.4f rad, imu %s %.0f Hz, vesc %s %.0f Hz, truth %s",
                            clPath, _cl.steer_delay, _cl.steer_tau, _cl.steer_dist_sigma, _cl.steer_dist_tau, _cl.speed_delay, _cl.speed_tau, _cl.gazebo_speed_tau,
                            _cl.pose_topic.c_str(), _cl.pose_rate, _cl.pose_latency, _cl.pose_heading_bias,
                            _cl.imu_topic.c_str(), _cl.imu_rate, _cl.vesc_topic.c_str(), _cl.vesc_rate, _cl.truth_topic.c_str());
            }

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
            if (_carLike)
            {
                CarLikeUpdate(currentTime.Double());
                return;
            }

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

        void CCarLikeRobotRosPlugin::OnRawServo(double servo)
        {
            std::lock_guard<std::mutex> lk(_cmdMutex);
            _servoQ.push(_simNow.load(), servo);
        }

        void CCarLikeRobotRosPlugin::OnRawErpm(double erpm)
        {
            std::lock_guard<std::mutex> lk(_cmdMutex);
            _erpmQ.push(_simNow.load(), erpm);
        }

        void CCarLikeRobotRosPlugin::CarLikeUpdate(double now)
        {
            _simNow.store(now);
            const double dt = _clLast < 0 ? 0.0 : now - _clLast;
            _clLast = now;

            // ── actuators: dead time -> first-order lag -> static map ─────────
            double servo = _cl.servo_center, erpm = 0.0;
            {
                std::lock_guard<std::mutex> lk(_cmdMutex);
                _servoQ.at(now - _cl.steer_delay, servo);
                _erpmQ.at(now - _cl.speed_delay, erpm);
            }
            const double deltaCmd = (servo - _cl.servo_center) / _cl.servo_per_rad;
            const double vCmd = _cl.speed_gain * erpm / _cl.erpm_per_mps * _cl.pose_per_vesc;
            auto lag = [dt](double state, double target, double tau) {
                return tau > 0 ? state + (1.0 - std::exp(-dt / tau)) * (target - state) : target;
            };
            _deltaState = lag(_deltaState, deltaCmd, _cl.steer_tau);
            _speedState = lag(_speedState, vCmd, std::max(_cl.speed_tau - _cl.gazebo_speed_tau, 0.0));
            // effective steering of the controller's model (wheelbase L_model) -> the Gazebo
            // model's steering angle giving the same yaw rate (its identified effective wheelbase)
            // + the yaw disturbance the car shows beyond this model (servo play, ground), as steering
            if (dt > 0)
                _steerDist = lag(_steerDist, _nSteer.step(dt, _clRng), _cl.steer_dist_lp_tau);
            const double deltaEff = InterpExtrap(_cl.map_cmd, _cl.map_eff, _deltaState) + _steerDist;
            const double deltaGz = std::atan(std::tan(deltaEff) * _cl.gazebo_wheelbase_eff / _cl.model_wheelbase);
            this->f_steer = static_cast<float>(-deltaGz * 180.0 / PI);   // plugin convention: negative = left
            this->f_speed = static_cast<float>(_speedState);
            this->setCommand();

            // ── truth ─────────────────────────────────────────────────────────
            const ignition::math::Pose3d pose = _model->WorldPose();
            const ignition::math::Vector3d linVel = _model->WorldLinearVel();
            const ignition::math::Vector3d angVel = _model->WorldAngularVel();
            const double yaw = pose.Rot().Yaw();
            const double vFwd = linVel.X() * std::cos(yaw) + linVel.Y() * std::sin(yaw);

            auto stamp = _rosNode->get_clock()->now();
            std::normal_distribution<double> n01(0.0, 1.0);

            if (now >= _nextImu)
            {
                _nextImu = std::max(_nextImu + 1.0 / _cl.imu_rate, now);
                sensor_msgs::msg::Imu imu;
                imu.header.stamp = stamp;
                imu.header.frame_id = "zed2_imu_link";
                const ignition::math::Quaterniond q = pose.Rot();
                imu.orientation.x = q.X(); imu.orientation.y = q.Y(); imu.orientation.z = q.Z(); imu.orientation.w = q.W();
                imu.angular_velocity.z = angVel.Z() + _cl.gyro_std * n01(_clRng);
                const ignition::math::Vector3d acc = pose.Rot().RotateVectorReverse(_model->WorldLinearAccel());
                imu.linear_acceleration.x = acc.X();
                imu.linear_acceleration.y = acc.Y();
                imu.linear_acceleration.z = acc.Z() + 9.81;
                _imuPub->publish(imu);
            }
            if (now >= _nextVesc)
            {
                _nextVesc = std::max(_nextVesc + 1.0 / _cl.vesc_rate, now);
                std_msgs::msg::Float64 m;
                m.data = vFwd / _cl.pose_per_vesc * _cl.erpm_per_mps;   // what the VESC reports (ERPM)
                _vescPub->publish(m);
            }
            if (now >= _nextTruth)
            {
                _nextTruth = std::max(_nextTruth + 1.0 / _cl.truth_rate, now);
                nav_msgs::msg::Odometry t;
                t.header.stamp = stamp;
                t.header.frame_id = "odom";
                t.child_frame_id = "base_link";
                t.pose.pose.position.x = pose.Pos().X();
                t.pose.pose.position.y = pose.Pos().Y();
                t.pose.pose.position.z = pose.Pos().Z();
                t.pose.pose.orientation.x = pose.Rot().X(); t.pose.pose.orientation.y = pose.Rot().Y();
                t.pose.pose.orientation.z = pose.Rot().Z(); t.pose.pose.orientation.w = pose.Rot().W();
                t.twist.twist.linear.x = linVel.X(); t.twist.twist.linear.y = linVel.Y();
                t.twist.twist.angular.z = angVel.Z();
                _truthPub->publish(t);
            }
            // localisation: a scan every 1/rate (+ period jitter); its pose is the true pose at
            // the scan instant plus noise, published latency (+ jitter) later, in order
            if (now >= _nextScan)
            {
                _nextScan = now + std::max(1.0 / _cl.pose_rate + _cl.pose_period_jitter * n01(_clRng), 0.02);
                const double dts = _lastScan < 0 ? 1.0 / _cl.pose_rate : now - _lastScan;
                _lastScan = now;
                const double eLat = _nLat.step(dts, _clRng), eLon = _nLon.step(dts, _clRng), eHdg = _nHdg.step(dts, _clRng);
                PendingPose p;
                p.x = pose.Pos().X() + eLon * std::cos(yaw) - eLat * std::sin(yaw);
                p.y = pose.Pos().Y() + eLon * std::sin(yaw) + eLat * std::cos(yaw);
                p.yaw = yaw - _cl.pose_heading_bias + eHdg;   // measured: course - PF heading = heading_bias
                p.pub = std::max(now + std::max(_cl.pose_latency + _cl.pose_latency_jitter * n01(_clRng), 0.0),
                                 _pending.empty() ? _lastPub : _pending.back().pub);
                _pending.push_back(p);
            }
            while (!_pending.empty() && _pending.front().pub <= now)
            {
                _lastPub = _pending.front().pub;
                PublishPose(_pending.front());
                _pending.pop_front();
            }
        }

        void CCarLikeRobotRosPlugin::PublishPose(const PendingPose &p)
        {
            const double x = p.x, y = p.y, yaw = p.yaw;
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = _rosNode->get_clock()->now();   // the car's PF stamps on publication
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            const ignition::math::Quaterniond q(0.0, 0.0, yaw);
            odom.pose.pose.orientation.x = q.X(); odom.pose.pose.orientation.y = q.Y();
            odom.pose.pose.orientation.z = q.Z(); odom.pose.pose.orientation.w = q.W();
            // the car's /pf/pose/odom carries no twist
            _odomPublisher->publish(odom);

            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = odom.child_frame_id;
            tf.transform.translation.x = x;
            tf.transform.translation.y = y;
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
