// Car-like mode of the car plugin: the real car's actuators and sensors, as identified
// from its rosbags (RC_Car_Codes: src/integral_rl_controller/scripts/identify_car_dynamics.py,
// results/identification/car_model.json). Every number comes from config/car_like.json;
// nothing here has a tuned default.
//
//   commands   /commands/servo/position, /commands/motor/speed (the car's units)
//              -> dead time -> first-order lag -> static steering map -> Gazebo joints
//   sensors    /pf/pose/odom          particle-filter-like pose: rate, latency, heading bias,
//                                      OU + white noise (lateral, longitudinal, heading), no twist
//              /zed2/zed_node/imu/data yaw rate + white noise
//              <vesc topic>            measured ERPM (std_msgs/Float64; RC_Car_Codes'
//                                      sim_vesc_bridge republishes it as /sensors/core)
//              <ground truth topic>    true pose/twist of the same point, for evaluation
#pragma once

#include <cmath>
#include <deque>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace gazebo
{
    namespace carlikerobot
    {
        struct CarLikeConfig
        {
            // command side
            double erpm_per_mps = 0;      // controller's real.speed_cmd_per_mps
            double pose_per_vesc = 1;     // pose-unit distance / VESC-derived distance
            double speed_gain = 1;        // VESC ERPM / commanded ERPM (steady state)
            double speed_delay = 0, speed_tau = 0;
            double servo_center = 0.5, servo_per_rad = 1;
            double steer_delay = 0, steer_tau = 0;
            std::vector<double> map_cmd, map_eff;  // piecewise linear delta_cmd -> delta_eff
            double model_wheelbase = 0.28;          // L of delta_eff (controller's model)
            double gazebo_wheelbase_eff = 0;         // identified effective L of this Gazebo model
            double gazebo_speed_tau = 0;             // Gazebo's own wheel-speed lag (subtracted)
            double steer_dist_sigma = 0, steer_dist_tau = 1;  // OU disturbance on the effective steering,
            double steer_dist_lp_tau = 0;                      // then a first-order low-pass (2nd-order roll-off)
            // sensors
            std::string pose_topic, imu_topic, vesc_topic, truth_topic;
            double pose_rate = 0, pose_latency = 0, pose_heading_bias = 0;
            double pose_period_jitter = 0, pose_latency_jitter = 0;   // s (std): scan period, publication delay
            double pose_position_extra_latency = 0;   // s: position older than the heading
            double compute_delay = 0;                 // s: controller, pose arrival -> command (added to both commands)
            double lat_ou_sigma = 0, lat_ou_tau = 1, lat_white = 0;
            double lon_ou_sigma = 0, lon_ou_tau = 1, lon_white = 0;
            double hdg_ou_sigma = 0, hdg_ou_tau = 1, hdg_white = 0;
            double imu_rate = 0, gyro_std = 0;
            double vesc_rate = 0, truth_rate = 0;
            unsigned seed = 0;
        };

        bool LoadCarLikeConfig(const std::string &path, CarLikeConfig &cfg, std::string &err);

        // Piecewise-linear map with linear extrapolation beyond the outer knots.
        double InterpExtrap(const std::vector<double> &x, const std::vector<double> &y, double q);

        // Commands with their arrival (sim) time; value in effect at time t - delay.
        class DelayLine
        {
        public:
            void push(double t, double v) { _q.emplace_back(t, v); }
            // latest value that arrived at or before t (keeps it as the new front)
            bool at(double t, double &v)
            {
                while (_q.size() > 1 && _q[1].first <= t)
                    _q.pop_front();
                if (_q.empty() || _q.front().first > t)
                    return false;
                v = _q.front().second;
                return true;
            }

        private:
            std::deque<std::pair<double, double>> _q;
        };

        // Ornstein-Uhlenbeck (stationary std sigma, time constant tau) + white noise.
        class OuNoise
        {
        public:
            void configure(double sigma, double tau, double white) { _s = sigma; _tau = tau; _w = white; }
            double step(double dt, std::mt19937 &rng)
            {
                std::normal_distribution<double> n(0.0, 1.0);
                if (!_init) { _x = _s * n(rng); _init = true; }
                else if (_s > 0) {
                    const double a = std::exp(-dt / _tau);
                    _x = a * _x + std::sqrt(1.0 - a * a) * _s * n(rng);
                }
                return _x + _w * n(rng);
            }

        private:
            double _s = 0, _tau = 1, _w = 0, _x = 0;
            bool _init = false;
        };

        // A pose sampled at the scan instant, published at `pub` (scan + latency + jitter).
        struct PendingPose { double pub, x, y, yaw; };
        struct TruthXY { double t, x, y; };
    }
}
