#include "car_like.hpp"

#include <fstream>
#include <sstream>

#include "rapidjson/document.h"

namespace gazebo
{
    namespace carlikerobot
    {
        namespace
        {
            // "a.b.c" in a JSON document; false (and err) if missing or not a number
            bool num(const rapidjson::Value &root, const char *path, double &out, std::string &err)
            {
                const rapidjson::Value *v = &root;
                std::stringstream ss(path);
                std::string key;
                while (std::getline(ss, key, '.'))
                {
                    if (!v->IsObject() || !v->HasMember(key.c_str()))
                    {
                        err = std::string("missing ") + path;
                        return false;
                    }
                    v = &(*v)[key.c_str()];
                }
                if (!v->IsNumber())
                {
                    err = std::string("not a number: ") + path;
                    return false;
                }
                out = v->GetDouble();
                return true;
            }

            bool str(const rapidjson::Value &root, const char *obj, const char *key, std::string &out, std::string &err)
            {
                if (!root.HasMember(obj) || !root[obj].HasMember(key) || !root[obj][key].IsString())
                {
                    err = std::string("missing ") + obj + "." + key;
                    return false;
                }
                out = root[obj][key].GetString();
                return true;
            }

            bool arr(const rapidjson::Value &root, const char *obj, const char *key, std::vector<double> &out, std::string &err)
            {
                if (!root.HasMember(obj) || !root[obj].HasMember(key) || !root[obj][key].IsArray())
                {
                    err = std::string("missing ") + obj + "." + key;
                    return false;
                }
                out.clear();
                for (const auto &e : root[obj][key].GetArray())
                    out.push_back(e.GetDouble());
                return true;
            }
        }

        bool LoadCarLikeConfig(const std::string &path, CarLikeConfig &c, std::string &err)
        {
            std::ifstream f(path);
            if (!f)
            {
                err = "cannot open " + path;
                return false;
            }
            std::stringstream buf;
            buf << f.rdbuf();
            rapidjson::Document d;
            d.Parse(buf.str().c_str());
            if (d.HasParseError() || !d.IsObject())
            {
                err = "JSON parse error in " + path;
                return false;
            }
            double seed = 0;
            bool ok =
                num(d, "command.erpm_per_mps", c.erpm_per_mps, err) &&
                num(d, "command.pose_per_vesc", c.pose_per_vesc, err) &&
                num(d, "command.speed_gain", c.speed_gain, err) &&
                num(d, "command.speed_delay_s", c.speed_delay, err) &&
                num(d, "command.speed_tau_s", c.speed_tau, err) &&
                num(d, "command.servo_center", c.servo_center, err) &&
                num(d, "command.servo_per_rad", c.servo_per_rad, err) &&
                num(d, "command.steer_delay_s", c.steer_delay, err) &&
                num(d, "command.steer_tau_s", c.steer_tau, err) &&
                arr(d, "command", "steer_map_cmd_rad", c.map_cmd, err) &&
                arr(d, "command", "steer_map_eff_rad", c.map_eff, err) &&
                num(d, "command.model_wheelbase_m", c.model_wheelbase, err) &&
                num(d, "gazebo.wheelbase_eff_m", c.gazebo_wheelbase_eff, err) &&
                num(d, "gazebo.speed_tau_s", c.gazebo_speed_tau, err) &&
                num(d, "disturbance.steer_ou_sigma_rad", c.steer_dist_sigma, err) &&
                num(d, "disturbance.steer_ou_tau_s", c.steer_dist_tau, err) &&
                num(d, "disturbance.steer_lowpass_tau_s", c.steer_dist_lp_tau, err) &&
                str(d, "pose", "topic", c.pose_topic, err) &&
                num(d, "pose.rate_hz", c.pose_rate, err) &&
                num(d, "pose.latency_s", c.pose_latency, err) &&
                num(d, "pose.heading_bias_rad", c.pose_heading_bias, err) &&
                num(d, "pose.period_jitter_s", c.pose_period_jitter, err) &&
                num(d, "pose.latency_jitter_s", c.pose_latency_jitter, err) &&
                num(d, "pose.lateral.ou_sigma_m", c.lat_ou_sigma, err) &&
                num(d, "pose.lateral.ou_tau_s", c.lat_ou_tau, err) &&
                num(d, "pose.lateral.white_m", c.lat_white, err) &&
                num(d, "pose.longitudinal.ou_sigma_m", c.lon_ou_sigma, err) &&
                num(d, "pose.longitudinal.ou_tau_s", c.lon_ou_tau, err) &&
                num(d, "pose.longitudinal.white_m", c.lon_white, err) &&
                num(d, "pose.heading.ou_sigma_rad", c.hdg_ou_sigma, err) &&
                num(d, "pose.heading.ou_tau_s", c.hdg_ou_tau, err) &&
                num(d, "pose.heading.white_rad", c.hdg_white, err) &&
                str(d, "imu", "topic", c.imu_topic, err) &&
                num(d, "imu.rate_hz", c.imu_rate, err) &&
                num(d, "imu.gyro_std_radps", c.gyro_std, err) &&
                str(d, "vesc", "topic", c.vesc_topic, err) &&
                num(d, "vesc.rate_hz", c.vesc_rate, err) &&
                str(d, "ground_truth", "topic", c.truth_topic, err) &&
                num(d, "ground_truth.rate_hz", c.truth_rate, err) &&
                num(d, "seed", seed, err);
            if (!ok)
                return false;
            c.seed = static_cast<unsigned>(seed);
            if (c.map_cmd.size() < 2 || c.map_cmd.size() != c.map_eff.size())
            {
                err = "steer_map_cmd_rad / steer_map_eff_rad: need >= 2 knots of equal count";
                return false;
            }
            if (c.erpm_per_mps <= 0 || c.servo_per_rad == 0 || c.pose_rate <= 0 || c.imu_rate <= 0 ||
                c.vesc_rate <= 0 || c.truth_rate <= 0 || c.gazebo_wheelbase_eff <= 0 || c.model_wheelbase <= 0 ||
                c.lat_ou_tau <= 0 || c.lon_ou_tau <= 0 || c.hdg_ou_tau <= 0 || c.steer_dist_tau <= 0)
            {
                err = "non-positive rate, gain, wheelbase or time constant";
                return false;
            }
            return true;
        }

        double InterpExtrap(const std::vector<double> &x, const std::vector<double> &y, double q)
        {
            const size_t n = x.size();
            size_t i = 0;
            if (q <= x[0])
                i = 0;
            else if (q >= x[n - 1])
                i = n - 2;
            else
                while (i + 1 < n - 1 && q > x[i + 1])
                    ++i;
            const double s = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
            return y[i] + s * (q - x[i]);
        }
    }
}
