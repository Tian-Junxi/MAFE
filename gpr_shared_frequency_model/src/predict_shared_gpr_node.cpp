/*
 * 节点：predict_shared_gpr_node
 *
 * 功能：
 *   加载训练好的共享 GPR 模型，对输入的四个电机转速实时估计主频及方差。
 *
 * 输入话题：
 *   /motor_speeds, std_msgs/Float64MultiArray
 *   data = [m1, m2, m3, m4]
 *
 * 输出话题：
 *   /gyro_predicted_main_frequency, std_msgs/Float64MultiArray
 *   /acc_predicted_main_frequency,  std_msgs/Float64MultiArray
 *
 * 输出格式：
 *   data = [mean1, var1, mean2, var2, mean3, var3, mean4, var4]
 *
 * 说明：
 *   四个电机共用同一个 GPR 模型。也就是说，预测时不是用四个模型，
 *   而是对同一个模型分别输入 m1,m2,m3,m4，输出四组主频均值和方差。
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

#include "gpr_shared_frequency_model/gpr1d_matern.hpp"

namespace gpr_shared_frequency_model {

// ============================== 时间统计工具 ==============================
// 用 steady_clock 统计预测耗时，单位为毫秒。该时钟是单调时钟，
// 不会因为系统时间同步或修改而跳变，适合做实时预测耗时统计。
using SteadyClock = std::chrono::steady_clock;

static double elapsedMs(const SteadyClock::time_point& start,
                        const SteadyClock::time_point& end = SteadyClock::now()) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

static std::string defaultOutputDir() {
  const char* home = std::getenv("HOME");
  if (home && std::string(home).size() > 0) {
    return std::string(home) + "/catkin_ws/output";
  }
  return "output";
}

static std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (a.back() == '/') return a + b;
  return a + "/" + b;
}

class SharedGprPredictNode {
public:
  SharedGprPredictNode() : nh_("~") {
    std::string model_dir;
    std::string gyro_model_name;
    std::string acc_model_name;

    nh_.param<std::string>("model_dir", model_dir, defaultOutputDir());
    nh_.param<bool>("print_timing", print_timing_, true);
    nh_.param<double>("timing_log_throttle_sec", timing_log_throttle_sec_, 0.0);
    nh_.param<std::string>("gyro_model_name", gyro_model_name, "gyro_gpr_model.gpr");
    nh_.param<std::string>("acc_model_name", acc_model_name, "acc_gpr_model.gpr");

    const std::string gyro_path = joinPath(model_dir, gyro_model_name);
    const std::string acc_path = joinPath(model_dir, acc_model_name);

    gyro_model_.load(gyro_path);
    acc_model_.load(acc_path);

    ROS_INFO_STREAM("Loaded gyro GPR model: " << gyro_path
                    << ", samples=" << gyro_model_.sampleSize());
    ROS_INFO_STREAM("Loaded acc GPR model: " << acc_path
                    << ", samples=" << acc_model_.sampleSize());
    ROS_INFO_STREAM("Prediction timing log: print_timing=" << (print_timing_ ? "true" : "false")
                    << ", timing_log_throttle_sec=" << timing_log_throttle_sec_);

    ros::NodeHandle root_nh;
    sub_speed_ = root_nh.subscribe("/motor_speeds", 10, &SharedGprPredictNode::speedCallback, this);
    pub_gyro_ = root_nh.advertise<std_msgs::Float64MultiArray>("/gyro_predicted_main_frequency", 10);
    pub_acc_  = root_nh.advertise<std_msgs::Float64MultiArray>("/acc_predicted_main_frequency", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_speed_;
  ros::Publisher pub_gyro_;
  ros::Publisher pub_acc_;

  GPR1DMatern gyro_model_;
  GPR1DMatern acc_model_;

  // 是否在终端打印每次预测耗时。若输入频率很高，可在 launch 中设为 false。
  bool print_timing_ = true;

  // 耗时日志节流时间，单位 s。0 表示每次收到 /motor_speeds 都打印。
  double timing_log_throttle_sec_ = 0.0;
  ros::Time last_timing_log_time_;

  void speedCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    const auto callback_begin = SteadyClock::now();

    if (msg->data.size() < 4) {
      ROS_WARN_THROTTLE(1.0, "Need at least 4 motor speeds: data=[m1,m2,m3,m4].");
      return;
    }

    std::vector<double> speeds(4);
    for (int i = 0; i < 4; ++i) {
      speeds[i] = msg->data[static_cast<size_t>(i)];
    }

    // 分别统计 gyro 模型预测耗时和 acc 模型预测耗时。
    // 这里包含 GPR 后验均值/方差计算和 ROS 消息发布前的数据组织时间。
    const auto gyro_begin = SteadyClock::now();
    publishPrediction(gyro_model_, speeds, pub_gyro_);
    const double gyro_predict_ms = elapsedMs(gyro_begin);

    const auto acc_begin = SteadyClock::now();
    publishPrediction(acc_model_, speeds, pub_acc_);
    const double acc_predict_ms = elapsedMs(acc_begin);

    const double callback_total_ms = elapsedMs(callback_begin);

    if (print_timing_) {
      const ros::Time now = ros::Time::now();
      const bool should_print =
          timing_log_throttle_sec_ <= 0.0 ||
          last_timing_log_time_.isZero() ||
          (now - last_timing_log_time_).toSec() >= timing_log_throttle_sec_;
      if (should_print) {
        ROS_INFO_STREAM("[Timing] GPR prediction: gyro=" << gyro_predict_ms << " ms"
                        << ", acc=" << acc_predict_ms << " ms"
                        << ", callback_total=" << callback_total_ms << " ms"
                        << ", input_speeds=[" << speeds[0] << ", " << speeds[1] << ", "
                        << speeds[2] << ", " << speeds[3] << "]");
        last_timing_log_time_ = now;
      }
    }
  }

  void publishPrediction(const GPR1DMatern& model,
                         const std::vector<double>& speeds,
                         ros::Publisher& pub) {
    const std::vector<Prediction1D> preds = model.predict(speeds);

    std_msgs::Float64MultiArray out;
    out.data.reserve(preds.size() * 2);
    for (const auto& p : preds) {
      out.data.push_back(p.mean);      // 主频均值，Hz
      out.data.push_back(p.variance);  // 主频方差，Hz^2
    }
    pub.publish(out);
  }
};

}  // namespace gpr_shared_frequency_model

int main(int argc, char** argv) {
  ros::init(argc, argv, "predict_shared_gpr_node");
  try {
    gpr_shared_frequency_model::SharedGprPredictNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("predict_shared_gpr_node failed: " << e.what());
    return 1;
  }
  return 0;
}

