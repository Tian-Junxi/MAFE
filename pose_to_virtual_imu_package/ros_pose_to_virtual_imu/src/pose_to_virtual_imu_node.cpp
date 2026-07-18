#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <string>

// =============================================================================
// pose_to_virtual_imu_node.cpp
// =============================================================================
// 功能：
//   从 ROS 位姿/里程计话题反解“虚拟 IMU / 参考 IMU”，用于 MAFE/FMAFE
//   方法中构造 IMU residual。
//
// 典型流程：
//   /odom or /pose  -->  本节点  -->  /virtual_imu_from_pose
//   /imu/data       -->  与虚拟 IMU 时间对齐  -->  residual
//
// residual 定义：
//   residual_gyro = measured_gyro - virtual_gyro
//   residual_acc  = measured_acc  - virtual_acc
//
// 为什么需要这个节点：
//   MAFE 关注电机/机架引起的高频窄带振动。如果直接分析原始 IMU，里面
//   同时包含真实机动运动和振动噪声。由位姿反推出低频运动对应的 IMU 后，
//   与真实 IMU 相减，可以更突出振动 residual。
//
// 核心算法：
//   1) 缓存一个奇数长度滑动窗口的 pose；
//   2) 以窗口中心为时间零点 tau=0；
//   3) 对位置 p^w(tau) 做局部多项式拟合：
//          p(tau)=c0+c1*tau+c2*tau^2+c3*tau^3
//      在中心处：
//          v^w = c1,  a^w = 2*c2
//   4) 对姿态使用 SO(3) 李代数拟合：
//          phi_i = Log(R_c^T R_i)
//      对 phi_i 关于 tau_i 拟合，一阶项就是中心处机体系角速度 omega^b；
//   5) 由惯导比力公式得到虚拟加速度计输出：
//          f^b = R_wb^T * (a^w - g^w)
//      其中 ROS ENU/z-up 下 g^w=[0,0,-9.80665]^T。
//
// 坐标约定：
//   输入四元数表示 body -> world，即 R_wb。
//   输出 angular_velocity 为 body frame 角速度，单位 rad/s。
//   输出 linear_acceleration 为 body frame 比力，单位 m/s^2。
//
// 注意事项：
//   1) 如果输入 pose 是 LiDAR/camera frame，而不是 body/IMU frame，需要先做外参转换；
//   2) 在线版输出频率基本等于输入 pose 频率；
//   3) 输出时间戳是滑窗中心，因此存在 half-window 延迟；
//   4) 位姿时间戳不递增或断帧时会丢弃/清空窗口。
// =============================================================================

// 单个 pose 样本。
struct PoseSample
{
  ros::Time stamp;            // ROS 原始时间戳，用于输出消息 header
  double t;                   // 相对起始时刻的时间，单位 s，用于数值拟合
  Eigen::Vector3d p_w;         // 世界系位置 p^w
  Eigen::Quaterniond q_wb;     // body -> world 四元数，Eigen 顺序为 w,x,y,z
};

class PoseToVirtualImuNode
{
public:
  PoseToVirtualImuNode()
    : nh_("~"),
      initialized_time_(false),
      csv_opened_(false),
      have_prev_q_(false)
  {
    // -------------------------------------------------------------------------
    // 读取 ROS 参数。
    // 参数文件见 config/pose_to_virtual_imu.yaml。
    // -------------------------------------------------------------------------
    nh_.param<std::string>("pose_type", pose_type_, std::string("odom"));
    nh_.param<std::string>("pose_topic", pose_topic_, std::string("/odom"));
    nh_.param<std::string>("output_imu_topic", output_imu_topic_, std::string("/virtual_imu_from_pose"));

    nh_.param<int>("window_size", window_size_, 21);
    nh_.param<int>("poly_order", poly_order_, 3);
    nh_.param<double>("gravity", gravity_, 9.80665);

    // 一阶低通时间常数。设为 0 表示关闭低通。
    nh_.param<double>("gyro_lowpass_tau", gyro_lowpass_tau_, 0.03);
    nh_.param<double>("acc_lowpass_tau", acc_lowpass_tau_, 0.03);

    // 限幅参数，用于抑制 pose 偶发跳变导致的非物理尖峰。
    nh_.param<bool>("enable_clipping", enable_clipping_, true);
    nh_.param<double>("max_gyro_rad_s", max_gyro_rad_s_, 20.0);
    nh_.param<double>("max_acc_m_s2", max_acc_m_s2_, 80.0);

    // 若相邻 pose 间隔超过该阈值，认为数据断帧，清空窗口重新开始。
    nh_.param<double>("max_pose_gap", max_pose_gap_, 0.2);

    // 是否在输出 sensor_msgs/Imu 中填充 orientation 字段。
    nh_.param<bool>("publish_orientation", publish_orientation_, true);

    // 协方差占位值。若后续不用协方差，可以保持默认。
    nh_.param<double>("gyro_covariance", gyro_covariance_, 1.0e-4);
    nh_.param<double>("acc_covariance", acc_covariance_, 1.0e-2);

    // CSV 调试输出。
    nh_.param<bool>("save_csv", save_csv_, true);
    nh_.param<std::string>("output_csv_path", output_csv_path_,
                           std::string("/tmp/virtual_imu_from_pose.csv"));

    // -------------------------------------------------------------------------
    // 参数合法性修正。
    // window_size 必须为奇数，因为输出取窗口中心。
    // poly_order 至少为 2，因为要取位置二阶导；同时不能太接近窗口长度。
    // -------------------------------------------------------------------------
    if (window_size_ < 5) window_size_ = 5;
    if (window_size_ % 2 == 0) window_size_ += 1;
    if (poly_order_ < 2) poly_order_ = 2;
    if (poly_order_ > window_size_ - 2) poly_order_ = window_size_ - 2;
    half_window_ = window_size_ / 2;

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(output_imu_topic_, 50);

    // 根据 pose_type 选择订阅 nav_msgs/Odometry 或 geometry_msgs/PoseStamped。
    if (pose_type_ == "odom")
    {
      odom_sub_ = nh_.subscribe(pose_topic_, 200,
                                &PoseToVirtualImuNode::odomCallback, this);
      ROS_INFO("[pose_to_virtual_imu] Subscribing nav_msgs/Odometry: %s",
               pose_topic_.c_str());
    }
    else
    {
      pose_sub_ = nh_.subscribe(pose_topic_, 200,
                                &PoseToVirtualImuNode::poseCallback, this);
      ROS_INFO("[pose_to_virtual_imu] Subscribing geometry_msgs/PoseStamped: %s",
               pose_topic_.c_str());
    }

    if (save_csv_) openCsv();

    ROS_INFO("[pose_to_virtual_imu] window_size=%d, half_window=%d, poly_order=%d",
             window_size_, half_window_, poly_order_);
    ROS_INFO("[pose_to_virtual_imu] output topic: %s", output_imu_topic_.c_str());
  }

  ~PoseToVirtualImuNode()
  {
    if (csv_.is_open()) csv_.close();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher imu_pub_;

  std::string pose_type_;
  std::string pose_topic_;
  std::string output_imu_topic_;
  std::string output_csv_path_;

  int window_size_;
  int half_window_;
  int poly_order_;
  double gravity_;
  double gyro_lowpass_tau_;
  double acc_lowpass_tau_;
  bool enable_clipping_;
  double max_gyro_rad_s_;
  double max_acc_m_s2_;
  double max_pose_gap_;
  bool publish_orientation_;
  double gyro_covariance_;
  double acc_covariance_;
  bool save_csv_;

  bool initialized_time_;
  ros::Time t0_stamp_;
  std::deque<PoseSample> buffer_;

  bool have_prev_q_;
  Eigen::Quaterniond prev_q_wb_;

  // 上一次输出，用于一阶低通。
  bool have_prev_output_ = false;
  double prev_output_t_ = 0.0;
  Eigen::Vector3d prev_gyro_b_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_acc_b_  = Eigen::Vector3d::Zero();

  std::ofstream csv_;
  bool csv_opened_;

private:
  // ---------------------------------------------------------------------------
  // nav_msgs/Odometry 回调。
  // 读取 position 和 orientation，并统一交给 pushSample 处理。
  // ---------------------------------------------------------------------------
  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    PoseSample s;
    s.stamp = msg->header.stamp;
    s.p_w = Eigen::Vector3d(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
    s.q_wb = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);
    pushSample(s);
  }

  // ---------------------------------------------------------------------------
  // geometry_msgs/PoseStamped 回调。
  // ---------------------------------------------------------------------------
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    PoseSample s;
    s.stamp = msg->header.stamp;
    s.p_w = Eigen::Vector3d(msg->pose.position.x,
                            msg->pose.position.y,
                            msg->pose.position.z);
    s.q_wb = Eigen::Quaterniond(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);
    pushSample(s);
  }

  // ---------------------------------------------------------------------------
  // 统一处理新 pose 样本。
  // 该函数完成：时间戳检查、相对时间转换、断帧处理、四元数符号连续化、
  // 滑窗缓存更新，以及在窗口满后触发估计。
  // ---------------------------------------------------------------------------
  void pushSample(PoseSample s)
  {
    if (s.stamp.isZero())
    {
      // 如果输入消息没有 stamp，用当前 ROS 时间兜底。
      // 但实际实验中建议输入 pose 一定带有准确时间戳。
      s.stamp = ros::Time::now();
    }

    if (!initialized_time_)
    {
      t0_stamp_ = s.stamp;
      initialized_time_ = true;
    }
    s.t = (s.stamp - t0_stamp_).toSec();

    if (!buffer_.empty())
    {
      double dt = s.t - buffer_.back().t;
      if (dt <= 0.0)
      {
        ROS_WARN_THROTTLE(1.0,
                          "[pose_to_virtual_imu] Non-increasing timestamp, skip sample.");
        return;
      }
      if (dt > max_pose_gap_)
      {
        ROS_WARN("[pose_to_virtual_imu] Pose gap %.3f s > %.3f s, clear buffer.",
                 dt, max_pose_gap_);
        buffer_.clear();
        have_prev_output_ = false;
      }
    }

    // 四元数归一化，防止上游数值误差影响 Log 映射。
    s.q_wb.normalize();

    // 四元数符号连续化。
    // Eigen::Quaterniond::coeffs() 顺序是 [x,y,z,w]，但点积符号判断不受顺序影响。
    if (have_prev_q_ && prev_q_wb_.coeffs().dot(s.q_wb.coeffs()) < 0.0)
    {
      s.q_wb.coeffs() *= -1.0;
    }
    prev_q_wb_ = s.q_wb;
    have_prev_q_ = true;

    buffer_.push_back(s);
    while (static_cast<int>(buffer_.size()) > window_size_)
    {
      buffer_.pop_front();
    }

    if (static_cast<int>(buffer_.size()) == window_size_)
    {
      estimateAndPublish();
    }
  }

  // ---------------------------------------------------------------------------
  // 滑窗估计核心函数。
  // 输出时间戳是窗口中心样本时间，因此这是一个“有 half-window 延迟”的在线估计。
  // ---------------------------------------------------------------------------
  void estimateAndPublish()
  {
    const int N = static_cast<int>(buffer_.size());
    const int c = half_window_;
    const PoseSample& center = buffer_[c];

    // 构建设计矩阵 A=[1,tau,tau^2,...]。
    // tau 使用相对中心时间，可以显著改善数值稳定性。
    Eigen::MatrixXd A(N, poly_order_ + 1);
    for (int i = 0; i < N; ++i)
    {
      double tau = buffer_[i].t - center.t;
      double val = 1.0;
      for (int j = 0; j <= poly_order_; ++j)
      {
        A(i,j) = val;
        val *= tau;
      }
    }

    // 使用 QR 做最小二乘，较普通正规方程更稳定。
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);

    Eigen::Vector3d v_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_w = Eigen::Vector3d::Zero();

    // -----------------------------------------------------------------------
    // 位置拟合：p(tau)=c0+c1*tau+c2*tau^2+...
    // 中心处速度 v=c1，加速度 a=2*c2。
    // -----------------------------------------------------------------------
    for (int axis = 0; axis < 3; ++axis)
    {
      Eigen::VectorXd y(N);
      for (int i = 0; i < N; ++i) y(i) = buffer_[i].p_w(axis);
      Eigen::VectorXd coef = qr.solve(y);
      v_w(axis) = coef(1);
      a_w(axis) = 2.0 * coef(2);
    }

    // -----------------------------------------------------------------------
    // 姿态拟合：phi_i = Log(q_c^{-1} * q_i)。
    // phi_i 是局部旋转向量，定义在中心姿态的切空间中。
    // 对 phi_i 关于 tau_i 拟合，一阶系数即中心处机体系角速度。
    // -----------------------------------------------------------------------
    Eigen::Vector3d gyro_b = Eigen::Vector3d::Zero();
    const Eigen::Quaterniond q_c = center.q_wb;
    const Eigen::Quaterniond q_c_inv = q_c.conjugate();

    Eigen::MatrixXd Phi(N, 3);
    for (int i = 0; i < N; ++i)
    {
      Eigen::Quaterniond q_rel = q_c_inv * buffer_[i].q_wb;
      q_rel.normalize();
      Eigen::Vector3d rv = quatLog(q_rel);
      Phi.row(i) = rv.transpose();
    }

    for (int axis = 0; axis < 3; ++axis)
    {
      Eigen::VectorXd y = Phi.col(axis);
      Eigen::VectorXd coef = qr.solve(y);
      gyro_b(axis) = coef(1);
    }

    // -----------------------------------------------------------------------
    // 世界系加速度转机体系比力。
    // q_c.conjugate() * vector 等价于 R_wb^T * vector。
    // -----------------------------------------------------------------------
    const Eigen::Vector3d g_w(0.0, 0.0, -gravity_);
    Eigen::Vector3d acc_b = q_c.conjugate() * (a_w - g_w);

    // -----------------------------------------------------------------------
    // 一阶低通。
    // 这里不改变时间戳，仅平滑输出值。若 lowpass_tau=0，则关闭对应低通。
    // -----------------------------------------------------------------------
    if (have_prev_output_)
    {
      double dt = std::max(1.0e-6, center.t - prev_output_t_);
      if (gyro_lowpass_tau_ > 0.0)
      {
        double alpha = dt / (gyro_lowpass_tau_ + dt);
        gyro_b = prev_gyro_b_ + alpha * (gyro_b - prev_gyro_b_);
      }
      if (acc_lowpass_tau_ > 0.0)
      {
        double alpha = dt / (acc_lowpass_tau_ + dt);
        acc_b = prev_acc_b_ + alpha * (acc_b - prev_acc_b_);
      }
    }

    // 限幅用于防止偶发异常点污染 residual 频谱。
    if (enable_clipping_)
    {
      clipVector(gyro_b, max_gyro_rad_s_);
      clipVector(acc_b, max_acc_m_s2_);
    }

    prev_output_t_ = center.t;
    prev_gyro_b_ = gyro_b;
    prev_acc_b_ = acc_b;
    have_prev_output_ = true;

    publishImu(center, gyro_b, acc_b);
    if (save_csv_ && csv_opened_) writeCsvLine(center, v_w, a_w, gyro_b, acc_b);
  }

  // ---------------------------------------------------------------------------
  // 四元数 Log 映射。
  // 对单位四元数 q=[cos(theta/2), u*sin(theta/2)]，返回 theta*u。
  // 返回值为旋转向量，单位 rad。
  // ---------------------------------------------------------------------------
  Eigen::Vector3d quatLog(Eigen::Quaterniond q) const
  {
    q.normalize();

    // 取短弧，避免 q 和 -q 对 Log 的影响。
    if (q.w() < 0.0)
    {
      q.coeffs() *= -1.0;
    }

    Eigen::Vector3d v(q.x(), q.y(), q.z());
    double nv = v.norm();
    if (nv < 1.0e-12)
    {
      return 2.0 * v;
    }
    double angle = 2.0 * std::atan2(nv, q.w());
    return angle * v / nv;
  }

  void clipVector(Eigen::Vector3d& v, double max_abs) const
  {
    for (int i = 0; i < 3; ++i)
    {
      if (v(i) >  max_abs) v(i) =  max_abs;
      if (v(i) < -max_abs) v(i) = -max_abs;
    }
  }

  // ---------------------------------------------------------------------------
  // 发布 sensor_msgs/Imu。
  // orientation 可选填输入中心位姿；若不想让下游使用 orientation，可设置
  // publish_orientation=false，此时 orientation_covariance[0] = -1。
  // ---------------------------------------------------------------------------
  void publishImu(const PoseSample& center,
                  const Eigen::Vector3d& gyro_b,
                  const Eigen::Vector3d& acc_b)
  {
    sensor_msgs::Imu msg;
    msg.header.stamp = center.stamp;
    msg.header.frame_id = "virtual_imu";

    if (publish_orientation_)
    {
      msg.orientation.w = center.q_wb.w();
      msg.orientation.x = center.q_wb.x();
      msg.orientation.y = center.q_wb.y();
      msg.orientation.z = center.q_wb.z();
      for (int i = 0; i < 9; ++i) msg.orientation_covariance[i] = 0.0;
    }
    else
    {
      msg.orientation_covariance[0] = -1.0;
    }

    msg.angular_velocity.x = gyro_b.x();
    msg.angular_velocity.y = gyro_b.y();
    msg.angular_velocity.z = gyro_b.z();

    msg.linear_acceleration.x = acc_b.x();
    msg.linear_acceleration.y = acc_b.y();
    msg.linear_acceleration.z = acc_b.z();

    for (int i = 0; i < 9; ++i)
    {
      msg.angular_velocity_covariance[i] = 0.0;
      msg.linear_acceleration_covariance[i] = 0.0;
    }
    msg.angular_velocity_covariance[0] = gyro_covariance_;
    msg.angular_velocity_covariance[4] = gyro_covariance_;
    msg.angular_velocity_covariance[8] = gyro_covariance_;
    msg.linear_acceleration_covariance[0] = acc_covariance_;
    msg.linear_acceleration_covariance[4] = acc_covariance_;
    msg.linear_acceleration_covariance[8] = acc_covariance_;

    imu_pub_.publish(msg);
  }

  // ---------------------------------------------------------------------------
  // 打开 CSV 文件并写表头。
  // CSV 用于离线检查虚拟 IMU 和后续 residual 构造。
  // ---------------------------------------------------------------------------
  void openCsv()
  {
    std::string cmd = "mkdir -p \"" + parentDir(output_csv_path_) + "\"";
    int ret = std::system(cmd.c_str());
    (void)ret;

    csv_.open(output_csv_path_.c_str(), std::ios::out);
    if (!csv_.is_open())
    {
      ROS_ERROR("[pose_to_virtual_imu] Cannot open CSV: %s", output_csv_path_.c_str());
      csv_opened_ = false;
      return;
    }
    csv_opened_ = true;
    csv_ << "stamp,t,px,py,pz,qx,qy,qz,qw,"
         << "vx_w,vy_w,vz_w,ax_w,ay_w,az_w,"
         << "gyro_x_b,gyro_y_b,gyro_z_b,acc_x_b,acc_y_b,acc_z_b\n";
    csv_ << std::setprecision(10);
    ROS_INFO("[pose_to_virtual_imu] CSV output: %s", output_csv_path_.c_str());
  }

  std::string parentDir(const std::string& path) const
  {
    std::size_t pos = path.find_last_of('/');
    if (pos == std::string::npos) return ".";
    if (pos == 0) return "/";
    return path.substr(0, pos);
  }

  void writeCsvLine(const PoseSample& s,
                    const Eigen::Vector3d& v_w,
                    const Eigen::Vector3d& a_w,
                    const Eigen::Vector3d& gyro_b,
                    const Eigen::Vector3d& acc_b)
  {
    csv_ << s.stamp.toSec() << "," << s.t << ","
         << s.p_w.x() << "," << s.p_w.y() << "," << s.p_w.z() << ","
         << s.q_wb.x() << "," << s.q_wb.y() << "," << s.q_wb.z() << "," << s.q_wb.w() << ","
         << v_w.x() << "," << v_w.y() << "," << v_w.z() << ","
         << a_w.x() << "," << a_w.y() << "," << a_w.z() << ","
         << gyro_b.x() << "," << gyro_b.y() << "," << gyro_b.z() << ","
         << acc_b.x() << "," << acc_b.y() << "," << acc_b.z() << "\n";
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_virtual_imu_node");
  PoseToVirtualImuNode node;
  ros::spin();
  return 0;
}
