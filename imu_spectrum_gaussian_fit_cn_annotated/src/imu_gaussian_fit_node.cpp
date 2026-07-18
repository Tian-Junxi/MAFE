/*
 * 文件：imu_gaussian_fit_node.cpp
 * 功能：ROS 离线节点，完成“IMU 数据 -> FFT 频谱 -> 高斯谱峰参数”的完整流程。
 *
 * 输入：
 *   CSV 文件。默认第 1~3 列为陀螺仪，第 4~6 列为加速度计。
 *   采样率默认 200 Hz，也可以通过 launch/yaml 参数修改。
 *
 * 输出：
 *   gyro_peak_params.csv / acc_peak_params.csv
 *     每个谱峰的 A、主频 fd、sigma、方差 sigma^2、左右边界 fL/fR。
 *
 *   gyro_spectrum_fit.csv / acc_spectrum_fit.csv
 *     原始频谱和每个高斯峰曲线数据。
 *
 *   gyro_spectrum_fit.svg / acc_spectrum_fit.svg
 *     带高斯拟合曲线的频谱图。
 *
 * 在你的整体研究流程中的位置：
 *   该节点对应论文“Identification of Vibration-Noise Parameters”部分。
 *   它负责从原始 IMU 测量中提取主频和方差；这些结果随后可与电机转速配对，
 *   作为“共享 GPR 主频估计模型”的训练样本。
 */
#include <ros/ros.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "imu_spectrum_gaussian_fit/csv_reader.hpp"
#include "imu_spectrum_gaussian_fit/fft_spectrum.hpp"
#include "imu_spectrum_gaussian_fit/gaussian_peak_fitter.hpp"
#include "imu_spectrum_gaussian_fit/io_utils.hpp"
#include "imu_spectrum_gaussian_fit/svg_plotter.hpp"

namespace fit = imu_spectrum_gaussian_fit;

/**
 * @brief 创建目录。若目录已存在则不报错。
 */
static void makeDirIfNeeded(const std::string& dir) {
  if (dir.empty()) return;
  struct stat st;
  if (stat(dir.c_str(), &st) == 0) return;
  mkdir(dir.c_str(), 0755);
}

/**
 * @brief 从 ROS 参数服务器读取 vector<int>。
 */
static std::vector<int> getIntVectorParam(ros::NodeHandle& pnh,
                                          const std::string& name,
                                          const std::vector<int>& default_value) {
  std::vector<int> out;
  if (!pnh.getParam(name, out)) return default_value;
  return out;
}

/**
 * @brief 从 CSV 表中提取某个传感器的三轴时间序列。
 *
 * @param table CsvReader 读取出来的二维数组，每一行是一个采样时刻。
 * @param cols  三轴对应的列索引，例如 gyro_cols=[0,1,2]。
 * @param x/y/z 输出三轴数据序列。
 *
 * 这里单独写成函数，是为了让陀螺仪和加速度计复用同一段逻辑。
 * 注意：该函数不检查单位。陀螺仪可以是 rad/s 或 deg/s，加速度计可以是 m/s^2 或 g；
 * 只要同一组实验单位一致，FFT 主频提取不受单位量纲影响，幅值大小会随单位变化。
 */
static void extractThreeAxis(const std::vector<std::vector<double>>& table,
                             const std::vector<int>& cols,
                             std::vector<double>& x,
                             std::vector<double>& y,
                             std::vector<double>& z) {
  if (cols.size() != 3) {
    throw std::runtime_error("三轴列索引必须包含 3 个元素");
  }
  x = fit::CsvReader::getColumn(table, cols[0]);
  y = fit::CsvReader::getColumn(table, cols[1]);
  z = fit::CsvReader::getColumn(table, cols[2]);
}

/**
 * @brief 对某个传感器三轴数据执行：FFT -> 多谱峰检测 -> 高斯拟合 -> 保存参数和图。
 */
static void processOneSensor(const std::string& sensor_name,
                             const std::vector<double>& x,
                             const std::vector<double>& y,
                             const std::vector<double>& z,
                             double sample_rate_hz,
                             bool remove_mean,
                             bool use_hann_window,
                             const fit::GaussianPeakFitter::Options& fitter_options,
                             const std::string& output_dir) {
  ROS_INFO_STREAM("开始处理 " << sensor_name << " 三轴 IMU 数据，样本数 = " << x.size());

  // ------------------------------------------------------------------------
  // 1. 三轴 FFT 合成频谱
  // ------------------------------------------------------------------------
  // 对 x/y/z 三轴分别做 FFT，得到 Sx(f), Sy(f), Sz(f)。
  // 然后使用 sqrt(Sx^2 + Sy^2 + Sz^2) 合成一个总幅值谱。
  // 这样做的原因是：振动可能主要出现在任意一个 IMU 轴向，
  // 合成谱可以更稳健地捕获整体振动主频。
  fit::Spectrum1D spectrum = fit::FftSpectrum::combineThreeAxisSpectrum(
      x, y, z, sample_rate_hz, remove_mean, use_hann_window);

  // ------------------------------------------------------------------------
  // 2. 局部高斯谱峰拟合
  // ------------------------------------------------------------------------
  // GaussianPeakFitter 会先在频谱中找 num_peaks 个局部峰，
  // 再对每个峰的邻域 Omega_p 进行非线性最小二乘拟合：
  //   S(f)=A*exp(-(f-mu)^2/(2*sigma^2))
  // 拟合得到的 mu 即主频 fd，sigma^2 即主频峰方差。
  fit::GaussianPeakFitter fitter(fitter_options);
  std::vector<fit::GaussianPeakParam> peaks = fitter.fit(spectrum);

  // ------------------------------------------------------------------------
  // 3. 保存输出文件
  // ------------------------------------------------------------------------
  const std::string prefix = output_dir + "/" + sensor_name;
  fit::IoUtils::savePeakParamsCsv(prefix + "_peak_params.csv", peaks);
  fit::IoUtils::saveSpectrumFitCsv(prefix + "_spectrum_fit.csv", spectrum, peaks);
  fit::SvgPlotter::plotSpectrumWithGaussianFits(
      prefix + "_spectrum_fit.svg",
      spectrum,
      peaks,
      sensor_name + " FFT spectrum with Gaussian fitted peaks",
      fitter_options.min_freq_hz,
      fitter_options.max_freq_hz);

  // 4. 在终端打印主要结果。
  ROS_INFO_STREAM("========== " << sensor_name << " Gaussian peak parameters ==========");
  for (const auto& p : peaks) {
    ROS_INFO_STREAM("Peak " << p.peak_id
                    << " valid=" << p.valid
                    << " fd=" << p.mu_hz << " Hz"
                    << " sigma=" << p.sigma_hz << " Hz"
                    << " variance=" << p.variance_hz2 << " Hz^2"
                    << " fL=" << p.f_left_hz << " Hz"
                    << " fR=" << p.f_right_hz << " Hz"
                    << " rmse=" << p.rmse);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_gaussian_fit_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    // =====================================================================
    // 1. 读取 ROS 参数
    // =====================================================================
    // 所有参数都可以在 launch 文件或 yaml 文件中配置。
    // 最常用的参数包括：
    //   csv_path       : IMU 数据路径
    //   sample_rate_hz : IMU 采样率，例如 200 Hz
    //   gyro_cols      : 陀螺仪三轴列索引
    //   acc_cols       : 加速度计三轴列索引
    //   num_peaks      : 希望提取的谱峰个数，四电机场景通常取 4
    //   beta           : fL/fR 的频带系数，论文中取 beta=2
    // =====================================================================
    std::string csv_path;
    std::string output_dir;
    double sample_rate_hz;
    bool has_header;
    bool remove_mean;
    bool use_hann_window;

    pnh.param<std::string>("csv_path", csv_path, "");
    pnh.param<std::string>("output_dir", output_dir, "/tmp/imu_spectrum_gaussian_fit");
    pnh.param<double>("sample_rate_hz", sample_rate_hz, 200.0);
    pnh.param<bool>("has_header", has_header, true);
    pnh.param<bool>("remove_mean", remove_mean, true);
    pnh.param<bool>("use_hann_window", use_hann_window, true);

    std::vector<int> gyro_cols = getIntVectorParam(pnh, "gyro_cols", {0, 1, 2});
    std::vector<int> acc_cols = getIntVectorParam(pnh, "acc_cols", {3, 4, 5});

    fit::GaussianPeakFitter::Options fitter_options;
    pnh.param<int>("num_peaks", fitter_options.num_peaks, 4);
    pnh.param<double>("min_freq_hz", fitter_options.min_freq_hz, 2.0);
    pnh.param<double>("max_freq_hz", fitter_options.max_freq_hz, sample_rate_hz / 2.0);
    pnh.param<double>("min_peak_distance_hz", fitter_options.min_peak_distance_hz, 3.0);
    pnh.param<double>("fit_half_window_hz", fitter_options.fit_half_window_hz, 5.0);
    pnh.param<double>("beta", fitter_options.beta, 2.0);
    pnh.param<double>("min_relative_peak_height", fitter_options.min_relative_peak_height, 0.05);
    pnh.param<int>("max_solver_iterations", fitter_options.max_solver_iterations, 80);

    if (csv_path.empty()) {
      throw std::runtime_error("参数 ~csv_path 为空，请指定 IMU CSV 文件路径");
    }
    if (sample_rate_hz <= 0.0) {
      throw std::runtime_error("sample_rate_hz 必须大于 0");
    }
    if (fitter_options.max_freq_hz > sample_rate_hz / 2.0) {
      ROS_WARN_STREAM("max_freq_hz 超过 Nyquist 频率，自动截断为 " << sample_rate_hz / 2.0 << " Hz");
      fitter_options.max_freq_hz = sample_rate_hz / 2.0;
    }

    makeDirIfNeeded(output_dir);

    ROS_INFO_STREAM("读取 IMU CSV: " << csv_path);
    ROS_INFO_STREAM("采样率: " << sample_rate_hz << " Hz");
    ROS_INFO_STREAM("输出目录: " << output_dir);

    // =====================================================================
    // 2. 读取 CSV 数据
    // =====================================================================
    // 读取后的 table 是二维数组：
    //   table[row][col]
    // row 表示采样时刻，col 表示 CSV 列号。
    // 例如默认情况下：
    //   table[i][0], table[i][1], table[i][2] 是陀螺仪三轴；
    //   table[i][3], table[i][4], table[i][5] 是加速度计三轴。
    // =====================================================================
    std::vector<std::vector<double>> table = fit::CsvReader::readNumericCsv(csv_path, has_header);
    if (table.empty()) {
      throw std::runtime_error("CSV 文件没有有效数值数据");
    }

    std::vector<double> gx, gy, gz, ax, ay, az;
    extractThreeAxis(table, gyro_cols, gx, gy, gz);
    extractThreeAxis(table, acc_cols, ax, ay, az);

    // =====================================================================
    // 3. 分别处理陀螺仪和加速度计
    // =====================================================================
    // 两类传感器走完全相同的算法流程，但输出文件名前缀不同：
    //   gyro_* 对应陀螺仪频谱；
    //   acc_*  对应加速度计频谱。
    // =====================================================================
    processOneSensor("gyro", gx, gy, gz,
                     sample_rate_hz, remove_mean, use_hann_window,
                     fitter_options, output_dir);

    processOneSensor("acc", ax, ay, az,
                     sample_rate_hz, remove_mean, use_hann_window,
                     fitter_options, output_dir);

    ROS_INFO_STREAM("处理完成。主要输出文件：");
    ROS_INFO_STREAM("  " << output_dir << "/gyro_peak_params.csv");
    ROS_INFO_STREAM("  " << output_dir << "/gyro_spectrum_fit.csv");
    ROS_INFO_STREAM("  " << output_dir << "/gyro_spectrum_fit.svg");
    ROS_INFO_STREAM("  " << output_dir << "/acc_peak_params.csv");
    ROS_INFO_STREAM("  " << output_dir << "/acc_spectrum_fit.csv");
    ROS_INFO_STREAM("  " << output_dir << "/acc_spectrum_fit.svg");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("IMU Gaussian fitting failed: " << e.what());
    return 1;
  }

  return 0;
}
