/*
 * 文件：gaussian_peak_fitter.hpp
 * 功能：在 FFT 频谱中搜索候选谱峰，并对每个谱峰局部区间做高斯函数拟合。
 *
 * 对应论文公式：
 *   S(f) = A exp( - (f-mu)^2 / (2 sigma_f^2) )
 *
 * 拟合目标：
 *   给定局部频率区间 Omega_p 中的离散频谱点 {f_k, S(f_k)}，求解：
 *
 *   {A_hat, mu_hat, sigma_hat}
 *     = arg min_{A,mu,sigma} sum_{f_k in Omega_p}
 *       [ S(f_k) - A exp( - (f_k-mu)^2/(2 sigma^2) ) ]^2
 *
 * 拟合结果与论文变量对应：
 *   fd = mu_hat                 主频 / 谱峰中心频率
 *   variance = sigma_hat^2       主频峰离散程度 / 频率方差
 *   fL = mu_hat - beta*sigma_hat 左边界
 *   fR = mu_hat + beta*sigma_hat 右边界
 *
 * 为什么使用 Ceres：
 *   上式是非线性最小二乘问题，Ceres 可以自动求导并稳定优化 A、mu、sigma。
 *   为避免 A 或 sigma 在迭代中变成负数，代码实际优化 log_A 和 log_sigma。
 *
 * 与后续 GPR 的接口：
 *   本文件输出的 fd_mu_hz 和 variance_hz2 可以作为：
 *      电机转速 m  ->  主频 fd 及其方差
 *   的训练标签，进入后续共享 GPR 主频预测模块。
 */
#pragma once

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "imu_spectrum_gaussian_fit/fft_spectrum.hpp"

namespace imu_spectrum_gaussian_fit {

/**
 * @brief 单个高斯谱峰拟合结果。
 *
 * 对应论文公式：
 *
 *   S(f) = A exp( - (f - mu)^2 / (2 sigma_f^2) )
 *
 * 其中：
 * - amplitude_A : 峰值幅度 A
 * - mu_hz       : 高斯中心频率 mu，也就是主频 fd
 * - sigma_hz    : 高斯标准差 sigma_f
 * - variance_hz2: sigma_f^2，频率方差，后续可作为主频不确定性/频带宽度指标
 * - f_left_hz   : fL = mu - beta * sigma_f
 * - f_right_hz  : fR = mu + beta * sigma_f
 * - rmse        : 局部频带内拟合残差均方根
 */
struct GaussianPeakParam {
  int peak_id = -1;
  double amplitude_A = 0.0;
  double mu_hz = 0.0;
  double sigma_hz = 0.0;
  double variance_hz2 = 0.0;
  double f_left_hz = 0.0;
  double f_right_hz = 0.0;
  double rmse = 0.0;
  int left_index = 0;
  int right_index = 0;
  int peak_index = 0;
  bool valid = false;
};

/**
 * @brief 高斯残差项。
 *
 * 为保证 A 和 sigma_f 始终为正，这里优化 log_A 和 log_sigma：
 *
 *   A     = exp(log_A)
 *   sigma = exp(log_sigma)
 *
 * 残差定义为：
 *
 *   residual = S(f_k) - A exp(-(f_k - mu)^2 / (2 sigma^2))
 *
 * 与论文最小二乘形式一致。
 */
struct GaussianResidual {
  GaussianResidual(double f_hz, double s) : f_hz_(f_hz), s_(s) {}

  template <typename T>
  bool operator()(const T* const log_A,
                  const T* const mu,
                  const T* const log_sigma,
                  T* residual) const {
    const T A = ceres::exp(log_A[0]);
    const T sigma = ceres::exp(log_sigma[0]);
    const T d = T(f_hz_) - mu[0];
    const T pred = A * ceres::exp(-T(0.5) * d * d / (sigma * sigma));
    residual[0] = T(s_) - pred;
    return true;
  }

  double f_hz_;
  double s_;
};

/**
 * @brief 谱峰检测与局部高斯拟合。
 *
 * 主要功能：
 * 1. 在指定频率范围内寻找局部峰值；
 * 2. 选取幅值最大的 num_peaks 个峰；
 * 3. 对每个峰附近的候选区间 Omega_p 做高斯最小二乘拟合；
 * 4. 输出主频 mu、方差 sigma^2、左右边界 fL/fR；
 * 5. 可根据拟合参数生成每条高斯拟合曲线。
 */
class GaussianPeakFitter {
public:
  struct Options {
    // 需要拟合的谱峰个数。对应你的四个电机，默认取 4。
    int num_peaks = 4;

    // 候选频率范围。默认排除 0~2 Hz 的低频慢变/漂移成分，并保留到 Nyquist 频率。
    double min_freq_hz = 2.0;
    double max_freq_hz = 100.0;

    // 局部峰之间的最小距离，避免同一个谱峰被重复选中。
    double min_peak_distance_hz = 3.0;

    // 每个局部高斯拟合的初始半窗口宽度。
    // 实际窗口还会受相邻峰中点限制，避免两个峰严重混在同一个局部拟合窗口中。
    double fit_half_window_hz = 5.0;

    // Gaussian 95% 频带系数。论文中 beta = 2。
    double beta = 2.0;

    // 峰值最小相对高度，低于 max_amplitude * min_relative_peak_height 的峰会被忽略。
    double min_relative_peak_height = 0.05;

    // Ceres 最大迭代次数。
    int max_solver_iterations = 80;
  };

  explicit GaussianPeakFitter(const Options& options = Options()) : options_(options) {}

  /**
   * @brief 拟合频谱中的多个高斯谱峰。
   */
  std::vector<GaussianPeakParam> fit(const Spectrum1D& spectrum) const {
    // 输入检查：频率数组和幅值数组必须一一对应。
    // frequency_hz[i] 和 amplitude[i] 表示同一个 FFT 频率点处的幅值。
    if (spectrum.frequency_hz.size() != spectrum.amplitude.size() || spectrum.frequency_hz.empty()) {
      throw std::runtime_error("频谱数据为空或长度不一致");
    }

    // 第一步：先做离散谱峰检测。
    // detectPeaks() 返回的是频谱数组中的索引，而不是频率值。
    // 例如 idx=100，则对应频率为 spectrum.frequency_hz[100]。
    std::vector<int> peak_indices = detectPeaks(spectrum);
    std::vector<GaussianPeakParam> results;
    results.reserve(peak_indices.size());

    // 按频率从低到高排序，便于定义相邻峰的局部拟合区间。
    std::sort(peak_indices.begin(), peak_indices.end(), [&](int a, int b) {
      return spectrum.frequency_hz[a] < spectrum.frequency_hz[b];
    });

    for (size_t p = 0; p < peak_indices.size(); ++p) {
      const int peak_idx = peak_indices[p];
      const double peak_f = spectrum.frequency_hz[peak_idx];

      // 第二步：围绕每个候选峰构造局部拟合区间 Omega_p。
      // 初始拟合窗口：peak_f +/- fit_half_window_hz。
      double left_f = peak_f - options_.fit_half_window_hz;
      double right_f = peak_f + options_.fit_half_window_hz;

      // 如果存在相邻选中峰，则用相邻峰中点限制窗口边界，避免两个峰混合拟合。
      if (p > 0) {
        const double prev_f = spectrum.frequency_hz[peak_indices[p - 1]];
        left_f = std::max(left_f, 0.5 * (prev_f + peak_f));
      }
      if (p + 1 < peak_indices.size()) {
        const double next_f = spectrum.frequency_hz[peak_indices[p + 1]];
        right_f = std::min(right_f, 0.5 * (peak_f + next_f));
      }

      left_f = std::max(left_f, options_.min_freq_hz);
      right_f = std::min(right_f, options_.max_freq_hz);

      // 第三步：在局部区间内调用 Ceres 做高斯函数非线性最小二乘拟合。
      GaussianPeakParam param = fitOnePeak(spectrum, peak_idx, left_f, right_f);
      param.peak_id = static_cast<int>(p + 1);
      results.push_back(param);
    }

    return results;
  }

  /**
   * @brief 根据拟合参数生成某个高斯谱峰在全部频率点上的拟合曲线。
   */
  static std::vector<double> evaluateGaussianCurve(const std::vector<double>& freq_hz,
                                                   const GaussianPeakParam& p) {
    std::vector<double> y(freq_hz.size(), 0.0);
    if (!p.valid || p.sigma_hz <= 0.0) return y;
    for (size_t i = 0; i < freq_hz.size(); ++i) {
      const double d = freq_hz[i] - p.mu_hz;
      y[i] = p.amplitude_A * std::exp(-0.5 * d * d / (p.sigma_hz * p.sigma_hz));
    }
    return y;
  }

  /**
   * @brief 生成多个高斯峰的叠加拟合曲线。
   */
  static std::vector<double> evaluateGaussianSum(const std::vector<double>& freq_hz,
                                                 const std::vector<GaussianPeakParam>& peaks) {
    std::vector<double> y(freq_hz.size(), 0.0);
    for (const auto& p : peaks) {
      std::vector<double> yp = evaluateGaussianCurve(freq_hz, p);
      for (size_t i = 0; i < y.size(); ++i) y[i] += yp[i];
    }
    return y;
  }

private:
  Options options_;

  /**
   * @brief 寻找候选局部峰。
   *
   * 实现逻辑：
   * - 只在 [min_freq_hz, max_freq_hz] 范围内查找；
   * - 当前点大于左右邻点，则认为是局部峰候选；
   * - 按幅值从大到小排序；
   * - 加入 min_peak_distance_hz 约束，避免同一峰附近重复取点；
   * - 最多返回 num_peaks 个。
   */
  std::vector<int> detectPeaks(const Spectrum1D& spectrum) const {
    const auto& f = spectrum.frequency_hz;
    const auto& s = spectrum.amplitude;

    // 先统计指定频带内的最大幅值，用于设置相对峰值阈值。
    // 例如 min_relative_peak_height=0.05，表示小于最大峰 5% 的局部峰被认为是噪声峰。
    double max_amp = 0.0;
    for (size_t i = 0; i < s.size(); ++i) {
      if (f[i] >= options_.min_freq_hz && f[i] <= options_.max_freq_hz) {
        max_amp = std::max(max_amp, s[i]);
      }
    }
    const double min_amp = max_amp * options_.min_relative_peak_height;

    // candidates 存放所有满足“比左右邻点都大”的局部峰候选。
    // 这里采用简单局部极大值检测，优点是无需额外依赖，适合实时/离线工程原型。
    std::vector<int> candidates;
    for (size_t i = 1; i + 1 < s.size(); ++i) {
      if (f[i] < options_.min_freq_hz || f[i] > options_.max_freq_hz) continue;
      if (s[i] < min_amp) continue;
      if (s[i] >= s[i - 1] && s[i] >= s[i + 1]) candidates.push_back(static_cast<int>(i));
    }

    // 按幅值从大到小排序，优先选取能量最强的谱峰。
    std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
      return s[a] > s[b];
    });

    // selected 是最终选中的谱峰索引。
    // min_peak_distance_hz 用于避免同一宽峰附近出现多个局部微小峰时被重复选中。
    std::vector<int> selected;
    for (int idx : candidates) {
      bool too_close = false;
      for (int chosen : selected) {
        if (std::abs(f[idx] - f[chosen]) < options_.min_peak_distance_hz) {
          too_close = true;
          break;
        }
      }
      if (!too_close) {
        selected.push_back(idx);
        if (static_cast<int>(selected.size()) >= options_.num_peaks) break;
      }
    }
    return selected;
  }

  GaussianPeakParam fitOnePeak(const Spectrum1D& spectrum,
                               int peak_index,
                               double left_f,
                               double right_f) const {
    const auto& f = spectrum.frequency_hz;
    const auto& s = spectrum.amplitude;

    // 根据传入的左/右频率边界，转换为频谱数组索引区间 [left_idx, right_idx]。
    int left_idx = peak_index;
    int right_idx = peak_index;
    while (left_idx > 0 && f[left_idx] > left_f) --left_idx;
    while (right_idx + 1 < static_cast<int>(f.size()) && f[right_idx] < right_f) ++right_idx;

    // 至少需要几个频点才能稳定拟合 A, mu, sigma。
    if (right_idx - left_idx + 1 < 5) {
      GaussianPeakParam invalid;
      invalid.peak_index = peak_index;
      invalid.left_index = left_idx;
      invalid.right_index = right_idx;
      return invalid;
    }

    // 初值：A 为峰值幅度，mu 为局部最大频率，sigma 初值取窗口宽度的 1/4。
    double A0 = std::max(s[peak_index], 1e-12);
    double mu0 = f[peak_index];
    double sigma0 = std::max((right_f - left_f) / 4.0, 0.2);

    double log_A = std::log(A0);
    double mu = mu0;
    double log_sigma = std::log(sigma0);

    // 构建 Ceres 最小二乘问题。
    // 每一个频率点 (f_i, S_i) 对应一个 residual：
    //   r_i = S_i - A*exp(-(f_i-mu)^2/(2*sigma^2))
    // Ceres 会最小化 sum_i r_i^2。
    ceres::Problem problem;
    for (int i = left_idx; i <= right_idx; ++i) {
      auto* cost = new ceres::AutoDiffCostFunction<GaussianResidual, 1, 1, 1, 1>(
          new GaussianResidual(f[i], s[i]));
      problem.AddResidualBlock(cost, nullptr, &log_A, &mu, &log_sigma);
    }

    // 给 mu 和 sigma 设置合理边界，防止拟合跑出局部峰。
    problem.SetParameterLowerBound(&mu, 0, left_f);
    problem.SetParameterUpperBound(&mu, 0, right_f);
    problem.SetParameterLowerBound(&log_sigma, 0, std::log(0.05));
    problem.SetParameterUpperBound(&log_sigma, 0, std::log(std::max(20.0, right_f - left_f)));

    // 配置求解器。这里参数量很少，DENSE_QR 足够稳定。
    // 如果后续数据量很大，也可以改成其他线性求解器。
    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = options_.max_solver_iterations;
    solver_options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    GaussianPeakParam out;
    out.peak_index = peak_index;
    out.left_index = left_idx;
    out.right_index = right_idx;
    out.amplitude_A = std::exp(log_A);
    out.mu_hz = mu;
    out.sigma_hz = std::exp(log_sigma);
    out.variance_hz2 = out.sigma_hz * out.sigma_hz;
    // 与论文公式对应：
    //   fd = mu
    //   fL = mu - beta*sigma
    //   fR = mu + beta*sigma
    // 当 beta=2 时，若局部峰确实近似高斯，区间 [fL, fR] 可理解为约 95% 频率范围。
    out.f_left_hz = out.mu_hz - options_.beta * out.sigma_hz;
    out.f_right_hz = out.mu_hz + options_.beta * out.sigma_hz;
    out.valid = summary.IsSolutionUsable();

    // 计算局部 RMSE。
    double se = 0.0;
    int count = 0;
    for (int i = left_idx; i <= right_idx; ++i) {
      const double d = f[i] - out.mu_hz;
      const double pred = out.amplitude_A * std::exp(-0.5 * d * d / (out.sigma_hz * out.sigma_hz));
      const double e = s[i] - pred;
      se += e * e;
      ++count;
    }
    out.rmse = count > 0 ? std::sqrt(se / static_cast<double>(count)) : 0.0;
    return out;
  }
};

}  // namespace imu_spectrum_gaussian_fit
