#pragma once

/*
 * 文件：gpr1d_matern.hpp
 * 功能：一维高斯过程回归（Gaussian Process Regression, GPR）模型。
 *
 * 本文件面向“电机转速 -> 振动主频”的建模任务：
 *   输入 x：单个电机转速 m，例如 ESC RPM 或归一化转速；
 *   输出 y：该转速诱导的 IMU 振动主频 f_d；
 *   输出不确定性：GPR 后验预测方差 sigma_f^2。
 *
 * 重要说明：
 *   1. 这是“共享单电机 GPR 模型”。四个电机不分别训练四个模型，
 *      而是把四个电机的样本全部展开后汇总到同一个训练集中，训练一个
 *      g(m)。预测时对 m1,m2,m3,m4 分别调用同一个模型，从而得到
 *      4 个主频及其方差。
 *
 *   2. 核函数采用论文中常见的 Matern 3/2 核：
 *        k(m,m') = sigma_f^2 (1 + sqrt(3)|m-m'|/l)
 *                  exp(-sqrt(3)|m-m'|/l)
 *      其中 l 是长度尺度，sigma_f^2 是信号方差。
 *
 *   3. 支持“异方差观测噪声”：如果 Excel/CSV 中有每个谱峰的方差列，
 *      可以把这些方差作为每个训练样本的观测噪声方差加入 K 的对角线。
 *      这样某些峰提取不确定性大时，对模型训练的影响会自动减弱。
 *
 *   4. 模型文件保存为 .gpr，但内部是 YAML 文本格式，便于 ROS/C++ 读取，
 *      也便于人工检查。后缀 .gpr 只是工程命名，并不是 GPR 强制格式。
 */

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace gpr_shared_frequency_model {

// GPR 单点预测结果：mean 为主频均值，variance 为主频预测方差。
struct Prediction1D {
  double mean = 0.0;
  double variance = 0.0;
};

class GPR1DMatern {
public:
  GPR1DMatern() = default;

  /*
   * 训练 GPR 模型。
   *
   * 参数：
   *   x_raw: 原始输入样本，电机转速，例如 [221, 265, 307, ...]
   *   y_raw: 原始输出样本，匹配后的主频，例如 [50, 83, 83, ...]
   *   obs_noise_var_raw: 每个主频样本的观测方差。
   *                      若没有可靠方差，可传空向量，内部按 0 处理，
   *                      并由 base_noise_var_ 表示整体残差噪声。
   *   optimize_hyperparams: 是否用网格搜索优化核函数超参数。
   */
  void train(const std::vector<double>& x_raw,
             const std::vector<double>& y_raw,
             const std::vector<double>& obs_noise_var_raw = {},
             bool optimize_hyperparams = true) {
    if (x_raw.size() != y_raw.size() || x_raw.size() < 3) {
      throw std::runtime_error("GPR1DMatern::train: x/y size mismatch or too few samples.");
    }

    train_x_raw_ = x_raw;
    train_y_raw_ = y_raw;

    if (obs_noise_var_raw.empty()) {
      train_obs_noise_var_raw_.assign(x_raw.size(), 0.0);
    } else {
      if (obs_noise_var_raw.size() != x_raw.size()) {
        throw std::runtime_error("GPR1DMatern::train: observation noise variance size mismatch.");
      }
      train_obs_noise_var_raw_ = obs_noise_var_raw;
    }

    computeNormalization();
    buildNormalizedVectors();

    // 输出已经归一化，signal_var=1 通常是合理初值。
    signal_var_ = 1.0;
    length_scale_ = 1.0;
    base_noise_var_ = 1e-3;

    if (optimize_hyperparams) {
      gridSearchHyperparams();
    }

    rebuildFromStoredData();
  }

  /*
   * 对单个电机转速进行预测。
   *
   * 返回：
   *   mean: 预测主频，单位与训练数据一致，一般为 Hz；
   *   variance: GPR 后验预测方差，单位为 Hz^2。
   *
   * 注意：这里输出的是 latent function 的后验方差，表示模型对主频函数
   * g(m) 的不确定性。若你想得到“观测主频”的方差，可额外叠加测量噪声项。
   */
  Prediction1D predict(double x_raw) const {
    if (!is_trained_) {
      throw std::runtime_error("GPR1DMatern::predict: model is not trained or loaded.");
    }

    const double x = normalizeX(x_raw);
    const int n = static_cast<int>(train_x_norm_.size());

    // k_* = [k(x*, x_1), ..., k(x*, x_N)]^T，不包含白噪声项。
    Eigen::VectorXd k_star(n);
    for (int i = 0; i < n; ++i) {
      k_star(i) = kernelNoNoise(x, train_x_norm_(i));
    }

    // 预测均值：mu_* = k_*^T K^{-1} y = k_*^T alpha。
    const double mean_norm = k_star.dot(alpha_);

    // 预测方差：var_* = k(x*,x*) - k_*^T K^{-1} k_*。
    // 用 Cholesky 分解 K = L L^T，先解 L v = k_*。
    Eigen::VectorXd v = L_.triangularView<Eigen::Lower>().solve(k_star);
    double var_norm = signal_var_ - v.squaredNorm();
    if (var_norm < 1e-12) var_norm = 1e-12;

    Prediction1D out;
    out.mean = y_mean_ + y_std_ * mean_norm;
    out.variance = y_std_ * y_std_ * var_norm;
    return out;
  }

  // 批量预测：对四个电机转速或更多转速逐个调用同一个共享模型。
  std::vector<Prediction1D> predict(const std::vector<double>& x_raw) const {
    std::vector<Prediction1D> out;
    out.reserve(x_raw.size());
    for (double x : x_raw) {
      out.push_back(predict(x));
    }
    return out;
  }

  /*
   * 保存模型。
   *
   * path 可以是 .gpr、.yaml、.txt 等。当前内部采用 YAML 文本格式。
   * 保存内容包括：训练样本、观测噪声、归一化参数、核函数超参数。
   */
  void save(const std::string& path) const {
    if (!is_trained_) {
      throw std::runtime_error("GPR1DMatern::save: model is not trained.");
    }

    YAML::Emitter y;
    y << YAML::BeginMap;
    y << YAML::Key << "model_type" << YAML::Value << "GPR1D_Matern32_SharedMotorModel";
    y << YAML::Key << "x_mean" << YAML::Value << x_mean_;
    y << YAML::Key << "x_std" << YAML::Value << x_std_;
    y << YAML::Key << "y_mean" << YAML::Value << y_mean_;
    y << YAML::Key << "y_std" << YAML::Value << y_std_;
    y << YAML::Key << "signal_var" << YAML::Value << signal_var_;
    y << YAML::Key << "length_scale" << YAML::Value << length_scale_;
    y << YAML::Key << "base_noise_var" << YAML::Value << base_noise_var_;

    y << YAML::Key << "train_x_raw" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (double v : train_x_raw_) y << v;
    y << YAML::EndSeq;

    y << YAML::Key << "train_y_raw" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (double v : train_y_raw_) y << v;
    y << YAML::EndSeq;

    y << YAML::Key << "train_obs_noise_var_raw" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (double v : train_obs_noise_var_raw_) y << v;
    y << YAML::EndSeq;

    y << YAML::EndMap;

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      throw std::runtime_error("Cannot write GPR model file: " + path);
    }
    ofs << y.c_str();
  }

  // 从 .gpr/.yaml 文件加载模型。加载后会自动重建 K、L、alpha。
  void load(const std::string& path) {
    YAML::Node node = YAML::LoadFile(path);
    train_x_raw_ = node["train_x_raw"].as<std::vector<double>>();
    train_y_raw_ = node["train_y_raw"].as<std::vector<double>>();
    if (node["train_obs_noise_var_raw"]) {
      train_obs_noise_var_raw_ = node["train_obs_noise_var_raw"].as<std::vector<double>>();
    } else {
      train_obs_noise_var_raw_.assign(train_x_raw_.size(), 0.0);
    }

    x_mean_ = node["x_mean"].as<double>();
    x_std_ = node["x_std"].as<double>();
    y_mean_ = node["y_mean"].as<double>();
    y_std_ = node["y_std"].as<double>();
    signal_var_ = node["signal_var"].as<double>();
    length_scale_ = node["length_scale"].as<double>();
    base_noise_var_ = node["base_noise_var"].as<double>();

    rebuildFromStoredData();
  }

  double lengthScale() const { return length_scale_; }
  double signalVar() const { return signal_var_; }
  double baseNoiseVar() const { return base_noise_var_; }
  int sampleSize() const { return static_cast<int>(train_x_raw_.size()); }

private:
  // 原始训练样本。
  std::vector<double> train_x_raw_;
  std::vector<double> train_y_raw_;
  std::vector<double> train_obs_noise_var_raw_;

  // 归一化后的训练样本。
  Eigen::VectorXd train_x_norm_;
  Eigen::VectorXd train_y_norm_;
  Eigen::VectorXd obs_noise_var_norm_;

  // Cholesky 分解和 alpha = K^{-1} y。
  Eigen::MatrixXd L_;
  Eigen::VectorXd alpha_;

  // 输入输出归一化参数。
  double x_mean_ = 0.0;
  double x_std_ = 1.0;
  double y_mean_ = 0.0;
  double y_std_ = 1.0;

  // Matern 3/2 核函数超参数。
  double signal_var_ = 1.0;
  double length_scale_ = 1.0;

  // 额外基础噪声方差，位于归一化输出空间。
  // 总对角噪声 = base_noise_var_ + obs_noise_var_norm_[i]。
  double base_noise_var_ = 1e-3;

  bool is_trained_ = false;

  void computeNormalization() {
    x_mean_ = mean(train_x_raw_);
    y_mean_ = mean(train_y_raw_);
    x_std_ = stddev(train_x_raw_, x_mean_);
    y_std_ = stddev(train_y_raw_, y_mean_);
    if (x_std_ < 1e-12) x_std_ = 1.0;
    if (y_std_ < 1e-12) y_std_ = 1.0;
  }

  double normalizeX(double x) const { return (x - x_mean_) / x_std_; }
  double normalizeY(double y) const { return (y - y_mean_) / y_std_; }

  void buildNormalizedVectors() {
    const int n = static_cast<int>(train_x_raw_.size());
    train_x_norm_.resize(n);
    train_y_norm_.resize(n);
    obs_noise_var_norm_.resize(n);

    for (int i = 0; i < n; ++i) {
      train_x_norm_(i) = normalizeX(train_x_raw_[i]);
      train_y_norm_(i) = normalizeY(train_y_raw_[i]);

      // 原始主频方差单位为 Hz^2，归一化后需要除以 y_std^2。
      double nv = train_obs_noise_var_raw_[i];
      if (!std::isfinite(nv) || nv < 0.0) nv = 0.0;
      obs_noise_var_norm_(i) = nv / (y_std_ * y_std_);
    }
  }

  void rebuildFromStoredData() {
    const int n = static_cast<int>(train_x_raw_.size());
    if (n < 3 || train_y_raw_.size() != train_x_raw_.size()) {
      throw std::runtime_error("Invalid stored GPR training data.");
    }
    if (train_obs_noise_var_raw_.size() != train_x_raw_.size()) {
      train_obs_noise_var_raw_.assign(train_x_raw_.size(), 0.0);
    }

    buildNormalizedVectors();
    Eigen::MatrixXd K = buildKernelMatrix(train_x_norm_, obs_noise_var_norm_,
                                          length_scale_, signal_var_, base_noise_var_);

    if (!factorizeAndSolve(K, train_y_norm_, L_, alpha_)) {
      throw std::runtime_error("GPR Cholesky factorization failed. Try larger noise.");
    }
    is_trained_ = true;
  }

  // Matern 3/2 核函数，不包含白噪声项。
  double kernelNoNoise(double a, double b) const {
    const double r = std::abs(a - b);
    const double z = std::sqrt(3.0) * r / length_scale_;
    return signal_var_ * (1.0 + z) * std::exp(-z);
  }

  Eigen::MatrixXd buildKernelMatrix(const Eigen::VectorXd& x_norm,
                                    const Eigen::VectorXd& obs_noise_var_norm,
                                    double length_scale,
                                    double signal_var,
                                    double base_noise_var) const {
    const int n = static_cast<int>(x_norm.size());
    Eigen::MatrixXd K(n, n);

    for (int i = 0; i < n; ++i) {
      for (int j = i; j < n; ++j) {
        const double r = std::abs(x_norm(i) - x_norm(j));
        const double z = std::sqrt(3.0) * r / length_scale;
        const double kij = signal_var * (1.0 + z) * std::exp(-z);
        K(i, j) = kij;
        K(j, i) = kij;
      }
    }

    // 加入观测噪声和数值 jitter，保证 K 正定或近似正定。
    for (int i = 0; i < n; ++i) {
      K(i, i) += base_noise_var + obs_noise_var_norm(i) + 1e-8;
    }
    return K;
  }

  bool factorizeAndSolve(const Eigen::MatrixXd& K,
                         const Eigen::VectorXd& y,
                         Eigen::MatrixXd& L_out,
                         Eigen::VectorXd& alpha_out) const {
    Eigen::LLT<Eigen::MatrixXd> llt(K);
    if (llt.info() != Eigen::Success) return false;
    L_out = llt.matrixL();
    alpha_out = llt.solve(y);
    return llt.info() == Eigen::Success;
  }

  // 对数边际似然，用于网格搜索选择超参数。
  double logMarginalLikelihood(double length_scale,
                               double signal_var,
                               double base_noise_var) const {
    Eigen::MatrixXd K = buildKernelMatrix(train_x_norm_, obs_noise_var_norm_,
                                          length_scale, signal_var, base_noise_var);
    Eigen::MatrixXd L;
    Eigen::VectorXd alpha;
    if (!factorizeAndSolve(K, train_y_norm_, L, alpha)) {
      return -std::numeric_limits<double>::infinity();
    }

    const int n = static_cast<int>(train_y_norm_.size());
    const double quad = train_y_norm_.dot(alpha);

    // log |K| = 2 sum log diag(L)，因此 0.5 log|K| = sum log diag(L)。
    double logdet_half = 0.0;
    for (int i = 0; i < n; ++i) {
      const double diag = L(i, i);
      if (diag <= 0.0) return -std::numeric_limits<double>::infinity();
      logdet_half += std::log(diag);
    }

    return -0.5 * quad - logdet_half - 0.5 * n * std::log(2.0 * M_PI);
  }

  // 简单网格搜索。优点是稳定、易懂；缺点是速度不如梯度优化。
  void gridSearchHyperparams() {
    const std::vector<double> length_grid = {0.05, 0.08, 0.12, 0.2, 0.35, 0.6, 1.0, 1.6, 2.5, 4.0, 7.0};
    const std::vector<double> signal_grid = {0.3, 0.6, 1.0, 1.8, 3.0};
    const std::vector<double> noise_grid  = {1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2, 8e-2};

    double best_lml = -std::numeric_limits<double>::infinity();
    double best_l = length_scale_;
    double best_signal = signal_var_;
    double best_noise = base_noise_var_;

    for (double l : length_grid) {
      for (double sv : signal_grid) {
        for (double nv : noise_grid) {
          const double lml = logMarginalLikelihood(l, sv, nv);
          if (lml > best_lml) {
            best_lml = lml;
            best_l = l;
            best_signal = sv;
            best_noise = nv;
          }
        }
      }
    }

    length_scale_ = best_l;
    signal_var_ = best_signal;
    base_noise_var_ = best_noise;
  }

  static double mean(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x;
    return s / static_cast<double>(v.size());
  }

  static double stddev(const std::vector<double>& v, double m) {
    double s = 0.0;
    for (double x : v) {
      const double d = x - m;
      s += d * d;
    }
    return std::sqrt(s / std::max<size_t>(1, v.size() - 1));
  }
};

// 决定系数 R^2，用于评价拟合解释能力。
inline double r2Score(const std::vector<double>& y, const std::vector<double>& yhat) {
  if (y.size() != yhat.size() || y.empty()) return std::numeric_limits<double>::quiet_NaN();
  double m = 0.0;
  for (double v : y) m += v;
  m /= static_cast<double>(y.size());

  double ss_res = 0.0;
  double ss_tot = 0.0;
  for (size_t i = 0; i < y.size(); ++i) {
    const double e = y[i] - yhat[i];
    const double d = y[i] - m;
    ss_res += e * e;
    ss_tot += d * d;
  }
  if (ss_tot < 1e-12) return 1.0;
  return 1.0 - ss_res / ss_tot;
}

// 均方根误差 RMSE，单位与 y 一致，本任务一般为 Hz。
inline double rmse(const std::vector<double>& y, const std::vector<double>& yhat) {
  if (y.size() != yhat.size() || y.empty()) return std::numeric_limits<double>::quiet_NaN();
  double s = 0.0;
  for (size_t i = 0; i < y.size(); ++i) {
    const double e = y[i] - yhat[i];
    s += e * e;
  }
  return std::sqrt(s / static_cast<double>(y.size()));
}

}  // namespace gpr_shared_frequency_model

