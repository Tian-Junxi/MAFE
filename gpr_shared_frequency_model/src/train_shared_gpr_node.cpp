/*
 * 节点：train_shared_gpr_node
 *
 * 功能：
 *   从 CSV 实验数据中训练“共享单电机 GPR 主频模型”。
 *
 * 数据结构：
 *   每一行包含 4 个电机转速 m1,m2,m3,m4，以及该行频谱中提取到的 1~4 个主频峰。
 *   当四个转速差异较大时，频谱可能出现 4 个主频；当某些转速接近时，多个电机的谱峰会合并，
 *   所以主频数量可能少于 4。
 *
 * 核心思想：
 *   1. 用初始平均转速先验得到一个初始匹配；
 *   2. 将四个电机样本全部展开为 (单个电机转速, 匹配主频)；
 *   3. 训练一个共享 GPR： f_d = g(m)；
 *   4. 用共享 GPR 重新匹配每行的“电机转速 -> 观测主频”；
 *   5. 反复执行“重匹配 -> 重训练”，直到匹配不再变化。
 *
 * 最终输出：
 *   model_dir/gyro_gpr_model.gpr
 *   model_dir/acc_gpr_model.gpr
 *
 * 这两个 .gpr 文件内部是 YAML 格式，后续 predict 节点加载它们即可对四个转速输出四个主频均值和方差。
 */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

#include "gpr_shared_frequency_model/csv_utils.hpp"
#include "gpr_shared_frequency_model/gpr1d_matern.hpp"

namespace gpr_shared_frequency_model {


// ============================== 时间统计工具 ==============================
// 说明：ROS 节点里常用 ros::Time 记录系统时间，但训练/预测耗时统计更适合使用
// std::chrono::steady_clock。steady_clock 是单调时钟，不受系统时间校准影响，
// 因此用于统计 GPR 训练、匹配、保存等代码段的 wall-clock 时间更稳定。
using SteadyClock = std::chrono::steady_clock;

static double elapsedMs(const SteadyClock::time_point& start,
                        const SteadyClock::time_point& end = SteadyClock::now()) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 默认输出目录：~/catkin_ws/output。
// 如果 launch 文件没有传入 model_dir，则使用该默认路径。
// 这里不依赖 rospack，直接通过 HOME 环境变量拼接，方便在任意 ROS 终端运行。
static std::string defaultOutputDir() {
  const char* home = std::getenv("HOME");
  if (home && std::string(home).size() > 0) {
    return std::string(home) + "/catkin_ws/output";
  }
  return "output";
}

struct PeakObs {
  double freq = std::numeric_limits<double>::quiet_NaN();  // 观测主频，Hz
  double var = 0.0;                                        // 主频观测方差，Hz^2
  int freq_col = -1;                                       // 原 CSV 中对应主频列号，便于检查
  int var_col = -1;                                        // 原 CSV 中对应方差列号，便于检查
};

struct DataRow {
  std::array<double, 4> motor_speed{};
  double avg_speed = 0.0;
  std::vector<PeakObs> gyro_peaks;
  std::vector<PeakObs> acc_peaks;
};

struct SensorConfig {
  std::string name;                    // 例如 "gyro" 或 "acc"
  std::vector<int> frequency_cols;      // 主频列号，0-based
  std::vector<int> variance_cols;       // 方差列号，0-based，与主频列一一对应
};

struct TrainConfig {
  std::vector<int> motor_speed_cols;    // 4 个电机转速列号，0-based
  SensorConfig gyro;
  SensorConfig acc;
  bool csv_has_header = true;
  int max_iterations = 8;
  bool optimize_hyperparams = true;
  std::string variance_mode = "variance"; // "variance": 表格列是方差；"std": 表格列是标准差/峰宽
  double min_observation_variance = 1e-6;
};

struct MatchedSample {
  int row_index = -1;
  int motor_id = -1;       // 1~4
  double speed = 0.0;
  double freq = 0.0;
  double obs_var = 0.0;
  int matched_peak_index = -1;
  int matched_freq_col = -1;
};

struct AssignmentResult {
  std::vector<std::array<int, 4>> assignments; // 每行 4 个电机分别匹配到第几个观测峰
  std::vector<MatchedSample> samples;          // 展开后的训练样本
  double total_cost = 0.0;
};

struct IterStat {
  int iteration = 0;
  int n_samples = 0;
  int changed_rows = 0;
  int changed_slots = 0;
  double r2 = 0.0;
  double rmse_hz = 0.0;
};

static std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (a.back() == '/') return a + b;
  return a + "/" + b;
}


// 递归创建输出目录。
// 说明：训练节点会把 .gpr 模型和 CSV 统计文件保存到 model_dir。
// 若该目录不存在，直接写文件会失败，因此这里在训练开始前自动创建目录。
static void makeDirRecursive(const std::string& dir) {
  if (dir.empty()) return;
  std::string path;
  for (size_t i = 0; i < dir.size(); ++i) {
    path.push_back(dir[i]);
    if (dir[i] == '/' || i + 1 == dir.size()) {
      if (path.empty() || path == "/") continue;
      if (::mkdir(path.c_str(), 0755) != 0) {
        if (errno == EEXIST) continue;
        throw std::runtime_error("Cannot create directory: " + path + ", error=" + std::strerror(errno));
      }
    }
  }
}

static std::vector<int> readIntVector(const YAML::Node& node, const std::string& key) {
  if (!node[key]) throw std::runtime_error("Missing config key: " + key);
  return node[key].as<std::vector<int>>();
}

static TrainConfig loadTrainConfig(const std::string& path) {
  YAML::Node y = YAML::LoadFile(path);
  TrainConfig cfg;
  cfg.motor_speed_cols = readIntVector(y, "motor_speed_cols");
  if (cfg.motor_speed_cols.size() != 4) {
    throw std::runtime_error("motor_speed_cols must contain exactly 4 columns.");
  }

  cfg.csv_has_header = y["csv_has_header"] ? y["csv_has_header"].as<bool>() : true;
  cfg.max_iterations = y["max_iterations"] ? y["max_iterations"].as<int>() : 8;
  cfg.optimize_hyperparams = y["optimize_hyperparams"] ? y["optimize_hyperparams"].as<bool>() : true;
  cfg.variance_mode = y["variance_mode"] ? y["variance_mode"].as<std::string>() : "variance";
  cfg.min_observation_variance = y["min_observation_variance"] ? y["min_observation_variance"].as<double>() : 1e-6;

  cfg.gyro.name = "gyro";
  cfg.gyro.frequency_cols = y["gyro"]["frequency_cols"].as<std::vector<int>>();
  cfg.gyro.variance_cols = y["gyro"]["variance_cols"].as<std::vector<int>>();

  cfg.acc.name = "acc";
  cfg.acc.frequency_cols = y["acc"]["frequency_cols"].as<std::vector<int>>();
  cfg.acc.variance_cols = y["acc"]["variance_cols"].as<std::vector<int>>();

  if (cfg.gyro.frequency_cols.size() != cfg.gyro.variance_cols.size()) {
    throw std::runtime_error("gyro frequency_cols and variance_cols size mismatch.");
  }
  if (cfg.acc.frequency_cols.size() != cfg.acc.variance_cols.size()) {
    throw std::runtime_error("acc frequency_cols and variance_cols size mismatch.");
  }
  return cfg;
}

static double convertVariance(double raw, const TrainConfig& cfg) {
  // 若原始方差列为空，则给一个很小的默认方差。
  if (!std::isfinite(raw) || raw < 0.0) return cfg.min_observation_variance;

  if (cfg.variance_mode == "std") {
    // 如果表格列存的是标准差或高斯峰宽 sigma，则方差 = sigma^2。
    return std::max(raw * raw, cfg.min_observation_variance);
  }

  // 默认认为表格列就是主频方差，单位 Hz^2。
  return std::max(raw, cfg.min_observation_variance);
}

static std::vector<PeakObs> extractPeaks(const std::vector<double>& row,
                                         const SensorConfig& scfg,
                                         const TrainConfig& cfg) {
  std::vector<PeakObs> peaks;
  for (size_t i = 0; i < scfg.frequency_cols.size(); ++i) {
    const int fcol = scfg.frequency_cols[i];
    const int vcol = scfg.variance_cols[i];
    const double f = getCellOrNan(row, fcol);
    if (!std::isfinite(f)) continue;

    PeakObs p;
    p.freq = f;
    p.var = convertVariance(getCellOrNan(row, vcol), cfg);
    p.freq_col = fcol;
    p.var_col = vcol;
    peaks.push_back(p);
  }
  return peaks;
}

static std::vector<DataRow> loadRowsFromCsv(const std::string& csv_path, const TrainConfig& cfg) {
  const auto table = readNumericCsv(csv_path, cfg.csv_has_header);
  std::vector<DataRow> rows;

  for (const auto& r : table) {
    DataRow row;
    bool valid_motor = true;
    double sum = 0.0;
    for (int i = 0; i < 4; ++i) {
      const double m = getCellOrNan(r, cfg.motor_speed_cols[i]);
      if (!std::isfinite(m)) {
        valid_motor = false;
        break;
      }
      row.motor_speed[i] = m;
      sum += m;
    }
    if (!valid_motor) continue;
    row.avg_speed = sum / 4.0;

    row.gyro_peaks = extractPeaks(r, cfg.gyro, cfg);
    row.acc_peaks = extractPeaks(r, cfg.acc, cfg);

    // 至少有一种传感器有有效主频，才保留该行。
    if (!row.gyro_peaks.empty() || !row.acc_peaks.empty()) {
      rows.push_back(row);
    }
  }
  return rows;
}

// 递归枚举 q^4 种匹配方式，并要求每个观测峰至少被一个电机使用。
static void enumerateAssignments(int q,
                                 int idx,
                                 std::array<int,4>& cur,
                                 std::vector<std::array<int,4>>& out) {
  if (idx == 4) {
    std::vector<bool> used(q, false);
    for (int i = 0; i < 4; ++i) used[cur[i]] = true;
    for (int j = 0; j < q; ++j) {
      if (!used[j]) return;
    }
    out.push_back(cur);
    return;
  }
  for (int j = 0; j < q; ++j) {
    cur[idx] = j;
    enumerateAssignments(q, idx + 1, cur, out);
  }
}

static std::vector<std::array<int,4>> allAssignments(int q) {
  std::vector<std::array<int,4>> out;
  std::array<int,4> cur{{0,0,0,0}};
  enumerateAssignments(q, 0, cur, out);
  return out;
}

using PredictFunc = std::function<Prediction1D(double)>;

static AssignmentResult matchRowsWithPredictor(const std::vector<DataRow>& rows,
                                               const std::string& sensor_name,
                                               const PredictFunc& predictor) {
  AssignmentResult result;
  result.assignments.resize(rows.size(), std::array<int,4>{{-1,-1,-1,-1}});

  for (size_t ri = 0; ri < rows.size(); ++ri) {
    const auto& row = rows[ri];
    const std::vector<PeakObs>& peaks = (sensor_name == "gyro") ? row.gyro_peaks : row.acc_peaks;
    const int q = static_cast<int>(peaks.size());
    if (q == 0) continue;

    const auto candidates = allAssignments(q);
    double best_cost = std::numeric_limits<double>::infinity();
    std::array<int,4> best_assign{{0,0,0,0}};

    for (const auto& a : candidates) {
      double cost = 0.0;
      for (int motor = 0; motor < 4; ++motor) {
        const Prediction1D pred = predictor(row.motor_speed[motor]);
        const int peak_idx = a[motor];
        const double residual = pred.mean - peaks[peak_idx].freq;
        const double denom = std::max(pred.variance + peaks[peak_idx].var, 1e-8);
        cost += residual * residual / denom;
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_assign = a;
      }
    }

    result.assignments[ri] = best_assign;
    result.total_cost += best_cost;

    for (int motor = 0; motor < 4; ++motor) {
      const int peak_idx = best_assign[motor];
      const PeakObs& p = peaks[peak_idx];
      MatchedSample s;
      s.row_index = static_cast<int>(ri);
      s.motor_id = motor + 1;
      s.speed = row.motor_speed[motor];
      s.freq = p.freq;
      s.obs_var = p.var;
      s.matched_peak_index = peak_idx;
      s.matched_freq_col = p.freq_col;
      result.samples.push_back(s);
    }
  }
  return result;
}

static GPR1DMatern trainSharedModelFromSamples(const std::vector<MatchedSample>& samples,
                                               bool optimize_hyperparams) {
  std::vector<double> x, y, var;
  x.reserve(samples.size());
  y.reserve(samples.size());
  var.reserve(samples.size());

  for (const auto& s : samples) {
    x.push_back(s.speed);
    y.push_back(s.freq);
    var.push_back(s.obs_var);
  }

  GPR1DMatern model;
  model.train(x, y, var, optimize_hyperparams);
  return model;
}

static std::pair<double,double> evaluateModelOnSamples(const GPR1DMatern& model,
                                                       const std::vector<MatchedSample>& samples) {
  std::vector<double> y, yhat;
  y.reserve(samples.size());
  yhat.reserve(samples.size());
  for (const auto& s : samples) {
    y.push_back(s.freq);
    yhat.push_back(model.predict(s.speed).mean);
  }
  return {r2Score(y, yhat), rmse(y, yhat)};
}

/*
 * 初始匹配：
 *   为了避免一开始完全没有模型，这里使用“平均转速 -> 第 j 个主频列”的临时 GPR 先验。
 *   注意：这些临时模型只用于初始化，不会作为最终模型保存。
 *   最终保存的仍然是“四个电机共用一个共享 GPR 模型”。
 */
static AssignmentResult initialMatchByAverageSpeedPriors(const std::vector<DataRow>& rows,
                                                         const std::string& sensor_name,
                                                         const TrainConfig& cfg) {
  const SensorConfig& scfg = (sensor_name == "gyro") ? cfg.gyro : cfg.acc;

  std::vector<GPR1DMatern> prior_models(scfg.frequency_cols.size());
  std::vector<bool> prior_valid(scfg.frequency_cols.size(), false);

  for (size_t peak_col_idx = 0; peak_col_idx < scfg.frequency_cols.size(); ++peak_col_idx) {
    std::vector<double> x, y, var;
    for (const auto& row : rows) {
      const auto& peaks = (sensor_name == "gyro") ? row.gyro_peaks : row.acc_peaks;
      for (const auto& p : peaks) {
        if (p.freq_col == scfg.frequency_cols[peak_col_idx]) {
          x.push_back(row.avg_speed);
          y.push_back(p.freq);
          var.push_back(p.var);
          break;
        }
      }
    }
    if (x.size() >= 3) {
      prior_models[peak_col_idx].train(x, y, var, cfg.optimize_hyperparams);
      prior_valid[peak_col_idx] = true;
    }
  }

  // 初始预测器：对于候选峰 j，使用对应主频列的平均转速先验；
  // 但 matchRowsWithPredictor 接口只接收一个 predictor，因此这里不用它，单独写匹配过程。
  AssignmentResult result;
  result.assignments.resize(rows.size(), std::array<int,4>{{-1,-1,-1,-1}});

  for (size_t ri = 0; ri < rows.size(); ++ri) {
    const auto& row = rows[ri];
    const std::vector<PeakObs>& peaks = (sensor_name == "gyro") ? row.gyro_peaks : row.acc_peaks;
    const int q = static_cast<int>(peaks.size());
    if (q == 0) continue;

    const auto candidates = allAssignments(q);
    double best_cost = std::numeric_limits<double>::infinity();
    std::array<int,4> best_assign{{0,0,0,0}};

    for (const auto& a : candidates) {
      double cost = 0.0;
      for (int motor = 0; motor < 4; ++motor) {
        const int peak_idx = a[motor];
        const PeakObs& p = peaks[peak_idx];

        // 找这个观测峰来自原始第几个主频列。
        int col_idx = -1;
        for (size_t j = 0; j < scfg.frequency_cols.size(); ++j) {
          if (p.freq_col == scfg.frequency_cols[j]) {
            col_idx = static_cast<int>(j);
            break;
          }
        }

        double pred_mean = p.freq;       // 如果没有先验，退化为零残差，但由约束保证每个峰被使用。
        double pred_var = 100.0;         // 较大的默认方差，表示先验不确定。
        if (col_idx >= 0 && prior_valid[col_idx]) {
          const Prediction1D pred = prior_models[col_idx].predict(row.motor_speed[motor]);
          pred_mean = pred.mean;
          pred_var = pred.variance;
        }

        const double residual = pred_mean - p.freq;
        const double denom = std::max(pred_var + p.var, 1e-8);
        cost += residual * residual / denom;
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_assign = a;
      }
    }

    result.assignments[ri] = best_assign;
    result.total_cost += best_cost;
    for (int motor = 0; motor < 4; ++motor) {
      const int peak_idx = best_assign[motor];
      const PeakObs& p = peaks[peak_idx];
      MatchedSample s;
      s.row_index = static_cast<int>(ri);
      s.motor_id = motor + 1;
      s.speed = row.motor_speed[motor];
      s.freq = p.freq;
      s.obs_var = p.var;
      s.matched_peak_index = peak_idx;
      s.matched_freq_col = p.freq_col;
      result.samples.push_back(s);
    }
  }

  return result;
}

static int countChangedRows(const std::vector<std::array<int,4>>& a,
                            const std::vector<std::array<int,4>>& b) {
  int n = 0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    if (a[i] != b[i]) ++n;
  }
  return n;
}

static int countChangedSlots(const std::vector<std::array<int,4>>& a,
                             const std::vector<std::array<int,4>>& b) {
  int n = 0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      if (a[i][j] != b[i][j]) ++n;
    }
  }
  return n;
}

static void writeIterStats(const std::string& path, const std::vector<IterStat>& stats) {
  std::ofstream ofs(path);
  ofs << "iteration,n_samples,changed_rows,changed_slots,R2,RMSE_Hz\n";
  ofs << std::setprecision(10);
  for (const auto& s : stats) {
    ofs << s.iteration << "," << s.n_samples << "," << s.changed_rows << "," << s.changed_slots
        << "," << s.r2 << "," << s.rmse_hz << "\n";
  }
}

static void writeMatchedSamples(const std::string& path, const std::vector<MatchedSample>& samples) {
  std::ofstream ofs(path);
  ofs << "row_index,motor_id,speed,matched_frequency,observation_variance,matched_peak_index,matched_frequency_column\n";
  ofs << std::setprecision(10);
  for (const auto& s : samples) {
    ofs << s.row_index << "," << s.motor_id << "," << s.speed << "," << s.freq << ","
        << s.obs_var << "," << s.matched_peak_index << "," << s.matched_freq_col << "\n";
  }
}

static GPR1DMatern trainOneSensor(const std::vector<DataRow>& rows,
                                  const std::string& sensor_name,
                                  const TrainConfig& cfg,
                                  const std::string& model_dir) {
  ROS_INFO_STREAM("Start training shared GPR for sensor: " << sensor_name);

  const auto sensor_time_begin = SteadyClock::now();
  std::vector<IterStat> iter_stats;

  // 第 0 轮：用平均转速临时先验做初始匹配。
  // 这里统计的是“初始隐变量匹配”的耗时，不等同于最终 GPR 训练耗时。
  const auto init_match_begin = SteadyClock::now();
  AssignmentResult current = initialMatchByAverageSpeedPriors(rows, sensor_name, cfg);
  const double init_match_ms = elapsedMs(init_match_begin);

  // 第 0 轮 GPR 拟合训练耗时。
  // trainSharedModelFromSamples() 内部会调用 GPR1DMatern::train()，包括：
  //   1) 样本归一化；
  //   2) Matérn 核矩阵构造；
  //   3) 可选超参数网格搜索；
  //   4) Cholesky 分解和 alpha = K^{-1}y 求解。
  const auto train0_begin = SteadyClock::now();
  GPR1DMatern model = trainSharedModelFromSamples(current.samples, cfg.optimize_hyperparams);
  const double train0_ms = elapsedMs(train0_begin);

  const auto eval0_begin = SteadyClock::now();
  auto eval0 = evaluateModelOnSamples(model, current.samples);
  const double eval0_ms = elapsedMs(eval0_begin);

  iter_stats.push_back({0, static_cast<int>(current.samples.size()), 0, 0, eval0.first, eval0.second});

  ROS_INFO_STREAM(sensor_name << " iter 0: samples=" << current.samples.size()
                  << ", R2=" << eval0.first << ", RMSE=" << eval0.second
                  << ", init_match_time=" << init_match_ms << " ms"
                  << ", gpr_train_time=" << train0_ms << " ms"
                  << ", eval_time=" << eval0_ms << " ms");

  double total_match_ms = init_match_ms;
  double total_gpr_train_ms = train0_ms;
  double total_eval_ms = eval0_ms;

  // 之后迭代：用共享 GPR 重新匹配，然后重新训练共享 GPR。
  for (int it = 1; it <= cfg.max_iterations; ++it) {
    const auto iter_begin = SteadyClock::now();

    const auto match_begin = SteadyClock::now();
    AssignmentResult next = matchRowsWithPredictor(
        rows, sensor_name,
        [&model](double speed) { return model.predict(speed); });
    const double match_ms = elapsedMs(match_begin);

    const int changed_rows = countChangedRows(current.assignments, next.assignments);
    const int changed_slots = countChangedSlots(current.assignments, next.assignments);

    const auto train_begin = SteadyClock::now();
    model = trainSharedModelFromSamples(next.samples, cfg.optimize_hyperparams);
    const double gpr_train_ms = elapsedMs(train_begin);

    const auto eval_begin = SteadyClock::now();
    auto eval = evaluateModelOnSamples(model, next.samples);
    const double eval_ms = elapsedMs(eval_begin);
    const double iter_ms = elapsedMs(iter_begin);

    total_match_ms += match_ms;
    total_gpr_train_ms += gpr_train_ms;
    total_eval_ms += eval_ms;

    iter_stats.push_back({it, static_cast<int>(next.samples.size()), changed_rows, changed_slots,
                          eval.first, eval.second});

    ROS_INFO_STREAM(sensor_name << " iter " << it
                    << ": changed_rows=" << changed_rows
                    << ", changed_slots=" << changed_slots
                    << ", R2=" << eval.first
                    << ", RMSE=" << eval.second
                    << ", match_time=" << match_ms << " ms"
                    << ", gpr_train_time=" << gpr_train_ms << " ms"
                    << ", eval_time=" << eval_ms << " ms"
                    << ", iter_total_time=" << iter_ms << " ms");

    current = next;
    if (changed_rows == 0) {
      ROS_INFO_STREAM(sensor_name << " converged at iteration " << it);
      break;
    }
  }

  const std::string model_path = joinPath(model_dir, sensor_name + "_gpr_model.gpr");
  const std::string stats_path = joinPath(model_dir, sensor_name + "_iter_stats.csv");
  const std::string samples_path = joinPath(model_dir, sensor_name + "_final_matched_samples.csv");

  const auto save_begin = SteadyClock::now();
  model.save(model_path);
  writeIterStats(stats_path, iter_stats);
  writeMatchedSamples(samples_path, current.samples);
  const double save_ms = elapsedMs(save_begin);
  const double sensor_total_ms = elapsedMs(sensor_time_begin);

  ROS_INFO_STREAM("Saved " << sensor_name << " model to: " << model_path);
  ROS_INFO_STREAM("Saved " << sensor_name << " iteration stats to: " << stats_path);
  ROS_INFO_STREAM("Saved " << sensor_name << " matched samples to: " << samples_path);

  ROS_INFO_STREAM("[Timing] " << sensor_name
                  << " total_time=" << sensor_total_ms << " ms"
                  << ", total_matching_time=" << total_match_ms << " ms"
                  << ", total_gpr_training_time=" << total_gpr_train_ms << " ms"
                  << ", total_eval_time=" << total_eval_ms << " ms"
                  << ", save_time=" << save_ms << " ms");

  return model;
}

}  // namespace gpr_shared_frequency_model

int main(int argc, char** argv) {
  ros::init(argc, argv, "train_shared_gpr_node");
  ros::NodeHandle nh("~");

  std::string csv_path;
  std::string model_dir;
  std::string param_file;

  nh.param<std::string>("csv_path", csv_path, "");
  nh.param<std::string>("model_dir", model_dir, defaultOutputDir());
  nh.param<std::string>("param_file", param_file, "");

  if (csv_path.empty() || param_file.empty()) {
    ROS_ERROR("Please set parameters: csv_path and param_file.");
    return 1;
  }

  try {
    using namespace gpr_shared_frequency_model;

    const auto total_train_begin = SteadyClock::now();

    TrainConfig cfg = loadTrainConfig(param_file);
    std::vector<DataRow> rows = loadRowsFromCsv(csv_path, cfg);
    if (rows.empty()) {
      ROS_ERROR("No valid rows loaded from CSV.");
      return 1;
    }

    // 若用户传入相对路径，例如 gpr_shared_frequency_model_result，
    // 则模型会保存到 roslaunch 当前工作目录下的同名文件夹。
    makeDirRecursive(model_dir);

    ROS_INFO_STREAM("Loaded valid rows: " << rows.size());
    ROS_INFO_STREAM("Model output directory: " << model_dir);

    trainOneSensor(rows, "gyro", cfg, model_dir);
    trainOneSensor(rows, "acc", cfg, model_dir);

    ROS_INFO_STREAM("Training finished. total_elapsed_time=" << elapsedMs(total_train_begin) << " ms");
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Training failed: " << e.what());
    return 1;
  }

  return 0;
}

