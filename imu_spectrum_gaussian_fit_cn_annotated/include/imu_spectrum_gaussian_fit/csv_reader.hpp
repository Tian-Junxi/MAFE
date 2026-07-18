/*
 * 文件：csv_reader.hpp
 * 功能：读取离线 IMU CSV 数据，并按照用户配置的列索引提取数值列。
 *
 * 典型输入 CSV 格式：
 *   gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
 *   ...
 *
 * 与本文流程的关系：
 *   该文件只负责“数据进入算法”的第一步，不做频谱分析。
 *   读取后的 6 列时间序列会交给 fft_spectrum.hpp 计算 FFT 单边频谱。
 *
 * 注意事项：
 *   1. 列索引采用 C/C++ 习惯的 0-based 编号：第一列为 0，第二列为 1。
 *   2. 如果 CSV 中有时间戳列，例如 time, gx, gy, gz, ax, ay, az，
 *      则应该在 yaml 中把 gyro_cols 设置为 [1,2,3]，acc_cols 设置为 [4,5,6]。
 *   3. 这里不依赖 ROS message，仅作为通用 C++ 工具，方便离线训练和批处理。
 */
#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace imu_spectrum_gaussian_fit {

/**
 * @brief 简单 CSV 读取工具。
 *
 * 本模块用于读取离线 IMU 数据文件。默认假设 CSV 每一行至少包含 6 列：
 *   第 1~3 列：陀螺仪三轴测量 gyro_x, gyro_y, gyro_z
 *   第 4~6 列：加速度计三轴测量 acc_x, acc_y, acc_z
 *
 * 说明：
 * - 这里没有强制列名，因为后续实验中你可能直接从 ROS bag / Excel 导出 CSV；
 * - 可以通过参数设置列索引，因此即使 CSV 有时间戳列，也可以指定真正的数据列；
 * - 列索引采用 0-based，即第一列为 0。
 */
class CsvReader {
public:
  static std::vector<std::vector<double>> readNumericCsv(const std::string& path,
                                                         bool has_header = true) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      throw std::runtime_error("无法打开 CSV 文件: " + path);
    }

    std::vector<std::vector<double>> data;
    std::string line;
    bool first_line = true;

    while (std::getline(ifs, line)) {
      trim(line);
      if (line.empty()) continue;

      if (first_line && has_header) {
        first_line = false;
        continue;
      }
      first_line = false;

      std::vector<std::string> tokens = split(line, ',');
      std::vector<double> row;
      row.reserve(tokens.size());

      bool row_valid = true;
      for (std::string token : tokens) {
        trim(token);
        if (token.empty()) {
          row.push_back(std::numeric_limits<double>::quiet_NaN());
          continue;
        }
        try {
          row.push_back(std::stod(token));
        } catch (...) {
          row_valid = false;
          break;
        }
      }
      if (row_valid && !row.empty()) data.push_back(row);
    }
    return data;
  }

  /**
   * @brief 从数值表中提取某一列。
   */
  static std::vector<double> getColumn(const std::vector<std::vector<double>>& table,
                                       int col_index) {
    if (col_index < 0) {
      throw std::runtime_error("列索引不能小于 0");
    }
    std::vector<double> out;
    out.reserve(table.size());
    for (size_t r = 0; r < table.size(); ++r) {
      if (static_cast<int>(table[r].size()) <= col_index) {
        throw std::runtime_error("CSV 第 " + std::to_string(r) + " 行列数不足");
      }
      out.push_back(table[r][col_index]);
    }
    return out;
  }

private:
  static std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) elems.push_back(item);
    return elems;
  }

  static void trim(std::string& s) {
    auto not_space = [](int ch) { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  }
};

}  // namespace imu_spectrum_gaussian_fit
