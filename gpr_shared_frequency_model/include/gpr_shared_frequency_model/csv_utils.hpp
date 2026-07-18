#pragma once

/*
 * 文件：csv_utils.hpp
 * 功能：简单 CSV 读取工具。
 *
 * 说明：
 *   1. ROS/C++ 直接读取 .xlsx 依赖较重，因此训练节点推荐读取 CSV。
 *   2. 你的 Excel 数据可以直接“另存为 CSV”，列结构保持不变。
 *   3. 本工具支持空单元格，空值会被解析为 NaN。
 */

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gpr_shared_frequency_model {

inline std::string trim(const std::string& s) {
  const char* ws = " \t\r\n\"";
  const size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos) return "";
  const size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

inline std::vector<std::string> splitCsvLine(const std::string& line) {
  // 简化版 CSV 解析器：支持普通逗号分隔；若字段中有逗号且带引号，建议用 Python 转换为简单 CSV。
  std::vector<std::string> out;
  std::string token;
  std::stringstream ss(line);
  while (std::getline(ss, token, ',')) {
    out.push_back(trim(token));
  }
  // 若行末是逗号，补一个空字段。
  if (!line.empty() && line.back() == ',') out.push_back("");
  return out;
}

inline double parseDoubleOrNan(const std::string& s) {
  const std::string t = trim(s);
  if (t.empty()) return std::numeric_limits<double>::quiet_NaN();
  try {
    size_t pos = 0;
    double v = std::stod(t, &pos);
    (void)pos;
    return v;
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

inline std::vector<std::vector<double>> readNumericCsv(const std::string& path, bool has_header) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Cannot open CSV file: " + path);
  }

  std::vector<std::vector<double>> data;
  std::string line;
  bool first = true;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    if (first && has_header) {
      first = false;
      continue;
    }
    first = false;

    const auto cells = splitCsvLine(line);
    std::vector<double> row;
    row.reserve(cells.size());
    for (const auto& c : cells) {
      row.push_back(parseDoubleOrNan(c));
    }
    data.push_back(row);
  }
  return data;
}

inline bool isFinite(double v) {
  return std::isfinite(v);
}

inline double getCellOrNan(const std::vector<double>& row, int col) {
  if (col < 0 || static_cast<size_t>(col) >= row.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return row[static_cast<size_t>(col)];
}

}  // namespace gpr_shared_frequency_model

