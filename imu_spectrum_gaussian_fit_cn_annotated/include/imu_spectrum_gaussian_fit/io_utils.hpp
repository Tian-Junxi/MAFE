/*
 * 文件：io_utils.hpp
 * 功能：把高斯拟合结果保存为 CSV，便于检查和与后续 GPR 模块对接。
 *
 * 输出文件说明：
 *   1) *_peak_params.csv
 *      每一行对应一个高斯谱峰，包含 A、mu(fd)、sigma、sigma^2、fL、fR、RMSE 等。
 *
 *   2) *_spectrum_fit.csv
 *      每一行对应一个频率点，包含原始 FFT 幅值、每个高斯峰曲线和高斯叠加曲线。
 *      该文件适合用 Python/Matlab/Origin 重新画图或进一步分析。
 */
#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "imu_spectrum_gaussian_fit/gaussian_peak_fitter.hpp"

namespace imu_spectrum_gaussian_fit {

/**
 * @brief 输出文件工具。
 *
 * 该工具负责把高斯拟合结果保存成 CSV，便于后续和 GPR 训练代码对接：
 * - peak_params.csv：每个谱峰的 A、主频、方差、左右边界；
 * - spectrum_fit.csv：频率、原始频谱、各个高斯拟合曲线、叠加曲线。
 */
class IoUtils {
public:
  static void savePeakParamsCsv(const std::string& path,
                                const std::vector<GaussianPeakParam>& peaks) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      throw std::runtime_error("无法写入峰参数 CSV: " + path);
    }
    ofs << "peak_id,valid,A,fd_mu_hz,sigma_hz,variance_hz2,f_left_hz,f_right_hz,rmse,left_index,right_index,peak_index\n";
    for (const auto& p : peaks) {
      ofs << p.peak_id << ","
          << (p.valid ? 1 : 0) << ","
          << p.amplitude_A << ","
          << p.mu_hz << ","
          << p.sigma_hz << ","
          << p.variance_hz2 << ","
          << p.f_left_hz << ","
          << p.f_right_hz << ","
          << p.rmse << ","
          << p.left_index << ","
          << p.right_index << ","
          << p.peak_index << "\n";
    }
  }

  static void saveSpectrumFitCsv(const std::string& path,
                                 const Spectrum1D& spectrum,
                                 const std::vector<GaussianPeakParam>& peaks) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      throw std::runtime_error("无法写入频谱拟合 CSV: " + path);
    }

    std::vector<std::vector<double>> gaussians;
    gaussians.reserve(peaks.size());
    for (const auto& p : peaks) {
      gaussians.push_back(GaussianPeakFitter::evaluateGaussianCurve(spectrum.frequency_hz, p));
    }
    std::vector<double> sum_fit = GaussianPeakFitter::evaluateGaussianSum(spectrum.frequency_hz, peaks);

    ofs << "frequency_hz,spectrum_amplitude";
    for (size_t i = 0; i < peaks.size(); ++i) {
      ofs << ",gaussian_peak_" << (i + 1);
    }
    ofs << ",gaussian_sum\n";

    for (size_t i = 0; i < spectrum.frequency_hz.size(); ++i) {
      ofs << spectrum.frequency_hz[i] << "," << spectrum.amplitude[i];
      for (size_t p = 0; p < gaussians.size(); ++p) {
        ofs << "," << gaussians[p][i];
      }
      ofs << "," << sum_fit[i] << "\n";
    }
  }
};

}  // namespace imu_spectrum_gaussian_fit
