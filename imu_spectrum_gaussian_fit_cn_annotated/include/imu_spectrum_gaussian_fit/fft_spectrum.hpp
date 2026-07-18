/*
 * 文件：fft_spectrum.hpp
 * 功能：根据 IMU 时间序列计算单边 FFT 幅值谱。
 *
 * 对应论文步骤：
 *   For each IMU data segment, the single-sided frequency spectrum is obtained by FFT.
 *
 * 主要计算流程：
 *   1) 输入一段 IMU 时间序列 x[n]；
 *   2) 去均值：x[n] <- x[n] - mean(x)，用于抑制直流项；
 *   3) 加 Hann 窗：减少有限长度截断导致的频谱泄漏；
 *   4) 调用 FFTW 计算实信号 FFT；
 *   5) 将双边谱转成单边幅值谱；
 *   6) 对三轴信号合成：S(f)=sqrt(Sx^2+Sy^2+Sz^2)。
 *
 * 输出的 Spectrum1D 中：
 *   frequency_hz[k] = k * fs / N
 *   amplitude[k]    = 对应频率点处的单边幅值
 *
 * 这个输出会继续传递给 gaussian_peak_fitter.hpp，执行局部峰检测和高斯拟合。
 */
#pragma once

#include <fftw3.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace imu_spectrum_gaussian_fit {

/**
 * @brief 单边频谱结果。
 *
 * frequency_hz[i] : 第 i 个频率点，单位 Hz
 * amplitude[i]    : 对应幅值谱。这里输出的是单边幅值谱，可用于后续谱峰检测和高斯拟合。
 */
struct Spectrum1D {
  std::vector<double> frequency_hz;
  std::vector<double> amplitude;
};

/**
 * @brief FFT 频谱提取工具。
 *
 * 论文对应关系：
 * 对每一段 IMU 数据，先通过 FFT 得到单边频谱 S(f)，然后在主振动频带附近进行高斯拟合。
 *
 * 这里实现步骤：
 * 1. 去均值，降低直流分量影响；
 * 2. 加 Hann 窗，降低频谱泄漏；
 * 3. 使用 FFTW 计算实数信号 FFT；
 * 4. 转换成单边幅值谱；
 * 5. 对三轴陀螺仪/加速度计，可用 combineThreeAxisSpectrum() 合成为一个幅值谱。
 */
class FftSpectrum {
public:
  /**
   * @brief 对单轴 IMU 数据计算单边幅值谱。
   * @param signal 输入时间序列。
   * @param sample_rate_hz 采样率，例如 200 Hz。
   * @param remove_mean 是否去均值。
   * @param use_hann_window 是否使用 Hann 窗。
   */
  static Spectrum1D computeSingleAxis(const std::vector<double>& signal,
                                      double sample_rate_hz,
                                      bool remove_mean = true,
                                      bool use_hann_window = true) {
    if (signal.size() < 8) {
      throw std::runtime_error("FFT 至少需要 8 个采样点");
    }
    if (sample_rate_hz <= 0.0) {
      throw std::runtime_error("采样率必须大于 0");
    }

    const int n = static_cast<int>(signal.size());
    const int n_freq = n / 2 + 1;

    std::vector<double> x(signal.begin(), signal.end());

    if (remove_mean) {
      double m = 0.0;
      for (double v : x) m += v;
      m /= static_cast<double>(x.size());
      for (double& v : x) v -= m;
    }

    // Hann 窗：w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
    // 同时计算 coherent gain，用于修正窗函数导致的幅值缩放。
    double window_sum = 0.0;
    if (use_hann_window) {
      for (int i = 0; i < n; ++i) {
        const double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / static_cast<double>(n - 1)));
        x[i] *= w;
        window_sum += w;
      }
    } else {
      window_sum = static_cast<double>(n);
    }
    const double coherent_gain = window_sum / static_cast<double>(n);

    fftw_complex* out = reinterpret_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * n_freq));
    double* in = reinterpret_cast<double*>(fftw_malloc(sizeof(double) * n));
    if (!out || !in) {
      if (out) fftw_free(out);
      if (in) fftw_free(in);
      throw std::runtime_error("FFTW 内存分配失败");
    }

    for (int i = 0; i < n; ++i) in[i] = x[i];

    fftw_plan plan = fftw_plan_dft_r2c_1d(n, in, out, FFTW_ESTIMATE);
    fftw_execute(plan);

    Spectrum1D spec;
    spec.frequency_hz.resize(n_freq);
    spec.amplitude.resize(n_freq);

    for (int k = 0; k < n_freq; ++k) {
      const double re = out[k][0];
      const double im = out[k][1];
      double amp = std::sqrt(re * re + im * im) / static_cast<double>(n);

      // 单边谱除 DC 和 Nyquist 外乘以 2。
      if (k != 0 && k != n / 2) amp *= 2.0;

      // 修正窗函数的 coherent gain。
      if (coherent_gain > 1e-12) amp /= coherent_gain;

      spec.frequency_hz[k] = sample_rate_hz * static_cast<double>(k) / static_cast<double>(n);
      spec.amplitude[k] = amp;
    }

    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
    return spec;
  }

  /**
   * @brief 三轴频谱合成。
   *
   * 对陀螺仪三轴或加速度计三轴，先分别做 FFT，再做幅值合成：
   *
   *   S(f) = sqrt(S_x(f)^2 + S_y(f)^2 + S_z(f)^2)
   *
   * 这样得到一个代表三轴整体振动能量的单边频谱，后续对这个频谱做谱峰检测和高斯拟合。
   */
  static Spectrum1D combineThreeAxisSpectrum(const std::vector<double>& x,
                                             const std::vector<double>& y,
                                             const std::vector<double>& z,
                                             double sample_rate_hz,
                                             bool remove_mean = true,
                                             bool use_hann_window = true) {
    if (x.size() != y.size() || x.size() != z.size()) {
      throw std::runtime_error("三轴 IMU 数据长度必须一致");
    }

    Spectrum1D sx = computeSingleAxis(x, sample_rate_hz, remove_mean, use_hann_window);
    Spectrum1D sy = computeSingleAxis(y, sample_rate_hz, remove_mean, use_hann_window);
    Spectrum1D sz = computeSingleAxis(z, sample_rate_hz, remove_mean, use_hann_window);

    Spectrum1D out;
    out.frequency_hz = sx.frequency_hz;
    out.amplitude.resize(sx.amplitude.size());

    for (size_t i = 0; i < out.amplitude.size(); ++i) {
      out.amplitude[i] = std::sqrt(sx.amplitude[i] * sx.amplitude[i] +
                                   sy.amplitude[i] * sy.amplitude[i] +
                                   sz.amplitude[i] * sz.amplitude[i]);
    }
    return out;
  }
};

}  // namespace imu_spectrum_gaussian_fit
