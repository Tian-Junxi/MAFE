/*
 * 文件：svg_plotter.hpp
 * 功能：不依赖 Python/matplotlib，直接用 C++ 生成 SVG 频谱图。
 *
 * 图中元素：
 *   - 黑色曲线：FFT 单边幅值谱 S(f)
 *   - 彩色曲线：每个局部高斯峰拟合结果
 *   - 灰色虚线：多个高斯峰叠加曲线
 *   - 竖线：主频 fd = mu_hat
 *   - 半透明区间：频率边界 [fL, fR]
 *
 * 为什么用 SVG：
 *   1) ROS/C++ 端不需要额外绘图库；
 *   2) SVG 是矢量图，可以放大查看；
 *   3) 后续可用浏览器/Inkscape/Illustrator 转 PNG/PDF。
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "imu_spectrum_gaussian_fit/gaussian_peak_fitter.hpp"

namespace imu_spectrum_gaussian_fit {

/**
 * @brief 简单 SVG 画图工具。
 *
 * 为了让整个 ROS/C++ 包尽量少依赖绘图库，这里直接用 C++ 写 SVG 文件：
 * - 黑色曲线：FFT 单边幅值谱；
 * - 彩色曲线：每个高斯谱峰拟合曲线；
 * - 灰色虚线：高斯叠加拟合曲线；
 * - 红色竖线：拟合得到的主频 mu；
 * - 橙色浅色区间：fL 到 fR 的频率边界。
 *
 * SVG 可以直接用浏览器打开，也可以插入论文初稿或转换为 PDF/PNG。
 */
class SvgPlotter {
public:
  static void plotSpectrumWithGaussianFits(const std::string& path,
                                           const Spectrum1D& spectrum,
                                           const std::vector<GaussianPeakParam>& peaks,
                                           const std::string& title,
                                           double min_freq_hz,
                                           double max_freq_hz) {
    if (spectrum.frequency_hz.empty()) {
      throw std::runtime_error("无法绘制空频谱");
    }

    const int W = 1200;
    const int H = 760;
    const int L = 90;
    const int R = 40;
    const int T = 70;
    const int B = 80;
    const int plotW = W - L - R;
    const int plotH = H - T - B;

    double x_min = min_freq_hz;
    double x_max = max_freq_hz;
    double y_max = 0.0;
    for (size_t i = 0; i < spectrum.frequency_hz.size(); ++i) {
      const double f = spectrum.frequency_hz[i];
      if (f >= x_min && f <= x_max) y_max = std::max(y_max, spectrum.amplitude[i]);
    }
    for (const auto& p : peaks) {
      if (p.valid) y_max = std::max(y_max, p.amplitude_A);
    }
    if (y_max <= 0.0) y_max = 1.0;
    y_max *= 1.15;

    auto sx = [&](double f) {
      return L + (f - x_min) / (x_max - x_min) * plotW;
    };
    auto sy = [&](double a) {
      return T + plotH - a / y_max * plotH;
    };

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      throw std::runtime_error("无法写入 SVG 文件: " + path);
    }

    ofs << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W << "' height='" << H << "' viewBox='0 0 " << W << " " << H << "'>\n";
    ofs << "<rect width='100%' height='100%' fill='white'/>\n";
    ofs << "<text x='" << W/2 << "' y='35' text-anchor='middle' font-size='24' font-family='Arial'>" << escape(title) << "</text>\n";

    // 坐标轴与网格
    ofs << "<rect x='" << L << "' y='" << T << "' width='" << plotW << "' height='" << plotH << "' fill='none' stroke='black' stroke-width='1.5'/>\n";
    for (int i = 0; i <= 10; ++i) {
      const double xf = x_min + (x_max - x_min) * i / 10.0;
      const double x = sx(xf);
      ofs << "<line x1='" << x << "' y1='" << T << "' x2='" << x << "' y2='" << T+plotH << "' stroke='#e6e6e6' stroke-width='1'/>\n";
      ofs << "<text x='" << x << "' y='" << T+plotH+25 << "' text-anchor='middle' font-size='14' font-family='Arial'>" << format(xf,1) << "</text>\n";
    }
    for (int i = 0; i <= 8; ++i) {
      const double ya = y_max * i / 8.0;
      const double y = sy(ya);
      ofs << "<line x1='" << L << "' y1='" << y << "' x2='" << L+plotW << "' y2='" << y << "' stroke='#e6e6e6' stroke-width='1'/>\n";
      ofs << "<text x='" << L-10 << "' y='" << y+5 << "' text-anchor='end' font-size='14' font-family='Arial'>" << format(ya,3) << "</text>\n";
    }
    ofs << "<text x='" << L+plotW/2 << "' y='" << H-25 << "' text-anchor='middle' font-size='18' font-family='Arial'>Frequency (Hz)</text>\n";
    ofs << "<text transform='translate(25 " << T+plotH/2 << ") rotate(-90)' text-anchor='middle' font-size='18' font-family='Arial'>Amplitude</text>\n";

    // 频率边界浅色区间
    const std::vector<std::string> colors = {"#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00", "#a65628"};
    for (size_t i = 0; i < peaks.size(); ++i) {
      const auto& p = peaks[i];
      if (!p.valid) continue;
      const double xl = std::max(x_min, p.f_left_hz);
      const double xr = std::min(x_max, p.f_right_hz);
      if (xr > xl) {
        ofs << "<rect x='" << sx(xl) << "' y='" << T << "' width='" << sx(xr)-sx(xl) << "' height='" << plotH
            << "' fill='" << colors[i % colors.size()] << "' fill-opacity='0.08'/>\n";
      }
    }

    // 原始频谱曲线
    writePolyline(ofs, spectrum.frequency_hz, spectrum.amplitude, sx, sy, x_min, x_max, "black", 2.0, "none");

    // 单个高斯拟合曲线
    for (size_t i = 0; i < peaks.size(); ++i) {
      const auto& p = peaks[i];
      if (!p.valid) continue;
      std::vector<double> y = GaussianPeakFitter::evaluateGaussianCurve(spectrum.frequency_hz, p);
      writePolyline(ofs, spectrum.frequency_hz, y, sx, sy, x_min, x_max, colors[i % colors.size()], 2.5, "none");
      ofs << "<line x1='" << sx(p.mu_hz) << "' y1='" << T << "' x2='" << sx(p.mu_hz) << "' y2='" << T+plotH
          << "' stroke='" << colors[i % colors.size()] << "' stroke-width='1.5' stroke-dasharray='5,5'/>\n";
    }

    // 高斯叠加曲线
    std::vector<double> sum_y = GaussianPeakFitter::evaluateGaussianSum(spectrum.frequency_hz, peaks);
    writePolyline(ofs, spectrum.frequency_hz, sum_y, sx, sy, x_min, x_max, "#666666", 2.0, "6,4");

    // 图例与参数表
    int legend_x = L + plotW - 330;
    int legend_y = T + 25;
    ofs << "<rect x='" << legend_x-15 << "' y='" << legend_y-22 << "' width='320' height='" << 35 + 28*static_cast<int>(peaks.size())
        << "' fill='white' fill-opacity='0.85' stroke='#cccccc'/>\n";
    ofs << "<text x='" << legend_x << "' y='" << legend_y << "' font-size='15' font-family='Arial'>Gaussian fitted peaks</text>\n";
    for (size_t i = 0; i < peaks.size(); ++i) {
      const auto& p = peaks[i];
      const int yy = legend_y + 25 + 24 * static_cast<int>(i);
      ofs << "<line x1='" << legend_x << "' y1='" << yy-5 << "' x2='" << legend_x+25 << "' y2='" << yy-5
          << "' stroke='" << colors[i % colors.size()] << "' stroke-width='3'/>\n";
      ofs << "<text x='" << legend_x+35 << "' y='" << yy << "' font-size='13' font-family='Arial'>P" << (i+1)
          << ": fd=" << format(p.mu_hz,2) << " Hz, var=" << format(p.variance_hz2,3) << "</text>\n";
    }

    ofs << "</svg>\n";
  }

private:
  template <typename SX, typename SY>
  static void writePolyline(std::ofstream& ofs,
                            const std::vector<double>& x,
                            const std::vector<double>& y,
                            SX sx,
                            SY sy,
                            double x_min,
                            double x_max,
                            const std::string& color,
                            double width,
                            const std::string& dasharray) {
    ofs << "<polyline fill='none' stroke='" << color << "' stroke-width='" << width << "'";
    if (dasharray != "none") ofs << " stroke-dasharray='" << dasharray << "'";
    ofs << " points='";
    for (size_t i = 0; i < x.size(); ++i) {
      if (x[i] < x_min || x[i] > x_max) continue;
      ofs << sx(x[i]) << "," << sy(y[i]) << " ";
    }
    ofs << "'/>\n";
  }

  static std::string format(double v, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << v;
    return oss.str();
  }

  static std::string escape(const std::string& s) {
    std::string out;
    for (char c : s) {
      if (c == '&') out += "&amp;";
      else if (c == '<') out += "&lt;";
      else if (c == '>') out += "&gt;";
      else out.push_back(c);
    }
    return out;
  }
};

}  // namespace imu_spectrum_gaussian_fit
