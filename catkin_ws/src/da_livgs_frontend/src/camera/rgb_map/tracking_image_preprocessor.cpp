#include "tracking_image_preprocessor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>

namespace da_livgs_frontend
{

TrackingImagePreprocessor::TrackingImagePreprocessor(
    const TrackingPreprocessConfig &config) : config_(config)
{
  config_.low_percentile = std::clamp(config_.low_percentile, 0.0, 1.0);
  config_.high_percentile = std::clamp(
      config_.high_percentile, config_.low_percentile, 1.0);
  config_.minimum_dynamic_range = std::max(0, config_.minimum_dynamic_range);
  config_.maximum_black_fraction = std::clamp(
      config_.maximum_black_fraction, 0.0, 1.0);
  config_.maximum_saturated_fraction = std::clamp(
      config_.maximum_saturated_fraction, 0.0, 1.0);
  config_.minimum_usable_range = std::max(1, config_.minimum_usable_range);
}

TrackingPreprocessDiagnostics TrackingImagePreprocessor::Apply(
    cv::Mat &gray) const
{
  const auto start = std::chrono::steady_clock::now();
  TrackingPreprocessDiagnostics diagnostics;
  diagnostics.mode = static_cast<int>(config_.mode);
  if (gray.empty() || gray.type() != CV_8UC1)
  {
    return diagnostics;
  }

  std::array<int, 256> histogram{};
  for (int row = 0; row < gray.rows; ++row)
  {
    const uchar *values = gray.ptr<uchar>(row);
    for (int col = 0; col < gray.cols; ++col)
      ++histogram[values[col]];
  }
  const int pixel_count = gray.rows * gray.cols;
  const auto percentile_value = [&](double fraction) {
    const int target = std::max(
        1, static_cast<int>(std::ceil(fraction * pixel_count)));
    int cumulative = 0;
    for (int value = 0; value < 256; ++value)
    {
      cumulative += histogram[value];
      if (cumulative >= target)
        return value;
    }
    return 255;
  };
  diagnostics.raw_low = percentile_value(config_.low_percentile);
  diagnostics.raw_high = percentile_value(config_.high_percentile);
  diagnostics.black_fraction = static_cast<double>(std::accumulate(
      histogram.begin(), histogram.begin() + 6, 0)) / pixel_count;
  diagnostics.saturated_fraction = static_cast<double>(std::accumulate(
      histogram.begin() + 250, histogram.end(), 0)) / pixel_count;

  if (config_.mode == TrackingPreprocessMode::LegacyClahe)
  {
    const cv::Size tile_size(
        std::max(static_cast<int>(gray.cols * 32.0 / 640), 4),
        std::max(static_cast<int>(gray.cols * 32.0 / 640), 4));
    cv::Mat equalized;
    cv::createCLAHE(3.0, tile_size)->apply(gray, equalized);
    gray = equalized;
    diagnostics.applied = true;
  }
  else if (config_.mode == TrackingPreprocessMode::ConditionalPercentile)
  {
    const int dynamic_range = diagnostics.raw_high - diagnostics.raw_low;
    const bool abnormal =
        dynamic_range < config_.minimum_dynamic_range ||
        diagnostics.black_fraction > config_.maximum_black_fraction ||
        diagnostics.saturated_fraction > config_.maximum_saturated_fraction;
    if (abnormal && dynamic_range >= config_.minimum_usable_range)
    {
      const double scale = 255.0 / dynamic_range;
      gray.convertTo(gray, CV_8UC1, scale, -scale * diagnostics.raw_low);
      diagnostics.applied = true;
    }
  }

  diagnostics.elapsed_ms = std::chrono::duration_cast<
      std::chrono::duration<double, std::milli>>(
          std::chrono::steady_clock::now() - start).count();
  return diagnostics;
}

} // namespace da_livgs_frontend
