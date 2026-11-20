#pragma once

#include <opencv2/core.hpp>

namespace da_livgs_frontend
{

enum class TrackingPreprocessMode
{
  Raw = 0,
  LegacyClahe = 1,
  ConditionalPercentile = 2,
};

struct TrackingPreprocessConfig
{
  TrackingPreprocessMode mode = TrackingPreprocessMode::LegacyClahe;
  double low_percentile = 0.02;
  double high_percentile = 0.98;
  int minimum_dynamic_range = 80;
  double maximum_black_fraction = 0.10;
  double maximum_saturated_fraction = 0.10;
  int minimum_usable_range = 8;
};

struct TrackingPreprocessDiagnostics
{
  int mode = static_cast<int>(TrackingPreprocessMode::LegacyClahe);
  bool applied = false;
  int raw_low = 0;
  int raw_high = 0;
  double black_fraction = 0.0;
  double saturated_fraction = 0.0;
  double elapsed_ms = 0.0;
};

class TrackingImagePreprocessor
{
public:
  explicit TrackingImagePreprocessor(
      const TrackingPreprocessConfig &config = {});

  TrackingPreprocessDiagnostics Apply(cv::Mat &gray) const;

private:
  TrackingPreprocessConfig config_;
};

} // namespace da_livgs_frontend
