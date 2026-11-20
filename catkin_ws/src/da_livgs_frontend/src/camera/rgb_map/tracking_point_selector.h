#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <vector>

namespace da_livgs_frontend
{

enum class TrackingPointSelectionMode
{
  Disabled = 0,
  Quality = 1,
  QualityGrid = 2,
};

struct TrackingPointSelectionConfig
{
  TrackingPointSelectionMode mode = TrackingPointSelectionMode::Disabled;
  int border_pixels = 12;
  int minimum_intensity = 5;
  int maximum_intensity = 250;
  double maximum_extreme_fraction = 0.8;
  double minimum_corner_eigenvalue = 1e-4;
  int grid_rows = 4;
  int grid_cols = 6;
  int minimum_tracked_points = 500;
};

struct TrackingPointCandidate
{
  size_t source_index = 0;
  cv::Point2f pixel;
};

struct TrackingPointSelectionDiagnostics
{
  int mode = 0;
  int proposed = 0;
  int quality_eligible = 0;
  int selected = 0;
  int rejected_border = 0;
  int rejected_extreme = 0;
  int rejected_gradient = 0;
  int rejected_spacing = 0;
  int fallback_selected = 0;
  double grid_coverage_before = 0.0;
  double grid_coverage_after = 0.0;
  double selected_mean_quality = 0.0;
  double elapsed_ms = 0.0;
};

struct TrackingPointSelectionResult
{
  std::vector<size_t> source_indices;
  TrackingPointSelectionDiagnostics diagnostics;
};

class TrackingPointSelector
{
public:
  explicit TrackingPointSelector(
      const TrackingPointSelectionConfig &config = {});

  TrackingPointSelectionResult Select(
      const cv::Mat &gray,
      const std::vector<TrackingPointCandidate> &candidates,
      const std::vector<cv::Point2f> &occupied_pixels,
      size_t maximum_selected,
      size_t minimum_selected,
      double minimum_spacing_pixels) const;

  const TrackingPointSelectionConfig &Config() const { return config_; }

private:
  TrackingPointSelectionConfig config_;
};

} // namespace da_livgs_frontend
