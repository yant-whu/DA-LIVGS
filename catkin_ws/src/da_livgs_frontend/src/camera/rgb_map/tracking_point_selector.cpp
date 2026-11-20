#include "tracking_point_selector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace da_livgs_frontend
{
namespace
{

struct ScoredCandidate
{
  size_t source_index = 0;
  cv::Point2f pixel;
  float quality = 0.0F;
  int cell = 0;
};

int ClampCell(double coordinate, int extent, int cells)
{
  if (extent <= 0 || cells <= 1)
    return 0;
  return std::min(cells - 1, std::max(0,
      static_cast<int>(coordinate * cells / extent)));
}

std::pair<int, int> SpacingCell(const cv::Point2f &pixel, double spacing)
{
  const double safe_spacing = std::max(spacing, 1.0);
  return {static_cast<int>(std::lround(pixel.x / safe_spacing)),
          static_cast<int>(std::lround(pixel.y / safe_spacing))};
}

double GridCoverage(const std::vector<int> &counts)
{
  if (counts.empty())
    return 0.0;
  return static_cast<double>(std::count_if(
      counts.begin(), counts.end(), [](int count) { return count > 0; })) /
      counts.size();
}

double LocalCornerQuality(const cv::Mat &gray, int u, int v, int radius)
{
  double gradient_x_squared = 0.0;
  double gradient_xy = 0.0;
  double gradient_y_squared = 0.0;
  int sample_count = 0;
  for (int row = v - radius; row <= v + radius; ++row)
  {
    const uchar *previous = gray.ptr<uchar>(row - 1);
    const uchar *current = gray.ptr<uchar>(row);
    const uchar *next = gray.ptr<uchar>(row + 1);
    for (int col = u - radius; col <= u + radius; ++col)
    {
      const double gradient_x = current[col + 1] - current[col - 1];
      const double gradient_y = next[col] - previous[col];
      gradient_x_squared += gradient_x * gradient_x;
      gradient_xy += gradient_x * gradient_y;
      gradient_y_squared += gradient_y * gradient_y;
      ++sample_count;
    }
  }
  if (sample_count == 0)
    return 0.0;
  const double trace = gradient_x_squared + gradient_y_squared;
  const double difference = gradient_x_squared - gradient_y_squared;
  const double discriminant = std::sqrt(std::max(
      0.0, difference * difference + 4.0 * gradient_xy * gradient_xy));
  const double minimum_eigenvalue = 0.5 * (trace - discriminant);
  constexpr double maximum_gradient_squared = 4.0 * 255.0 * 255.0;
  return minimum_eigenvalue /
      (sample_count * maximum_gradient_squared);
}

} // namespace

TrackingPointSelector::TrackingPointSelector(
    const TrackingPointSelectionConfig &config) : config_(config)
{
  config_.border_pixels = std::max(config_.border_pixels, 2);
  config_.minimum_intensity = std::max(0,
      std::min(255, config_.minimum_intensity));
  config_.maximum_intensity = std::max(config_.minimum_intensity,
      std::min(255, config_.maximum_intensity));
  config_.maximum_extreme_fraction = std::max(0.0,
      std::min(1.0, config_.maximum_extreme_fraction));
  config_.minimum_corner_eigenvalue = std::max(
      0.0, config_.minimum_corner_eigenvalue);
  config_.grid_rows = std::max(config_.grid_rows, 1);
  config_.grid_cols = std::max(config_.grid_cols, 1);
  config_.minimum_tracked_points = std::max(
      config_.minimum_tracked_points, 0);
}

TrackingPointSelectionResult TrackingPointSelector::Select(
    const cv::Mat &gray,
    const std::vector<TrackingPointCandidate> &candidates,
    const std::vector<cv::Point2f> &occupied_pixels,
    size_t maximum_selected,
    size_t minimum_selected,
    double minimum_spacing_pixels) const
{
  const auto start = std::chrono::steady_clock::now();
  TrackingPointSelectionResult result;
  auto &diagnostics = result.diagnostics;
  diagnostics.mode = static_cast<int>(config_.mode);
  diagnostics.proposed = static_cast<int>(candidates.size());
  minimum_selected = std::min(minimum_selected, maximum_selected);

  if (maximum_selected == 0 || candidates.empty())
  {
    diagnostics.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return result;
  }

  if (config_.mode == TrackingPointSelectionMode::Disabled)
  {
    const size_t count = std::min(maximum_selected, candidates.size());
    result.source_indices.reserve(count);
    for (size_t index = 0; index < count; ++index)
      result.source_indices.push_back(candidates[index].source_index);
    diagnostics.quality_eligible = static_cast<int>(count);
    diagnostics.selected = static_cast<int>(count);
    diagnostics.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return result;
  }

  if (gray.empty() || gray.type() != CV_8UC1)
  {
    diagnostics.rejected_gradient = diagnostics.proposed;
    diagnostics.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return result;
  }

  const int cell_count = config_.grid_rows * config_.grid_cols;
  std::vector<int> grid_counts(cell_count, 0);
  std::set<std::pair<int, int>> spacing_occupied;
  for (const cv::Point2f &pixel : occupied_pixels)
  {
    if (pixel.x < 0.0F || pixel.x >= gray.cols ||
        pixel.y < 0.0F || pixel.y >= gray.rows)
      continue;
    const int col = ClampCell(pixel.x, gray.cols, config_.grid_cols);
    const int row = ClampCell(pixel.y, gray.rows, config_.grid_rows);
    ++grid_counts[row * config_.grid_cols + col];
    spacing_occupied.insert(SpacingCell(pixel, minimum_spacing_pixels));
  }
  diagnostics.grid_coverage_before = GridCoverage(grid_counts);

  std::vector<ScoredCandidate> eligible;
  std::vector<ScoredCandidate> fallback;
  eligible.reserve(candidates.size());
  fallback.reserve(candidates.size());
  constexpr int patch_radius = 2;
  for (const TrackingPointCandidate &candidate : candidates)
  {
    const int u = static_cast<int>(std::lround(candidate.pixel.x));
    const int v = static_cast<int>(std::lround(candidate.pixel.y));
    const int margin = std::max(config_.border_pixels, patch_radius + 1);
    if (u < margin || u >= gray.cols - margin ||
        v < margin || v >= gray.rows - margin)
    {
      ++diagnostics.rejected_border;
      continue;
    }

    int extreme_count = 0;
    int patch_count = 0;
    for (int row = v - patch_radius; row <= v + patch_radius; ++row)
    {
      const uchar *pixels = gray.ptr<uchar>(row);
      for (int col = u - patch_radius; col <= u + patch_radius; ++col)
      {
        const int intensity = pixels[col];
        if (intensity <= config_.minimum_intensity ||
            intensity >= config_.maximum_intensity)
          ++extreme_count;
        ++patch_count;
      }
    }
    if (patch_count == 0 ||
        static_cast<double>(extreme_count) / patch_count >
            config_.maximum_extreme_fraction)
    {
      ++diagnostics.rejected_extreme;
      continue;
    }

    const int col = ClampCell(candidate.pixel.x, gray.cols, config_.grid_cols);
    const int row = ClampCell(candidate.pixel.y, gray.rows, config_.grid_rows);
    const double quality = LocalCornerQuality(gray, u, v, patch_radius);
    if (!std::isfinite(quality) ||
        quality < config_.minimum_corner_eigenvalue)
    {
      ++diagnostics.rejected_gradient;
      if (std::isfinite(quality))
      {
        fallback.push_back(
            {candidate.source_index, candidate.pixel,
             static_cast<float>(quality),
             row * config_.grid_cols + col});
      }
      continue;
    }

    eligible.push_back(
        {candidate.source_index, candidate.pixel, static_cast<float>(quality),
         row * config_.grid_cols + col});
  }
  diagnostics.quality_eligible = static_cast<int>(eligible.size());

  double selected_quality_sum = 0.0;
  const auto accept = [&](const ScoredCandidate &candidate,
                          TrackingPointSelectionResult *selection,
                          std::set<std::pair<int, int>> *occupied,
                          std::vector<int> *counts) {
    const auto spacing_cell = SpacingCell(
        candidate.pixel, minimum_spacing_pixels);
    if (occupied->count(spacing_cell) != 0)
    {
      ++selection->diagnostics.rejected_spacing;
      return false;
    }
    occupied->insert(spacing_cell);
    ++(*counts)[candidate.cell];
    selection->source_indices.push_back(candidate.source_index);
    return true;
  };

  const auto select_candidates = [&](const std::vector<ScoredCandidate> &pool,
                                     bool use_grid,
                                     size_t selection_limit) {
    if (!use_grid)
    {
      for (const ScoredCandidate &candidate : pool)
      {
        if (result.source_indices.size() >= selection_limit)
          break;
        if (accept(candidate, &result, &spacing_occupied, &grid_counts))
          selected_quality_sum += candidate.quality;
      }
      return;
    }

    std::vector<std::vector<ScoredCandidate>> buckets(cell_count);
    for (const ScoredCandidate &candidate : pool)
      buckets[candidate.cell].push_back(candidate);
    for (auto &bucket : buckets)
    {
      std::stable_sort(bucket.begin(), bucket.end(),
          [](const ScoredCandidate &left, const ScoredCandidate &right) {
            if (left.quality != right.quality)
              return left.quality > right.quality;
            return left.source_index < right.source_index;
          });
    }
    std::vector<size_t> offsets(cell_count, 0);
    while (result.source_indices.size() < selection_limit)
    {
      int selected_cell = -1;
      int minimum_count = std::numeric_limits<int>::max();
      for (int cell = 0; cell < cell_count; ++cell)
      {
        if (offsets[cell] >= buckets[cell].size())
          continue;
        if (grid_counts[cell] < minimum_count)
        {
          minimum_count = grid_counts[cell];
          selected_cell = cell;
        }
      }
      if (selected_cell < 0)
        break;

      const ScoredCandidate &candidate =
          buckets[selected_cell][offsets[selected_cell]++];
      if (accept(candidate, &result, &spacing_occupied, &grid_counts))
        selected_quality_sum += candidate.quality;
    }
  };

  const bool use_grid =
      config_.mode == TrackingPointSelectionMode::QualityGrid;
  select_candidates(eligible, use_grid, maximum_selected);
  if (result.source_indices.size() < minimum_selected)
  {
    const size_t before_fallback = result.source_indices.size();
    if (!use_grid)
    {
      std::stable_sort(fallback.begin(), fallback.end(),
          [](const ScoredCandidate &left, const ScoredCandidate &right) {
            if (left.quality != right.quality)
              return left.quality > right.quality;
            return left.source_index < right.source_index;
          });
    }
    select_candidates(fallback, use_grid, minimum_selected);
    diagnostics.fallback_selected = static_cast<int>(
        result.source_indices.size() - before_fallback);
  }

  diagnostics.selected = static_cast<int>(result.source_indices.size());
  diagnostics.grid_coverage_after = GridCoverage(grid_counts);
  if (diagnostics.selected > 0)
    diagnostics.selected_mean_quality =
        selected_quality_sum / diagnostics.selected;
  diagnostics.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

} // namespace da_livgs_frontend
