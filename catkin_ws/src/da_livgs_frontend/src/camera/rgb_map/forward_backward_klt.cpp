#include "forward_backward_klt.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace da_livgs_frontend
{

ForwardBackwardKltDiagnostics FilterForwardBackwardTracks(
    const std::vector<cv::Point2f> &original_points,
    const std::vector<cv::Point2f> &reverse_points,
    const std::vector<unsigned char> &reverse_status,
    double maximum_error,
    std::vector<unsigned char> *forward_status)
{
  ForwardBackwardKltDiagnostics diagnostics;
  diagnostics.enabled = true;
  if (!forward_status)
    return diagnostics;

  diagnostics.input_count = static_cast<int>(forward_status->size());
  const double threshold = std::max(0.0, maximum_error);
  std::vector<double> valid_cycle_errors;
  valid_cycle_errors.reserve(forward_status->size());

  for (size_t index = 0; index < forward_status->size(); ++index)
  {
    if (!(*forward_status)[index])
    {
      ++diagnostics.rejected_forward;
      continue;
    }
    if (index >= original_points.size() || index >= reverse_points.size() ||
        index >= reverse_status.size() || !reverse_status[index])
    {
      (*forward_status)[index] = 0;
      ++diagnostics.rejected_backward;
      continue;
    }

    const cv::Point2f difference = original_points[index] - reverse_points[index];
    const double cycle_error = std::hypot(
        static_cast<double>(difference.x), static_cast<double>(difference.y));
    if (!std::isfinite(cycle_error))
    {
      (*forward_status)[index] = 0;
      ++diagnostics.rejected_backward;
      continue;
    }
    valid_cycle_errors.push_back(cycle_error);
    constexpr double kThresholdTolerance = 1e-6;
    if (cycle_error > threshold + kThresholdTolerance)
    {
      (*forward_status)[index] = 0;
      ++diagnostics.rejected_cycle_error;
      continue;
    }
    ++diagnostics.accepted_count;
  }

  if (!valid_cycle_errors.empty())
  {
    diagnostics.mean_cycle_error = std::accumulate(
        valid_cycle_errors.begin(), valid_cycle_errors.end(), 0.0) /
        valid_cycle_errors.size();
    const size_t p95_index = static_cast<size_t>(std::ceil(
        0.95 * static_cast<double>(valid_cycle_errors.size()))) - 1;
    std::nth_element(valid_cycle_errors.begin(),
                     valid_cycle_errors.begin() + p95_index,
                     valid_cycle_errors.end());
    diagnostics.p95_cycle_error = valid_cycle_errors[p95_index];
  }
  return diagnostics;
}

}  // namespace da_livgs_frontend
