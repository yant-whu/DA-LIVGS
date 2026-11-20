#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <vector>

namespace da_livgs_frontend
{

struct ForwardBackwardKltConfig
{
  bool enabled = false;
  double maximum_error = 0.5;
  int backward_pyramid_levels = 1;
};

struct ForwardBackwardKltDiagnostics
{
  bool enabled = false;
  int input_count = 0;
  int accepted_count = 0;
  int rejected_forward = 0;
  int rejected_backward = 0;
  int rejected_cycle_error = 0;
  double mean_cycle_error = 0.0;
  double p95_cycle_error = 0.0;
  double elapsed_ms = 0.0;
};

ForwardBackwardKltDiagnostics FilterForwardBackwardTracks(
    const std::vector<cv::Point2f> &original_points,
    const std::vector<cv::Point2f> &reverse_points,
    const std::vector<unsigned char> &reverse_status,
    double maximum_error,
    std::vector<unsigned char> *forward_status);

}  // namespace da_livgs_frontend
