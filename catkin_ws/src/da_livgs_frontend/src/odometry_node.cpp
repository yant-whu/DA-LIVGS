/*
 * DA-LIVGS: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ros/package.h>
#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <cstdlib>
#ifdef _OPENMP
#include <omp.h>
#endif

#include <odom/odometry_manager.h>

using namespace da_livgs_frontend;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "da_livgs_frontend");
  ros::NodeHandle nh("~");

  std::string config_path;
  nh.param<std::string>("config_path", config_path, "da_livgs.yaml");
  ROS_INFO("Odometry load %s.", config_path.c_str());

  YAML::Node config_node = YAML::LoadFile(config_path);

  int odometry_mode_override = -1;
  int if_3dgs_override = -1;
  double bag_start_override = -1.0;
  double bag_durr_override = -2.0;
  int diagnostics_override = -1;
  int adaptive_fusion_override = -1;
  int adaptive_fusion_mode_override = -1;
  int adaptive_fusion_warmup_frames_override = -1;
  int adaptive_fusion_minimum_visual_samples_override = -1;
  int compact_pnp_override = -1;
  int support_conditioned_factor_override = -1;
  int persistent_degeneracy_shadow_override = -1;
  int persistent_degeneracy_action_override = -1;
  int mapping_backpressure_override = -1;
  double mapping_backpressure_timeout_seconds_override = -1.0;
  int directional_fusion_override = -1;
  int directional_fusion_mode_override = -1;
  int tracking_preprocess_mode_override = -1;
  int tracking_point_selection_mode_override = -1;
  int tracking_forward_backward_override = -1;
  double tracking_forward_backward_max_error_override = -1.0;
  double directional_minimum_visual_weight_override = -1.0;
  double directional_activation_enter_score_override = -1.0;
  double directional_activation_exit_score_override = -1.0;
  int directional_activation_enter_frames_override = -1;
  int directional_activation_exit_frames_override = -1;
  int directional_minimum_output_rank_override = -1;
  double directional_minimum_trace_ratio_override = -1.0;
  double directional_safety_trace_zero_override = -1.0;
  double directional_safety_trace_full_override = -1.0;
  double directional_safety_alpha_override = -1.0;
  double directional_safety_fall_alpha_override = -1.0;
  int gaussian_feedback_shadow_override = -1;
  double gaussian_feedback_shadow_maximum_staleness_ms_override = -1.0;
  double gaussian_feedback_shadow_linger_seconds_override = -1.0;
  std::string gaussian_feedback_shadow_log_path_override;
  int gaussian_feedback_transport_shadow_override = -1;
  double gaussian_feedback_transport_maximum_source_age_ms_override = -1.0;
  double gaussian_feedback_transport_state_ttl_ms_override = -1.0;
  double gaussian_feedback_transport_state_alpha_override = -1.0;
  int gaussian_endpoint_prior_shadow_override = -1;
  int gaussian_endpoint_prior_allow_validation_override = -1;
  double gaussian_endpoint_prior_maximum_source_age_ms_override = -1.0;
  double gaussian_endpoint_prior_minimum_confidence_override = -1.0;
  int gaussian_endpoint_prior_minimum_consecutive_accepted_override = -1;
  double gaussian_endpoint_prior_gain_override = -1.0;
  int gaussian_endpoint_factor_override = -1;
  int gaussian_endpoint_factor_apply_override = -1;
  int gaussian_endpoint_factor_validation_only_override = -1;
  double gaussian_endpoint_factor_maximum_application_age_ms_override = -1.0;
  double gaussian_endpoint_factor_rotation_sqrt_information_override = -1.0;
  double gaussian_endpoint_factor_translation_sqrt_information_override = -1.0;
  std::string gaussian_endpoint_factor_log_path_override;
  int gaussian_endpoint_replay_override = -1;
  std::string gaussian_endpoint_replay_path_override;
  std::string gaussian_endpoint_replay_log_path_override;
  int gaussian_endpoint_trust_region_override = -1;
  int gaussian_endpoint_trust_region_apply_override = -1;
  std::string gaussian_endpoint_trust_region_log_path_override;
  int gaussian_mapping_pose_overlay_override = -1;
  int gaussian_mapping_pose_overlay_recovery_override = -1;
  int gaussian_mapping_pose_overlay_validation_corruption_override = -1;
  int gaussian_mapping_pose_overlay_source_override = -1;
  double gaussian_mapping_pose_overlay_candidate_gain_override = -1.0;
  double gaussian_mapping_pose_overlay_candidate_rotation_gain_ratio_override = -1.0;
  double gaussian_mapping_pose_overlay_candidate_translation_cap_metres_override = -1.0;
  double gaussian_mapping_pose_overlay_candidate_minimum_confidence_override = -1.0;
  int gaussian_mapping_pose_overlay_application_mask_override = -1;
  int gaussian_mapping_pose_overlay_independent_application_masks_override = -1;
  int gaussian_mapping_pose_overlay_recovery_application_mask_override = -1;
  int gaussian_mapping_pose_overlay_corruption_application_mask_override = -1;
  double gaussian_mapping_pose_overlay_state_timeout_ms_override = -1.0;
  double gaussian_mapping_pose_overlay_maximum_translation_metres_override = -1.0;
  double gaussian_mapping_pose_overlay_maximum_rotation_degrees_override = -1.0;
  double gaussian_mapping_pose_overlay_maximum_corruption_translation_metres_override = -1.0;
  double gaussian_mapping_pose_overlay_maximum_corruption_rotation_degrees_override = -1.0;
  std::string gaussian_mapping_pose_overlay_path_override;
  std::string gaussian_mapping_pose_overlay_log_path_override;
  std::string diagnostics_path_override;
  double observability_length_scale = 1.0;
  int deterministic_seed = -1;
  bool deterministic_single_thread = false;
  bool deterministic_map_update = false;
  bool deterministic_solver_ordering = false;
  nh.param<int>("odometry_mode_override", odometry_mode_override, -1);
  nh.param<int>("if_3dgs_override", if_3dgs_override, -1);
  nh.param<double>("bag_start_override", bag_start_override, -1.0);
  nh.param<double>("bag_durr_override", bag_durr_override, -2.0);
  nh.param<int>("diagnostics_override", diagnostics_override, -1);
  nh.param<int>("adaptive_fusion_override", adaptive_fusion_override, -1);
  nh.param<int>("adaptive_fusion_mode_override",
                adaptive_fusion_mode_override, -1);
  nh.param<int>("adaptive_fusion_warmup_frames_override",
                adaptive_fusion_warmup_frames_override, -1);
  nh.param<int>("adaptive_fusion_minimum_visual_samples_override",
                adaptive_fusion_minimum_visual_samples_override, -1);
  nh.param<int>("compact_pnp_override", compact_pnp_override, -1);
  nh.param<int>("support_conditioned_factor_override",
                support_conditioned_factor_override, -1);
  nh.param<int>("persistent_degeneracy_shadow_override",
                persistent_degeneracy_shadow_override, -1);
  nh.param<int>("persistent_degeneracy_action_override",
                persistent_degeneracy_action_override, -1);
  nh.param<int>("mapping_backpressure_override",
                mapping_backpressure_override, -1);
  nh.param<double>("mapping_backpressure_timeout_seconds_override",
                   mapping_backpressure_timeout_seconds_override, -1.0);
  nh.param<int>("directional_fusion_override", directional_fusion_override, -1);
  nh.param<int>("directional_fusion_mode_override",
                directional_fusion_mode_override, -1);
  nh.param<int>("tracking_preprocess_mode_override",
                tracking_preprocess_mode_override, -1);
  nh.param<int>("tracking_point_selection_mode_override",
                tracking_point_selection_mode_override, -1);
  nh.param<int>("tracking_forward_backward_override",
                tracking_forward_backward_override, -1);
  nh.param<double>("tracking_forward_backward_max_error_override",
                   tracking_forward_backward_max_error_override, -1.0);
  nh.param<double>("directional_minimum_visual_weight_override",
                   directional_minimum_visual_weight_override, -1.0);
  nh.param<double>("directional_activation_enter_score_override",
                   directional_activation_enter_score_override, -1.0);
  nh.param<double>("directional_activation_exit_score_override",
                   directional_activation_exit_score_override, -1.0);
  nh.param<int>("directional_activation_enter_frames_override",
                directional_activation_enter_frames_override, -1);
  nh.param<int>("directional_activation_exit_frames_override",
                directional_activation_exit_frames_override, -1);
  nh.param<int>("directional_minimum_output_rank_override",
                directional_minimum_output_rank_override, -1);
  nh.param<double>("directional_minimum_trace_ratio_override",
                   directional_minimum_trace_ratio_override, -1.0);
  nh.param<double>("directional_safety_trace_zero_override",
                   directional_safety_trace_zero_override, -1.0);
  nh.param<double>("directional_safety_trace_full_override",
                   directional_safety_trace_full_override, -1.0);
  nh.param<double>("directional_safety_alpha_override",
                   directional_safety_alpha_override, -1.0);
  nh.param<double>("directional_safety_fall_alpha_override",
                   directional_safety_fall_alpha_override, -1.0);
  nh.param<int>("gaussian_feedback_shadow_override",
                gaussian_feedback_shadow_override, -1);
  nh.param<double>("gaussian_feedback_shadow_maximum_staleness_ms_override",
                   gaussian_feedback_shadow_maximum_staleness_ms_override,
                   -1.0);
  nh.param<double>("gaussian_feedback_shadow_linger_seconds_override",
                   gaussian_feedback_shadow_linger_seconds_override, -1.0);
  nh.param<std::string>("gaussian_feedback_shadow_log_path_override",
                        gaussian_feedback_shadow_log_path_override, "");
  nh.param<int>("gaussian_feedback_transport_shadow_override",
                gaussian_feedback_transport_shadow_override, -1);
  nh.param<double>("gaussian_feedback_transport_maximum_source_age_ms_override",
                   gaussian_feedback_transport_maximum_source_age_ms_override,
                   -1.0);
  nh.param<double>("gaussian_feedback_transport_state_ttl_ms_override",
                   gaussian_feedback_transport_state_ttl_ms_override, -1.0);
  nh.param<double>("gaussian_feedback_transport_state_alpha_override",
                   gaussian_feedback_transport_state_alpha_override, -1.0);
  nh.param<int>("gaussian_endpoint_prior_shadow_override",
                gaussian_endpoint_prior_shadow_override, -1);
  nh.param<int>("gaussian_endpoint_prior_allow_validation_override",
                gaussian_endpoint_prior_allow_validation_override, -1);
  nh.param<double>("gaussian_endpoint_prior_maximum_source_age_ms_override",
                   gaussian_endpoint_prior_maximum_source_age_ms_override,
                   -1.0);
  nh.param<double>("gaussian_endpoint_prior_minimum_confidence_override",
                   gaussian_endpoint_prior_minimum_confidence_override, -1.0);
  nh.param<int>(
      "gaussian_endpoint_prior_minimum_consecutive_accepted_override",
      gaussian_endpoint_prior_minimum_consecutive_accepted_override, -1);
  nh.param<double>("gaussian_endpoint_prior_gain_override",
                   gaussian_endpoint_prior_gain_override, -1.0);
  nh.param<int>("gaussian_endpoint_factor_override",
                gaussian_endpoint_factor_override, -1);
  nh.param<int>("gaussian_endpoint_factor_apply_override",
                gaussian_endpoint_factor_apply_override, -1);
  nh.param<int>("gaussian_endpoint_factor_validation_only_override",
                gaussian_endpoint_factor_validation_only_override, -1);
  nh.param<double>("gaussian_endpoint_factor_maximum_application_age_ms_override",
                   gaussian_endpoint_factor_maximum_application_age_ms_override,
                   -1.0);
  nh.param<double>("gaussian_endpoint_factor_rotation_sqrt_information_override",
                   gaussian_endpoint_factor_rotation_sqrt_information_override,
                   -1.0);
  nh.param<double>("gaussian_endpoint_factor_translation_sqrt_information_override",
                   gaussian_endpoint_factor_translation_sqrt_information_override,
                   -1.0);
  nh.param<std::string>("gaussian_endpoint_factor_log_path_override",
                        gaussian_endpoint_factor_log_path_override, "");
  nh.param<int>("gaussian_endpoint_replay_override",
                gaussian_endpoint_replay_override, -1);
  nh.param<std::string>("gaussian_endpoint_replay_path_override",
                        gaussian_endpoint_replay_path_override, "");
  nh.param<std::string>("gaussian_endpoint_replay_log_path_override",
                        gaussian_endpoint_replay_log_path_override, "");
  nh.param<int>("gaussian_endpoint_trust_region_override",
                gaussian_endpoint_trust_region_override, -1);
  nh.param<int>("gaussian_endpoint_trust_region_apply_override",
                gaussian_endpoint_trust_region_apply_override, -1);
  nh.param<std::string>("gaussian_endpoint_trust_region_log_path_override",
                        gaussian_endpoint_trust_region_log_path_override, "");
  nh.param<int>("gaussian_mapping_pose_overlay_override",
                gaussian_mapping_pose_overlay_override, -1);
  nh.param<int>("gaussian_mapping_pose_overlay_recovery_override",
                gaussian_mapping_pose_overlay_recovery_override, -1);
  nh.param<int>("gaussian_mapping_pose_overlay_validation_corruption_override",
                gaussian_mapping_pose_overlay_validation_corruption_override,
                -1);
  nh.param<int>("gaussian_mapping_pose_overlay_source_override",
                gaussian_mapping_pose_overlay_source_override, -1);
  nh.param<double>("gaussian_mapping_pose_overlay_candidate_gain_override",
                   gaussian_mapping_pose_overlay_candidate_gain_override,
                   -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_candidate_rotation_gain_ratio_override",
      gaussian_mapping_pose_overlay_candidate_rotation_gain_ratio_override,
      -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_candidate_translation_cap_metres_override",
      gaussian_mapping_pose_overlay_candidate_translation_cap_metres_override,
      -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_candidate_minimum_confidence_override",
      gaussian_mapping_pose_overlay_candidate_minimum_confidence_override,
      -1.0);
  nh.param<int>("gaussian_mapping_pose_overlay_application_mask_override",
                gaussian_mapping_pose_overlay_application_mask_override, -1);
  nh.param<int>(
      "gaussian_mapping_pose_overlay_independent_application_masks_override",
      gaussian_mapping_pose_overlay_independent_application_masks_override,
      -1);
  nh.param<int>(
      "gaussian_mapping_pose_overlay_recovery_application_mask_override",
      gaussian_mapping_pose_overlay_recovery_application_mask_override, -1);
  nh.param<int>(
      "gaussian_mapping_pose_overlay_corruption_application_mask_override",
      gaussian_mapping_pose_overlay_corruption_application_mask_override, -1);
  nh.param<double>("gaussian_mapping_pose_overlay_state_timeout_ms_override",
                   gaussian_mapping_pose_overlay_state_timeout_ms_override,
                   -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_maximum_translation_metres_override",
      gaussian_mapping_pose_overlay_maximum_translation_metres_override,
      -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_maximum_rotation_degrees_override",
      gaussian_mapping_pose_overlay_maximum_rotation_degrees_override, -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_maximum_corruption_translation_metres_override",
      gaussian_mapping_pose_overlay_maximum_corruption_translation_metres_override,
      -1.0);
  nh.param<double>(
      "gaussian_mapping_pose_overlay_maximum_corruption_rotation_degrees_override",
      gaussian_mapping_pose_overlay_maximum_corruption_rotation_degrees_override,
      -1.0);
  nh.param<std::string>("gaussian_mapping_pose_overlay_path_override",
                        gaussian_mapping_pose_overlay_path_override, "");
  nh.param<std::string>("gaussian_mapping_pose_overlay_log_path_override",
                        gaussian_mapping_pose_overlay_log_path_override, "");
  nh.param<std::string>("diagnostics_path_override",
                        diagnostics_path_override, "");
  nh.param<double>("observability_length_scale",
                   observability_length_scale, 1.0);
  nh.param<int>("deterministic_seed", deterministic_seed, -1);
  nh.param<bool>("deterministic_single_thread",
                 deterministic_single_thread, false);
  nh.param<bool>("deterministic_map_update",
                 deterministic_map_update, false);
  nh.param<bool>("deterministic_solver_ordering",
                 deterministic_solver_ordering, false);
  if (odometry_mode_override >= 0) {
    config_node["odometry_mode"] = odometry_mode_override;
  }
  if (if_3dgs_override >= 0) {
    config_node["if_3dgs"] = (if_3dgs_override != 0);
  }
  if (bag_start_override >= 0.0) {
    config_node["bag_start"] = bag_start_override;
  }
  if (bag_durr_override >= -1.0) {
    config_node["bag_durr"] = bag_durr_override;
  }
  if (diagnostics_override >= 0) {
    config_node["diagnostics_enabled"] = (diagnostics_override != 0);
  }
  if (adaptive_fusion_override >= 0) {
    config_node["adaptive_fusion_enabled"] =
        (adaptive_fusion_override != 0);
  }
  if (adaptive_fusion_mode_override >= 1) {
    config_node["adaptive_fusion_mode"] = adaptive_fusion_mode_override;
  }
  if (adaptive_fusion_warmup_frames_override > 0) {
    config_node["adaptive_fusion_warmup_frames"] =
        adaptive_fusion_warmup_frames_override;
  }
  if (adaptive_fusion_minimum_visual_samples_override > 0) {
    config_node["adaptive_fusion_minimum_visual_samples"] =
        adaptive_fusion_minimum_visual_samples_override;
  }
  if (compact_pnp_override >= 0) {
    config_node["compact_pnp_enabled"] = (compact_pnp_override != 0);
  }
  if (support_conditioned_factor_override >= 0) {
    config_node["support_conditioned_factor_enabled"] =
        (support_conditioned_factor_override != 0);
  }
  if (persistent_degeneracy_shadow_override >= 0) {
    config_node["persistent_degeneracy_shadow_enabled"] =
        (persistent_degeneracy_shadow_override != 0);
  }
  if (persistent_degeneracy_action_override >= 0) {
    config_node["persistent_degeneracy_action_enabled"] =
        (persistent_degeneracy_action_override != 0);
  }
  if (mapping_backpressure_override >= 0) {
    config_node["mapping_backpressure_enabled"] =
        (mapping_backpressure_override != 0);
  }
  if (mapping_backpressure_timeout_seconds_override > 0.0) {
    config_node["mapping_backpressure_timeout_seconds"] =
        mapping_backpressure_timeout_seconds_override;
  }
  if (directional_fusion_override >= 0) {
    config_node["directional_fusion_enabled"] =
        (directional_fusion_override != 0);
  }
  if (directional_fusion_mode_override >= 1) {
    config_node["directional_fusion_mode"] =
        directional_fusion_mode_override;
  }
  if (tracking_preprocess_mode_override >= 0) {
    config_node["tracking_preprocess_mode"] =
        std::max(0, std::min(2, tracking_preprocess_mode_override));
  }
  if (tracking_point_selection_mode_override >= 0) {
    config_node["tracking_point_selection_mode"] =
        std::max(0, std::min(2, tracking_point_selection_mode_override));
  }
  if (tracking_forward_backward_override >= 0) {
    config_node["tracking_forward_backward_enabled"] =
        (tracking_forward_backward_override != 0);
  }
  if (tracking_forward_backward_max_error_override >= 0.0) {
    config_node["tracking_forward_backward_max_error"] =
        tracking_forward_backward_max_error_override;
  }
  if (directional_minimum_visual_weight_override >= 0.0) {
    config_node["directional_minimum_visual_weight"] =
        directional_minimum_visual_weight_override;
  }
  if (directional_activation_enter_score_override >= 0.0) {
    config_node["directional_activation_enter_score"] =
        directional_activation_enter_score_override;
  }
  if (directional_activation_exit_score_override >= 0.0) {
    config_node["directional_activation_exit_score"] =
        directional_activation_exit_score_override;
  }
  if (directional_activation_enter_frames_override >= 1) {
    config_node["directional_activation_enter_frames"] =
        directional_activation_enter_frames_override;
  }
  if (directional_activation_exit_frames_override >= 1) {
    config_node["directional_activation_exit_frames"] =
        directional_activation_exit_frames_override;
  }
  if (directional_minimum_output_rank_override >= 1) {
    config_node["directional_minimum_output_rank"] =
        directional_minimum_output_rank_override;
  }
  if (directional_minimum_trace_ratio_override >= 0.0) {
    config_node["directional_minimum_information_trace_ratio"] =
        directional_minimum_trace_ratio_override;
  }
  if (directional_safety_trace_zero_override >= 0.0) {
    config_node["directional_safety_confidence_trace_zero"] =
        directional_safety_trace_zero_override;
  }
  if (directional_safety_trace_full_override >= 0.0) {
    config_node["directional_safety_confidence_trace_full"] =
        directional_safety_trace_full_override;
  }
  if (directional_safety_alpha_override >= 0.0) {
    config_node["directional_safety_confidence_alpha"] =
        directional_safety_alpha_override;
  }
  if (directional_safety_fall_alpha_override >= 0.0) {
    config_node["directional_safety_confidence_fall_alpha"] =
        directional_safety_fall_alpha_override;
  }
  if (gaussian_feedback_shadow_override >= 0) {
    config_node["gaussian_feedback_shadow_enabled"] =
        (gaussian_feedback_shadow_override != 0);
  }
  if (gaussian_feedback_shadow_maximum_staleness_ms_override >= 0.0) {
    config_node["gaussian_feedback_shadow_maximum_staleness_ms"] =
        gaussian_feedback_shadow_maximum_staleness_ms_override;
  }
  if (gaussian_feedback_shadow_linger_seconds_override >= 0.0) {
    config_node["gaussian_feedback_shadow_linger_seconds"] =
        gaussian_feedback_shadow_linger_seconds_override;
  }
  if (!gaussian_feedback_shadow_log_path_override.empty()) {
    config_node["gaussian_feedback_shadow_log_path"] =
        gaussian_feedback_shadow_log_path_override;
  }
  if (gaussian_feedback_transport_shadow_override >= 0) {
    config_node["gaussian_feedback_transport_shadow_enabled"] =
        gaussian_feedback_transport_shadow_override != 0;
  }
  if (gaussian_feedback_transport_maximum_source_age_ms_override >= 0.0) {
    config_node["gaussian_feedback_transport_maximum_source_age_ms"] =
        gaussian_feedback_transport_maximum_source_age_ms_override;
  }
  if (gaussian_feedback_transport_state_ttl_ms_override >= 0.0) {
    config_node["gaussian_feedback_transport_state_ttl_ms"] =
        gaussian_feedback_transport_state_ttl_ms_override;
  }
  if (gaussian_feedback_transport_state_alpha_override >= 0.0) {
    config_node["gaussian_feedback_transport_state_alpha"] =
        gaussian_feedback_transport_state_alpha_override;
  }
  if (gaussian_endpoint_prior_shadow_override >= 0) {
    config_node["gaussian_endpoint_prior_shadow_enabled"] =
        gaussian_endpoint_prior_shadow_override != 0;
  }
  if (gaussian_endpoint_prior_allow_validation_override >= 0) {
    config_node["gaussian_endpoint_prior_allow_validation"] =
        gaussian_endpoint_prior_allow_validation_override != 0;
  }
  if (gaussian_endpoint_prior_maximum_source_age_ms_override >= 0.0) {
    config_node["gaussian_endpoint_prior_maximum_source_age_ms"] =
        gaussian_endpoint_prior_maximum_source_age_ms_override;
  }
  if (gaussian_endpoint_prior_minimum_confidence_override >= 0.0) {
    config_node["gaussian_endpoint_prior_minimum_confidence"] =
        gaussian_endpoint_prior_minimum_confidence_override;
  }
  if (gaussian_endpoint_prior_minimum_consecutive_accepted_override >= 0) {
    config_node["gaussian_endpoint_prior_minimum_consecutive_accepted"] =
        gaussian_endpoint_prior_minimum_consecutive_accepted_override;
  }
  if (gaussian_endpoint_prior_gain_override >= 0.0) {
    config_node["gaussian_endpoint_prior_gain"] =
        gaussian_endpoint_prior_gain_override;
  }
  if (gaussian_endpoint_factor_override >= 0) {
    config_node["gaussian_endpoint_factor_enabled"] =
        gaussian_endpoint_factor_override != 0;
  }
  if (gaussian_endpoint_factor_apply_override >= 0) {
    config_node["gaussian_endpoint_factor_apply"] =
        gaussian_endpoint_factor_apply_override != 0;
  }
  if (gaussian_endpoint_factor_validation_only_override >= 0) {
    config_node["gaussian_endpoint_factor_validation_only"] =
        gaussian_endpoint_factor_validation_only_override != 0;
  }
  if (gaussian_endpoint_factor_maximum_application_age_ms_override >= 0.0) {
    config_node["gaussian_endpoint_factor_maximum_application_age_ms"] =
        gaussian_endpoint_factor_maximum_application_age_ms_override;
  }
  if (gaussian_endpoint_factor_rotation_sqrt_information_override >= 0.0) {
    config_node["gaussian_endpoint_factor_rotation_sqrt_information"] =
        gaussian_endpoint_factor_rotation_sqrt_information_override;
  }
  if (gaussian_endpoint_factor_translation_sqrt_information_override >= 0.0) {
    config_node["gaussian_endpoint_factor_translation_sqrt_information"] =
        gaussian_endpoint_factor_translation_sqrt_information_override;
  }
  if (!gaussian_endpoint_factor_log_path_override.empty()) {
    config_node["gaussian_endpoint_factor_log_path"] =
        gaussian_endpoint_factor_log_path_override;
  }
  if (gaussian_endpoint_replay_override >= 0) {
    config_node["gaussian_endpoint_replay_enabled"] =
        gaussian_endpoint_replay_override != 0;
  }
  if (!gaussian_endpoint_replay_path_override.empty()) {
    config_node["gaussian_endpoint_replay_path"] =
        gaussian_endpoint_replay_path_override;
  }
  if (!gaussian_endpoint_replay_log_path_override.empty()) {
    config_node["gaussian_endpoint_replay_log_path"] =
        gaussian_endpoint_replay_log_path_override;
  }
  if (gaussian_endpoint_trust_region_override >= 0) {
    config_node["gaussian_endpoint_trust_region_enabled"] =
        gaussian_endpoint_trust_region_override != 0;
  }
  if (gaussian_endpoint_trust_region_apply_override >= 0) {
    config_node["gaussian_endpoint_trust_region_apply"] =
        gaussian_endpoint_trust_region_apply_override != 0;
  }
  if (!gaussian_endpoint_trust_region_log_path_override.empty()) {
    config_node["gaussian_endpoint_trust_region_log_path"] =
        gaussian_endpoint_trust_region_log_path_override;
  }
  if (gaussian_mapping_pose_overlay_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_enabled"] =
        gaussian_mapping_pose_overlay_override != 0;
  }
  if (gaussian_mapping_pose_overlay_recovery_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_recovery_enabled"] =
        gaussian_mapping_pose_overlay_recovery_override != 0;
  }
  if (gaussian_mapping_pose_overlay_validation_corruption_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_validation_corruption_enabled"] =
        gaussian_mapping_pose_overlay_validation_corruption_override != 0;
  }
  if (gaussian_mapping_pose_overlay_source_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_source"] =
        gaussian_mapping_pose_overlay_source_override;
  }
  if (gaussian_mapping_pose_overlay_candidate_gain_override >= 0.0) {
    config_node["gaussian_mapping_pose_overlay_candidate_gain"] =
        gaussian_mapping_pose_overlay_candidate_gain_override;
  }
  if (gaussian_mapping_pose_overlay_candidate_rotation_gain_ratio_override >=
      0.0) {
    config_node["gaussian_mapping_pose_overlay_candidate_rotation_gain_ratio"] =
        gaussian_mapping_pose_overlay_candidate_rotation_gain_ratio_override;
  }
  if (gaussian_mapping_pose_overlay_candidate_translation_cap_metres_override >=
      0.0) {
    config_node["gaussian_mapping_pose_overlay_candidate_translation_cap_metres"] =
        gaussian_mapping_pose_overlay_candidate_translation_cap_metres_override;
  }
  if (gaussian_mapping_pose_overlay_candidate_minimum_confidence_override >=
      0.0) {
    config_node["gaussian_mapping_pose_overlay_candidate_minimum_confidence"] =
        gaussian_mapping_pose_overlay_candidate_minimum_confidence_override;
  }
  if (gaussian_mapping_pose_overlay_application_mask_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_application_mask"] =
        gaussian_mapping_pose_overlay_application_mask_override;
  }
  if (gaussian_mapping_pose_overlay_independent_application_masks_override >=
      0) {
    config_node[
        "gaussian_mapping_pose_overlay_independent_application_masks"] =
        gaussian_mapping_pose_overlay_independent_application_masks_override !=
        0;
  }
  if (gaussian_mapping_pose_overlay_recovery_application_mask_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_recovery_application_mask"] =
        gaussian_mapping_pose_overlay_recovery_application_mask_override;
  }
  if (gaussian_mapping_pose_overlay_corruption_application_mask_override >= 0) {
    config_node["gaussian_mapping_pose_overlay_corruption_application_mask"] =
        gaussian_mapping_pose_overlay_corruption_application_mask_override;
  }
  if (gaussian_mapping_pose_overlay_state_timeout_ms_override >= 0.0) {
    config_node["gaussian_mapping_pose_overlay_state_timeout_ms"] =
        gaussian_mapping_pose_overlay_state_timeout_ms_override;
  }
  if (gaussian_mapping_pose_overlay_maximum_translation_metres_override >=
      0.0) {
    config_node["gaussian_mapping_pose_overlay_maximum_translation_metres"] =
        gaussian_mapping_pose_overlay_maximum_translation_metres_override;
  }
  if (gaussian_mapping_pose_overlay_maximum_rotation_degrees_override >= 0.0) {
    config_node["gaussian_mapping_pose_overlay_maximum_rotation_degrees"] =
        gaussian_mapping_pose_overlay_maximum_rotation_degrees_override;
  }
  if (gaussian_mapping_pose_overlay_maximum_corruption_translation_metres_override >=
      0.0) {
    config_node[
        "gaussian_mapping_pose_overlay_maximum_corruption_translation_metres"] =
        gaussian_mapping_pose_overlay_maximum_corruption_translation_metres_override;
  }
  if (gaussian_mapping_pose_overlay_maximum_corruption_rotation_degrees_override >=
      0.0) {
    config_node[
        "gaussian_mapping_pose_overlay_maximum_corruption_rotation_degrees"] =
        gaussian_mapping_pose_overlay_maximum_corruption_rotation_degrees_override;
  }
  if (!gaussian_mapping_pose_overlay_path_override.empty()) {
    config_node["gaussian_mapping_pose_overlay_path"] =
        gaussian_mapping_pose_overlay_path_override;
  }
  if (!gaussian_mapping_pose_overlay_log_path_override.empty()) {
    config_node["gaussian_mapping_pose_overlay_log_path"] =
        gaussian_mapping_pose_overlay_log_path_override;
  }
  if (!diagnostics_path_override.empty()) {
    config_node["diagnostics_path"] = diagnostics_path_override;
  }
  config_node["observability_length_scale"] = observability_length_scale;
  config_node["deterministic_map_update"] = deterministic_map_update;
  config_node["deterministic_solver_ordering"] =
      deterministic_solver_ordering;
  config_node["deterministic_seed"] = deterministic_seed;

  if (deterministic_seed >= 0) {
    cv::setRNGSeed(deterministic_seed);
    std::srand(deterministic_seed);
  }
  if (deterministic_single_thread) {
    cv::setNumThreads(1);
#ifdef _OPENMP
    omp_set_dynamic(0);
    omp_set_num_threads(1);
#endif
  }
  ROS_INFO("Determinism controls: seed=%d, single_thread=%s, map_update=%s, solver_ordering=%s, cv_threads=%d.",
           deterministic_seed,
           deterministic_single_thread ? "true" : "false",
           deterministic_map_update ? "background-disabled" : "background",
           deterministic_solver_ordering ? "residual-order" : "automatic",
           cv::getNumThreads());

  // std::string log_path = config_node["log_path"].as<std::string>();
  // FLAGS_log_dir = log_path;
  // FLAGS_colorlogtostderr = true;
  std::cout << "\n🥥 Start DA-LIVGS Odometry 🥥";

  OdometryManager odom_manager(config_node, nh);
  odom_manager.RunBag();

  double t_traj_max = odom_manager.SaveOdometry();
  if (t_traj_max <= 0.0) {
    ROS_ERROR("DA-LIVGS finished without a valid initialized trajectory.");
    return 3;
  }
  std::cout << "\n✨ All Done.\n\n";

  return 0;
}
