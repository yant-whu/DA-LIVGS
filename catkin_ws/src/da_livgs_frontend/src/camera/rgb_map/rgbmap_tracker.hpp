/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "pointcloud_rgbd.hpp"
#include "image_frame.hpp"
#include "tracking_point_selector.h"
#include "forward_backward_klt.h"
#include "opencv2/calib3d.hpp"
#include "../optical_flow/lkpyramid.hpp"
#include <utils/log_utils.h>
#include <algorithm>
extern std::string data_dump_dir;

using RgbObservation = std::pair<void *, cv::Point2f>;

struct RgbPointIndexLess
{
    bool operator()(const void *left, const void *right) const
    {
        const auto *left_point = static_cast<const RGB_pts *>(left);
        const auto *right_point = static_cast<const RGB_pts *>(right);
        if (left_point->m_pt_index != right_point->m_pt_index)
            return left_point->m_pt_index < right_point->m_pt_index;
        return std::less<const void *>()(left, right);
    }
};

inline bool stable_rgb_observation_less(const RgbObservation &left,
                                        const RgbObservation &right)
{
    RgbPointIndexLess point_less;
    if (point_less(left.first, right.first))
        return true;
    if (point_less(right.first, left.first))
        return false;
    if (left.second.x != right.second.x)
        return left.second.x < right.second.x;
    return left.second.y < right.second.y;
}

class Rgbmap_tracker
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector< cv::Mat >     m_img_vec;
    char                       m_temp_char[ 1024 ];
    cv::Mat                    m_old_frame, m_old_gray;
    cv::Mat                    frame_gray, m_current_frame;
    cv::Mat                    m_current_mask;
    unsigned int               m_frame_idx = 0;
    double                     m_last_frame_time, m_current_frame_time;
    std::vector< int >         m_current_ids, m_old_ids;
    int                        if_debug_match_img = 0;
    unsigned int               m_maximum_vio_tracked_pts = 300;
    cv::Mat                    m_ud_map1, m_ud_map2;
    cv::Mat                    m_intrinsic, m_dist_coeffs;
    std::vector< cv::Point2f > m_last_tracked_pts, m_current_tracked_pts;
    std::vector< cv::Point2f > m_last_tracked_pts_tmp;
    std::vector< cv::Point2f > m_current_tracked_pts_tmp;
    std::vector< cv::Scalar >  m_colors;
    std::vector< void * >      m_rgb_pts_ptr_vec_in_last_frame;
    std::map< void *, cv::Point2f, RgbPointIndexLess > m_map_rgb_pts_in_last_frame_pos;
    std::map< void *, cv::Point2f, RgbPointIndexLess > m_map_rgb_pts_in_current_frame_pos;

    std::map< int, std::vector< cv::Point2f > > m_map_id_pts_vec;
    std::map< int, std::vector< int > >         m_map_id_pts_frame;
    std::map< int, std::vector< cv::Point2f > > m_map_frame_pts;
    cv::Mat                                   m_debug_track_img;
    eigen_q                                   q_last_estimated_q = eigen_q::Identity();
    vec_3                                     t_last_estimated = vec_3( 0, 0, 0 );
    std::shared_ptr< LK_optical_flow_kernel > m_lk_optical_flow_kernel;
    da_livgs_frontend::TrackingPointSelector tracking_point_selector;
    da_livgs_frontend::TrackingPointSelectionDiagnostics tracking_selection_diagnostics;
    da_livgs_frontend::ForwardBackwardKltConfig forward_backward_config;
    da_livgs_frontend::ForwardBackwardKltDiagnostics forward_backward_diagnostics;
    Rgbmap_tracker();
    ~Rgbmap_tracker(){};

    cv::Mat last_img;
    Eigen::aligned_vector<Eigen::Vector3d> new_points;
    Eigen::aligned_vector<Eigen::Vector2d> new_pixs;

    void set_intrinsic( Eigen::Matrix3d cam_K, Eigen::Matrix< double, 5, 1 > dist, cv::Size imageSize )
    {
        cv::eigen2cv( cam_K, m_intrinsic );
        cv::eigen2cv( dist, m_dist_coeffs );
        initUndistortRectifyMap( m_intrinsic, m_dist_coeffs, cv::Mat(), m_intrinsic, imageSize, CV_16SC2, m_ud_map1, m_ud_map2 );
    }

    void update_last_tracking_vector_and_ids()
    {
        int idx = 0;
        m_last_tracked_pts.clear();
        m_rgb_pts_ptr_vec_in_last_frame.clear();
        m_colors.clear();
        m_old_ids.clear();
        std::vector<RgbObservation> observations(
            m_map_rgb_pts_in_last_frame_pos.begin(),
            m_map_rgb_pts_in_last_frame_pos.end());
        std::sort(observations.begin(), observations.end(),
                  stable_rgb_observation_less);
        for (const auto &observation : observations)
        {
            m_rgb_pts_ptr_vec_in_last_frame.push_back(observation.first);
            m_last_tracked_pts.push_back(observation.second);
            m_colors.push_back(static_cast<RGB_pts *>(observation.first)->m_dbg_color);
            m_old_ids.push_back( idx );
            idx++;
        }
    }

    void set_track_pts( cv::Mat &img, std::vector< std::shared_ptr< RGB_pts > > &rgb_pts_vec, std::vector< cv::Point2f > &pts_proj_img_vec )
    {
        m_old_frame = img.clone();
        cv::cvtColor( m_old_frame, m_old_gray, cv::COLOR_BGR2GRAY );
        m_map_rgb_pts_in_last_frame_pos.clear();
        for ( unsigned int i = 0; i < rgb_pts_vec.size(); i++ )
        {
            m_map_rgb_pts_in_last_frame_pos[ ( void * ) rgb_pts_vec[ i ].get() ] = pts_proj_img_vec[ i ];  //
        }
        update_last_tracking_vector_and_ids();
    }

    bool init( const std::shared_ptr< Image_frame > &img_with_pose, std::vector< std::shared_ptr< RGB_pts > > &rgb_pts_vec, std::vector< cv::Point2f > &pts_proj_img_vec )
    {
        if (tracking_point_selector.Config().mode ==
            da_livgs_frontend::TrackingPointSelectionMode::Disabled)
        {
            set_track_pts( img_with_pose->m_img, rgb_pts_vec, pts_proj_img_vec );
        }
        else
        {
            std::vector<da_livgs_frontend::TrackingPointCandidate> candidates;
            const size_t candidate_count = std::min(
                rgb_pts_vec.size(), pts_proj_img_vec.size());
            candidates.reserve(candidate_count);
            for (size_t index = 0; index < candidate_count; ++index)
                candidates.push_back({index, pts_proj_img_vec[index]});
            const auto selection = tracking_point_selector.Select(
                img_with_pose->m_img_gray, candidates, {},
                m_maximum_vio_tracked_pts,
                std::min<size_t>(
                    tracking_point_selector.Config().minimum_tracked_points,
                    m_maximum_vio_tracked_pts),
                1.0);
            tracking_selection_diagnostics = selection.diagnostics;
            std::vector<std::shared_ptr<RGB_pts>> selected_points;
            std::vector<cv::Point2f> selected_pixels;
            selected_points.reserve(selection.source_indices.size());
            selected_pixels.reserve(selection.source_indices.size());
            for (size_t index : selection.source_indices)
            {
                selected_points.push_back(rgb_pts_vec[index]);
                selected_pixels.push_back(pts_proj_img_vec[index]);
            }
            set_track_pts(img_with_pose->m_img, selected_points,
                          selected_pixels);
        }
        m_current_frame_time = img_with_pose->m_timestamp;
        m_last_frame_time = m_current_frame_time;
        std::vector< uchar > status;
        m_lk_optical_flow_kernel->track_image( img_with_pose->m_img_gray, m_last_tracked_pts, m_current_tracked_pts, status );

        new_points.clear();
        new_pixs.clear();
        return m_map_rgb_pts_in_last_frame_pos.size() >= 10;
    }

    void update_points( std::vector< cv::Point2f > &pts_vec, std::vector< int > &pts_ids )
    {
        for ( unsigned int i = 0; i < pts_vec.size(); i++ )
        {
            m_map_id_pts_vec[ pts_ids[ i ] ].push_back( pts_vec[ i ] );
            m_map_id_pts_frame[ pts_ids[ i ] ].push_back( m_frame_idx );
            m_map_frame_pts[ m_frame_idx ].push_back( pts_vec[ i ] );
        }
    }

    void undistort_image( cv::Mat &img )
    {
        cv::Mat temp_img;
        temp_img = img.clone();
        remap( temp_img, img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR );
    }

    void update_and_append_track_pts( std::shared_ptr< Image_frame > &img_pose, Global_map &map_rgb, double mini_dis = 10.0, int minimum_frame_diff = 3e8 );
    void reject_error_tracking_pts( std::shared_ptr< Image_frame > &img_pose, double dis = 2.0 );

    template < typename T > void reduce_vector( std::vector< T > &v, std::vector< uchar > status )
    {
        int j = 0;
        for ( unsigned int i = 0; i < v.size(); i++ )
            if ( status[ i ] )
                v[ j++ ] = v[ i ];
        v.resize( j );
    }

    void track_img( std::shared_ptr< Image_frame > &img_pose, double dis = 2.0, int if_use_opencv = 1 );
    int get_all_tracked_pts( std::vector< std::vector< cv::Point2f > > *img_pt_vec = nullptr );
    int remove_outlier_using_ransac_pnp( std::shared_ptr< Image_frame > &img_pose, int if_remove_ourlier = 1 );

    int input_before_track = 0;
    int inlier_aft_track = 0;
    int inlier_aft_fmat = 0;
    int inlier_aft_pnp = 0;
    int deterministic_seed = -1;
};
