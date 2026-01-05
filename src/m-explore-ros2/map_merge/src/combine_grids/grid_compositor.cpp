/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <combine_grids/grid_compositor.h>

#include <opencv2/stitching/detail/util.hpp>

#include <rcpputils/asserts.hpp>


namespace combine_grids
{
namespace internal
{
nav_msgs::msg::OccupancyGrid::SharedPtr GridCompositor::compose(
    const std::vector<cv::Mat>& grids, const std::vector<cv::Rect>& rois)
{
  rcpputils::require_true(grids.size() == rois.size());

  nav_msgs::msg::OccupancyGrid::SharedPtr result_grid(new nav_msgs::msg::OccupancyGrid());

  std::vector<cv::Point> corners;
  corners.reserve(grids.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(grids.size());
  for (auto& roi : rois) {
    corners.push_back(roi.tl());
    sizes.push_back(roi.size());
  }
  cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

  result_grid->info.width = static_cast<uint>(dst_roi.width);
  result_grid->info.height = static_cast<uint>(dst_roi.height);
  result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
  // create view for opencv pointing to newly allocated grid
  cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

  // Si au moins un robot voit un mur (100), on garde le mur
  // Sinon, si au moins un robot voit libre (0), on garde libre
  // Sinon, inconnu (-1)
  for (int y = 0; y < result.rows; ++y) {
    for (int x = 0; x < result.cols; ++x) {
      bool any_wall = false;
      bool any_free = false;
      bool any_known = false;
      // Pour chaque carte source
      for (size_t i = 0; i < grids.size(); ++i) {
        int src_x = x - (corners[i].x - dst_roi.x);
        int src_y = y - (corners[i].y - dst_roi.y);
        if (src_x >= 0 && src_x < grids[i].cols && src_y >= 0 && src_y < grids[i].rows) {
          int8_t v = grids[i].at<int8_t>(src_y, src_x);
          if (v == 100) any_wall = true;
          if (v == 0) any_free = true;
          if (v != -1) any_known = true;
        }
      }
      int8_t fusion = -1;
      if (any_wall) fusion = 100;
      else if (any_free) fusion = 0;
      // sinon -1 (inconnu)
      result.at<int8_t>(y, x) = fusion;
    }
  }

  return result_grid;
}

}  // namespace internal

}  // namespace combine_grids

 
