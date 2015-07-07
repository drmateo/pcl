/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#ifndef PCL_FEATURES_IMPL_GSHOT_H_
#define PCL_FEATURES_IMPL_GSHOT_H_

#include <pcl/features/gshot.h>
#include <pcl/features/shot_grf.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <utility>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel (const std::vector<int> &indices,
                                                                                            const std::vector<float> &sqr_dists,
                                                                                            const Eigen::Vector4f& central_point,
                                                                                            std::vector<double> &binDistance,
                                                                                            const int nr_bins,
                                                                                            Eigen::VectorXf &shot)
{
  const PointRFT& current_frame = (*frames_)[0];

  pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel (indices, sqr_dists, central_point, current_frame, binDistance,
                                                                                             nr_bins, shot);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> bool
pcl::GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR("[pcl::%s::initCompute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return false;
  }

  // Global RF cannot work with k-search or radius-search specific
  if (this->getKSearch () != 0)
  {
    PCL_ERROR("[pcl::%s::initCompute] Error! Search method set to k-neighborhood. Call setKSearch(0) to use this class.\n", getClassName ().c_str ());
    return false;
  }

//  // Check if the radius for compute normals is specified
//  if (this->radius_for_normal_ == 0)
//  {
//    PCL_ERROR ("[pcl::%s::initCompute] Error! Call setRadiusNormal(r) to use this clss.\n", getClassName ().c_str ());
//    return false;
//  }

// Use the input dataset as the search surface itself
  if (!surface_)
  {
    fake_surface_ = true;
    surface_ = input_;
  }

  // Check if a space search locator was given
  if (!tree_)
  {
    if (surface_->isOrganized () && input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointInT> (false));
  }

  if (tree_->getInputCloud () != surface_)  // Make sure the tree searches the surface
    tree_->setInputCloud (surface_);

  // Default GRF estimation alg: SHOTGlobalReferenceFrameEstimation
  typename SHOTGlobalReferenceFrameEstimation<PointInT, PointRFT>::Ptr grf_estimator (new SHOTGlobalReferenceFrameEstimation<PointInT, PointRFT> ());
  grf_estimator->setRadiusSearch (search_radius_);
  grf_estimator->setInputCloud (input_);
  grf_estimator->setIndices (indices_);
  grf_estimator->setSearchMethod (tree_);
  if (!fake_surface_)
    grf_estimator->setSearchSurface (surface_);

  if (!initGlobalReferenceFrame (grf_estimator))
  {
    PCL_ERROR("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return false;
  }

  // Check if the global reference frames is set
  if (frames_never_defined_)
  {
    search_radius_ = grf_estimator->getRadiusSearch ();
    search_parameter_ = search_radius_;

    central_point_ = grf_estimator->getCentralPoint ();
  }
  else
  {
    compute3DCentroid<PointInT, float> (*surface_, central_point_);
    Eigen::Vector4f max_pt;
    getMaxDistance<PointInT> (*surface_, central_point_, max_pt);
    if (search_radius_ == 0)
      search_radius_ = (max_pt - central_point_).norm ();
    search_parameter_ = search_radius_;
  }

  if (!FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ())
  {
    PCL_ERROR("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>::computePointSHOT (const Eigen::Vector4f& central_point,
                                                                                const std::vector<int> &indices,
                                                                                const std::vector<float> &sqr_dists,
                                                                                Eigen::VectorXf &shot)
{
  //Skip the current feature if the number of its neighbors is not sufficient for its description
  if (indices.size () < 5)
  {
    PCL_WARN("[pcl::%s::computePointSHOT] Warning! Neighborhood has less than 5 vertexes. Aborting description \n", getClassName ().c_str ());
    shot.setConstant (descLength_, 1, std::numeric_limits<float>::quiet_NaN ());

    return;
  }

  // Clear the resultant shot
  std::vector<double> binDistanceShape;
  this->createBinDistanceShape (0, indices, binDistanceShape);

// Interpolate
  shot.setZero ();
  interpolateSingleChannel (indices, sqr_dists, central_point, binDistanceShape, nr_shape_bins_, shot);

// Normalize the final histogram
  this->normalizeHistogram (shot, descLength_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>::computeFeature (pcl::PointCloud<PointOutT> &output)
{
  descLength_ = nr_grid_sector_ * (nr_shape_bins_ + 1);

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_ * 3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  assert(descLength_ == 352);

  shot_.setZero (descLength_);

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  output.resize (1);
  output.width = 1;

  bool grf_is_nan = false;
  const PointRFT& current_frame = (*frames_)[0];
  if (!pcl_isfinite(current_frame.x_axis[0]) || !pcl_isfinite(current_frame.y_axis[0]) || !pcl_isfinite(current_frame.z_axis[0]))
  {
    PCL_WARN("[pcl::%s::computeFeature] The local reference frame is not valid! Aborting description of point with index %d\n", getClassName ().c_str (),
             (*indices_)[0]);
    grf_is_nan = true;
  }

  //if (!isFinite ((*input_)[(*indices_)[0]]) || grf_is_nan || this->searchForNeighbors ((*indices_)[0], search_parameter_, nn_indices, nn_dists) == 0)

  PointInT central_pt;
  central_pt.getVector4fMap () = central_point_;
  if (!isFinite (central_pt) || grf_is_nan || tree_->radiusSearch (central_pt, search_radius_, nn_indices, nn_dists) == 0)
  {
// Copy into the resultant cloud
    for (int d = 0; d < descLength_; ++d)
      output.points[0].descriptor[d] = std::numeric_limits<float>::quiet_NaN ();
    for (int d = 0; d < 9; ++d)
      output.points[0].rf[d] = std::numeric_limits<float>::quiet_NaN ();

    output.is_dense = false;
  }

  // Estimate the SHOT descriptor at each patch
  computePointSHOT (central_point_, nn_indices, nn_dists, shot_);

  // Copy into the resultant cloud
  for (int d = 0; d < descLength_; ++d)
    output.points[0].descriptor[d] = shot_[d];
  for (int d = 0; d < 3; ++d)
  {
    output.points[0].rf[d + 0] = frames_->points[0].x_axis[d];
    output.points[0].rf[d + 3] = frames_->points[0].y_axis[d];
    output.points[0].rf[d + 6] = frames_->points[0].z_axis[d];
  }
}

#define PCL_INSTANTIATE_GSHOTEstimationBase(T,NT,OutT,RFT) template class PCL_EXPORTS pcl::GSHOTEstimationBase<T,NT,OutT,RFT>;
#define PCL_INSTANTIATE_GSHOTEstimation(T,NT,OutT,RFT) template class PCL_EXPORTS pcl::GSHOTEstimation<T,NT,OutT,RFT>;

#endif    // PCL_FEATURES_IMPL_GSHOT_H_
