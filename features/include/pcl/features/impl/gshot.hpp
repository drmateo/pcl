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

// Useful constants.
#define PST_PI 3.1415926535897932384626433832795
#define PST_RAD_45 0.78539816339744830961566084581988
#define PST_RAD_90 1.5707963267948966192313216916398
#define PST_RAD_135 2.3561944901923449288469825374596
#define PST_RAD_180 PST_PI
#define PST_RAD_360 6.283185307179586476925286766558
#define PST_RAD_PI_7_8 2.7488935718910690836548129603691

const double zeroDoubleEps15 = 1E-15;
const float zeroFloatEps8 = 1E-8f;

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Check if val1 and val2 are equals.
  *
  * \param[in] val1 first number to check.
  * \param[in] val2 second number to check.
  * \param[in] zeroDoubleEps epsilon
  * \return true if val1 is equal to val2, false otherwise.
  */
inline bool
areEquals (double val1, double val2, double zeroDoubleEps = zeroDoubleEps15)
{
  return (fabs (val1 - val2)<zeroDoubleEps);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Check if val1 and val2 are equals.
  *
  * \param[in] val1 first number to check.
  * \param[in] val2 second number to check.
  * \param[in] zeroFloatEps epsilon
  * \return true if val1 is equal to val2, false otherwise.
  */
inline bool
areEquals (float val1, float val2, float zeroFloatEps = zeroFloatEps8)
{
  return (fabs (val1 - val2)<zeroFloatEps);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> bool
pcl::GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return false;
  }
  
  // Global RF cannot work with k-search or radius-search specific
  if (this->getKSearch () != 0)
  {
    PCL_ERROR("[pcl::%s::initCompute] Error! Search method set to k-neighborhood. Call setKSearch(0) to use this class.\n", getClassName().c_str ());
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
  
  if (tree_->getInputCloud () != surface_) // Make sure the tree searches the surface
    tree_->setInputCloud (surface_);
  
  // Default GRF estimation alg: SHOTGlobalReferenceFrameEstimation
  typename SHOTGlobalReferenceFrameEstimation<PointInT, PointRFT>::Ptr grf_estimator(new SHOTGlobalReferenceFrameEstimation<PointInT, PointRFT>());
  grf_estimator->setRadiusSearch (search_radius_);
  grf_estimator->setInputCloud (input_);
  grf_estimator->setIndices (indices_);
  grf_estimator->setSearchMethod (tree_);
  if (!fake_surface_)
    grf_estimator->setSearchSurface(surface_);

  if (!initGlobalReferenceFrame (grf_estimator))
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
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
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel (const std::vector<int> &indices,
                                                                                            const std::vector<float> &sqr_dists,
                                                                                            const Eigen::Vector4f& central_point,
                                                                                            const int& index,
                                                                                            std::vector<double> &binDistance,
                                                                                            const int nr_bins,
                                                                                            Eigen::VectorXf &shot)
{
  //const Eigen::Vector4f& central_point = (*input_)[(*indices_)[index]].getVector4fMap ();
  const PointRFT& current_frame = (*frames_)[index];

  Eigen::Vector4f current_frame_x (current_frame.x_axis[0], current_frame.x_axis[1], current_frame.x_axis[2], 0);
  Eigen::Vector4f current_frame_y (current_frame.y_axis[0], current_frame.y_axis[1], current_frame.y_axis[2], 0);
  Eigen::Vector4f current_frame_z (current_frame.z_axis[0], current_frame.z_axis[1], current_frame.z_axis[2], 0);

  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    if (!pcl_isfinite(binDistance[i_idx]))
      continue;

    Eigen::Vector4f delta = surface_->points[indices[i_idx]].getVector4fMap () - central_point;
    delta[3] = 0;

    // Compute the Euclidean norm
   double distance = sqrt (sqr_dists[i_idx]);

    if (areEquals (distance, 0.0))
      continue;

    double xInFeatRef = delta.dot (current_frame_x);
    double yInFeatRef = delta.dot (current_frame_y);
    double zInFeatRef = delta.dot (current_frame_z);

    // To avoid numerical problems afterwards
    if (fabs (yInFeatRef) < 1E-30)
      yInFeatRef  = 0;
    if (fabs (xInFeatRef) < 1E-30)
      xInFeatRef  = 0;
    if (fabs (zInFeatRef) < 1E-30)
      zInFeatRef  = 0;


    unsigned char bit4 = ((yInFeatRef > 0) || ((yInFeatRef == 0.0) && (xInFeatRef < 0))) ? 1 : 0;
    unsigned char bit3 = static_cast<unsigned char> (((xInFeatRef > 0) || ((xInFeatRef == 0.0) && (yInFeatRef > 0))) ? !bit4 : bit4);

    assert (bit3 == 0 || bit3 == 1);

    int desc_index = (bit4<<3) + (bit3<<2);

    desc_index = desc_index << 1;

    if ((xInFeatRef * yInFeatRef > 0) || (xInFeatRef == 0.0))
      desc_index += (fabs (xInFeatRef) >= fabs (yInFeatRef)) ? 0 : 4;
    else
      desc_index += (fabs (xInFeatRef) > fabs (yInFeatRef)) ? 4 : 0;

    desc_index += zInFeatRef > 0 ? 1 : 0;

    // 2 RADII
    desc_index += (distance > radius1_2_) ? 2 : 0;

    int step_index = static_cast<int>(floor (binDistance[i_idx] +0.5));
    int volume_index = desc_index * (nr_bins+1);

    //Interpolation on the cosine (adjacent bins in the histogram)
    binDistance[i_idx] -= step_index;
    double intWeight = (1- fabs (binDistance[i_idx]));

    if (binDistance[i_idx] > 0)
      shot[volume_index + ((step_index+1) % nr_bins)] += static_cast<float> (binDistance[i_idx]);
    else
      shot[volume_index + ((step_index - 1 + nr_bins) % nr_bins)] += - static_cast<float> (binDistance[i_idx]);

    //Interpolation on the distance (adjacent husks)

    if (distance > radius1_2_)   //external sphere
    {
      double radiusDistance = (distance - radius3_4_) / radius1_2_;

      if (distance > radius3_4_) //most external sector, votes only for itself
        intWeight += 1 - radiusDistance;  //peso=1-d
      else  //3/4 of radius, votes also for the internal sphere
      {
        intWeight += 1 + radiusDistance;
        shot[(desc_index - 2) * (nr_bins+1) + step_index] -= static_cast<float> (radiusDistance);
      }
    }
    else    //internal sphere
    {
      double radiusDistance = (distance - radius1_4_) / radius1_2_;

      if (distance < radius1_4_) //most internal sector, votes only for itself
        intWeight += 1 + radiusDistance;  //weight=1-d
      else  //3/4 of radius, votes also for the external sphere
      {
        intWeight += 1 - radiusDistance;
        shot[(desc_index + 2) * (nr_bins+1) + step_index] += static_cast<float> (radiusDistance);
      }
    }

    //Interpolation on the inclination (adjacent vertical volumes)
    double inclinationCos = zInFeatRef / distance;
    if (inclinationCos < - 1.0)
      inclinationCos = - 1.0;
    if (inclinationCos > 1.0)
      inclinationCos = 1.0;

    double inclination = acos (inclinationCos);

    assert (inclination >= 0.0 && inclination <= PST_RAD_180);

    if (inclination > PST_RAD_90 || (fabs (inclination - PST_RAD_90) < 1e-30 && zInFeatRef <= 0))
    {
      double inclinationDistance = (inclination - PST_RAD_135) / PST_RAD_90;
      if (inclination > PST_RAD_135)
        intWeight += 1 - inclinationDistance;
      else
      {
        intWeight += 1 + inclinationDistance;
        assert ((desc_index + 1) * (nr_bins+1) + step_index >= 0 && (desc_index + 1) * (nr_bins+1) + step_index < descLength_);
        shot[(desc_index + 1) * (nr_bins+1) + step_index] -= static_cast<float> (inclinationDistance);
      }
    }
    else
    {
      double inclinationDistance = (inclination - PST_RAD_45) / PST_RAD_90;
      if (inclination < PST_RAD_45)
        intWeight += 1 + inclinationDistance;
      else
      {
        intWeight += 1 - inclinationDistance;
        assert ((desc_index - 1) * (nr_bins+1) + step_index >= 0 && (desc_index - 1) * (nr_bins+1) + step_index < descLength_);
        shot[(desc_index - 1) * (nr_bins+1) + step_index] += static_cast<float> (inclinationDistance);
      }
    }

    if (yInFeatRef != 0.0 || xInFeatRef != 0.0)
    {
      //Interpolation on the azimuth (adjacent horizontal volumes)
      double azimuth = atan2 (yInFeatRef, xInFeatRef);

      int sel = desc_index >> 2;
      double angularSectorSpan = PST_RAD_45;
      double angularSectorStart = - PST_RAD_PI_7_8;

      double azimuthDistance = (azimuth - (angularSectorStart + angularSectorSpan*sel)) / angularSectorSpan;

      assert ((azimuthDistance < 0.5 || areEquals (azimuthDistance, 0.5)) && (azimuthDistance > - 0.5 || areEquals (azimuthDistance, - 0.5)));

      azimuthDistance = (std::max)(- 0.5, std::min (azimuthDistance, 0.5));

      if (azimuthDistance > 0)
      {
        intWeight += 1 - azimuthDistance;
        int interp_index = (desc_index + 4) % maxAngularSectors_;
        assert (interp_index * (nr_bins+1) + step_index >= 0 && interp_index * (nr_bins+1) + step_index < descLength_);
        shot[interp_index * (nr_bins+1) + step_index] += static_cast<float> (azimuthDistance);
      }
      else
      {
        int interp_index = (desc_index - 4 + maxAngularSectors_) % maxAngularSectors_;
        assert (interp_index * (nr_bins+1) + step_index >= 0 && interp_index * (nr_bins+1) + step_index < descLength_);
        intWeight += 1 + azimuthDistance;
        shot[interp_index * (nr_bins+1) + step_index] -= static_cast<float> (azimuthDistance);
      }

    }

    assert (volume_index + step_index >= 0 &&  volume_index + step_index < descLength_);
    shot[volume_index + step_index] += static_cast<float> (intWeight);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>::computePointSHOT (const Eigen::Vector4f& central_point,
                                                                                const int& index,
                                                                                const std::vector<int> &indices,
                                                                                const std::vector<float> &sqr_dists,
                                                                                Eigen::VectorXf &shot)
{
  //Skip the current feature if the number of its neighbors is not sufficient for its description
  if (indices.size () < 5)
  {
    PCL_WARN ("[pcl::%s::computePointSHOT] Warning! Neighborhood has less than 5 vertexes. Aborting description \n", getClassName ().c_str ());
    shot.setConstant(descLength_, 1, std::numeric_limits<float>::quiet_NaN () );

    return;
  }

   // Clear the resultant shot
  std::vector<double> binDistanceShape;
  this->createBinDistanceShape (0, indices, binDistanceShape);

  // Interpolate
  shot.setZero ();
  interpolateSingleChannel (indices, sqr_dists, central_point, index, binDistanceShape, nr_shape_bins_, shot);

  // Normalize the final histogram
  this->normalizeHistogram (shot, descLength_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT>::computeFeature (pcl::PointCloud<PointOutT> &output)
{
  descLength_ = nr_grid_sector_ * (nr_shape_bins_+1);

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_*3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  assert(descLength_ == 352);

  shot_.setZero (descLength_);

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  output.resize(1);
  output.width = 1;

  bool grf_is_nan = false;
  const PointRFT& current_frame = (*frames_)[0];
  if (!pcl_isfinite (current_frame.x_axis[0]) ||
      !pcl_isfinite (current_frame.y_axis[0]) ||
      !pcl_isfinite (current_frame.z_axis[0]))
  {
    PCL_WARN ("[pcl::%s::computeFeature] The local reference frame is not valid! Aborting description of point with index %d\n", getClassName ().c_str (), (*indices_)[0]);
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
  computePointSHOT (central_point_, 0, nn_indices, nn_dists, shot_);

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
