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

#ifndef PCL_GSHOT_H_
#define PCL_GSHOT_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/shot.h>

namespace pcl
{
  /** \brief GSHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
   * a given point cloud dataset containing points and normals using a global reference frames.
   *
   * The suggested PointOutT is pcl::SHOT352.
   *
   * \author Carlos M. Mateo
   * \ingroup features
   */
  template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT = pcl::ReferenceFrame>
  class GSHOTEstimationBase : public SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>
  {
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Feature<PointInT, PointRFT>::Ptr GRFEstimationPtr;
      typedef typename FeatureWithLocalReferenceFrames<PointInT, PointRFT>::PointCloudLRF PointCloudGRF;

    public:
      typedef boost::shared_ptr<GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> > Ptr;
      typedef boost::shared_ptr<const GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> > ConstPtr;

    protected:
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::feature_name_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::input_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::fake_indices_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::indices_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::k_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_parameter_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_radius_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::surface_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::fake_surface_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::tree_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::frames_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::frames_never_defined_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::getClassName;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::deinitCompute;

      /** \brief Empty constructor */
      GSHOTEstimationBase (int nr_shape_bins = 10)
      : SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> (nr_shape_bins)
      {
        feature_name_ = "GSHOTEstimationBase";
      }

    public:
      /** \brief Empty destructor */
      virtual
      ~GSHOTEstimationBase ()
      {
      }

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
       * \param[in] central_point the point where it is fixed the reference frame
       * \param[in] indices the k-neighborhood point indices in surface_
       * \param[in] sqr_dists the k-neighborhood point distances in surface_
       * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
       */
      virtual void
      computePointSHOT (const Eigen::Vector4f& central_point,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot) = 0;

      virtual Eigen::Vector4f
      getCentralPoint ()
      {
        return central_point_;
      }

    protected:

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

      /** \brief Quadrilinear interpolation used when color and shape descriptions are NOT activated simultaneously
       * \param[in] indices the neighborhood point indices
       * \param[in] sqr_dists the neighborhood point distances
       * \param[in] central_point the central point of the signature
       * \param[out] binDistance the resultant distance shape histogram
       * \param[in] nr_bins the number of bins in the shape histogram
       * \param[out] shot the resultant SHOT histogram
       */
      void
      interpolateSingleChannel (const std::vector<int> &indices,
                                const std::vector<float> &sqr_dists,
                                const Eigen::Vector4f& central_point,
                                std::vector<double> &binDistance,
                                const int nr_bins,
                                Eigen::VectorXf &shot);

      /** \brief Check if frames_ has been correctly initialized and compute it if needed.
       * \param lrf_estimation a pointer to a local reference frame estimation class to be used as default.
       * \return true if frames_ has been correctly initialized.
       */
      inline bool
      initGlobalReferenceFrame (const GRFEstimationPtr& grf_estimation = GRFEstimationPtr ())
      {
        return initLocalReferenceFrames (0, grf_estimation);
      }

      /* Virtual point center of the cloud */
      Eigen::Vector4f central_point_;

    private:
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::initLocalReferenceFrames;

      inline void
      computePointSHOT (const int /*index*/,
                        const std::vector<int> &/*indices*/,
                        const std::vector<float> &/*sqr_dists*/,
                        Eigen::VectorXf &/*shot*/)
      {
        PCL_ERROR("[pcl::%s::computePointSHOT] Error! Call computePointSHOT(indices, sqr_dists, shot) to use this class.\n", getClassName ().c_str ());
      }
//
  };

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
   * a given point cloud dataset containing points and normals using a global reference frames.
   *
   * The suggested PointOutT is pcl::SHOT352
   *
   * \author Carlos M. Mateo
   * \ingroup features
   */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
  class GSHOTEstimation : public GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>
  {
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

    public:
      typedef boost::shared_ptr<GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT> > Ptr;
      typedef boost::shared_ptr<const GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT> > ConstPtr;

      /** \brief Empty constructor. */
      GSHOTEstimation (int nr_shape_bins = 10)
      : GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> (nr_shape_bins)
      {
        feature_name_ = "GSHOTEstimation";
      }

      /** \brief Empty destructor */
      virtual
      ~GSHOTEstimation ()
      {
      }

    protected:
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::descLength_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_grid_sector_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_shape_bins_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::sqradius_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius3_4_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_4_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_2_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::shot_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::feature_name_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::indices_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::k_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_parameter_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_radius_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::input_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::tree_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::frames_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::getClassName;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::createBinDistanceShape;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::normalizeHistogram;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::central_point_;

      /** \brief Estimate the GSHOT descriptor for the ceneter point based on its spatial neighborhood of 3D points with normals
       * \param[in] central_point the point where it is fixed the reference frame
       * \param[in] indices the k-neighborhood point indices in surface_
       * \param[in] sqr_dists the k-neighborhood point distances in surface_
       * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
       */
      virtual void
      computePointSHOT (const Eigen::Vector4f& central_point,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot);

      /** \brief Estimate the Global Signatures of Histograms of OrienTations (GSHOT) descriptors at a set of points given by
       * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
       * setSearchMethod ()
       * \param output the resultant point cloud model dataset that contains the GSHOT feature estimates
       */
      void
      computeFeature (pcl::PointCloud<PointOutT> &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/gshot.hpp>
#endif

#endif  //#ifndef PCL_GSHOT_H_
