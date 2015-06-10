/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 */

#ifndef PCL_FEATURES_SHOT_GRF_H_
#define PCL_FEATURES_SHOT_GRF_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/shot_lrf.h>

namespace pcl
{
  /** \brief SHOTGlobalReferenceFrameEstimation estimates the Global Reference Frame used in the calculation
    * of the (GSHOT) descriptor.
    *
    * \author Carlos M. Mateo
    * \ingroup features
    */
  template<typename PointInT, typename PointOutT = ReferenceFrame>
  class SHOTGlobalReferenceFrameEstimation : public SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>
  {
    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
    typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
    typedef typename SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      typedef boost::shared_ptr<SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT> > ConstPtr;

      /** \brief Empty constructor */
      SHOTGlobalReferenceFrameEstimation ()
      {
        feature_name_ = "SHOTGlobalReferenceFrameEstimation";
      }

      /** \brief Empty destructor */
      virtual ~SHOTGlobalReferenceFrameEstimation ()
      {}

      virtual Eigen::Vector4f
      getCentralPoint ()
      {
        return central_point_;
      }
//
//      void
//      getRFCenterAndRadius (const PointCloudInConstPtr& input,
//                            const IndicesPtr& indices,
//                            const PointCloudInConstPtr& surface,
//                            int& rf_center,
//                            float& rf_radius);

    protected:
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::feature_name_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::input_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::indices_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::surface_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::tree_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::search_parameter_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::search_radius_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::fake_surface_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::fake_indices_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::initCompute;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::deinitCompute;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getClassName;
      
      /** \brief Computes disambiguated local RF for a point index
        * \param[in] index the index
        * \param[out] rf reference frame to compute
        */
      float
      getLocalRF (const Eigen::Vector4f& central_point, Eigen::Matrix3f &rf);

      /** \brief Computes disambiguated Global RF for the center point
        * \param[out] rf reference frame to compute
        */
      float
      getGlobalRF (Eigen::Matrix3f& rf)
      {
        return SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF ((*indices_)[0], rf);
      }

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

      /** \brief Feature estimation method.
        * \param[out] output the resultant features
        */
      virtual void
      computeFeature (PointCloudOut& output);

      Eigen::Vector4f central_point_;

    private:
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/shot_grf.hpp>
#endif

#endif /* PCL_FEATURES_SHOT_GRF_H_ */
