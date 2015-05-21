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
  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * The suggested PointOutT is pcl::SHOT352.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Carlos M. Mateo, Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT = pcl::ReferenceFrame>
  class GSHOTEstimationBase : public FeatureFromNormals<PointInT, PointNT, PointOutT>,
                              public FeatureWithGlobalReferenceFrame<PointInT, PointRFT>
  {
    public:
      typedef boost::shared_ptr<GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> > Ptr;
      typedef boost::shared_ptr<const GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> > ConstPtr;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::fake_indices_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::fake_surface_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::deinitCompute;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using FeatureWithGlobalReferenceFrame<PointInT, PointRFT>::frames_;
      using FeatureWithGlobalReferenceFrame<PointInT, PointRFT>::frames_never_defined_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

    protected:
      /** \brief Empty constructor.
        * \param[in] nr_shape_bins the number of bins in the shape histogram
        */
      GSHOTEstimationBase (int nr_shape_bins = 10) :
        nr_shape_bins_ (nr_shape_bins),
        shot_ (), grf_radius_ (0),
        sqradius_ (0), radius3_4_ (0), radius1_4_ (0), radius1_2_ (0),
        nr_grid_sector_ (32),
        maxAngularSectors_ (28),
        descLength_ (0)
      {
        feature_name_ = "GSHOTEstimationBase";
      };
      

    public:

      /** \brief Empty destructor */
      virtual ~GSHOTEstimationBase () {}

       /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
         * \param[in] index the index of the point in indices_
         * \param[in] indices the k-neighborhood point indices in surface_
         * \param[in] sqr_dists the k-neighborhood point distances in surface_
         * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
         */
      virtual void
      computePointSHOT (/*const int index,*/
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot) = 0;

      // /** \brief Set the radius used for local reference frame estimation if the frames are not set by the user */
      // virtual void
      // setGRFRadius (float radius) { grf_radius_ = radius; }

      /** \brief Get the radius used for local reference frame estimation */
      virtual float
      getGRFRadius () const { return grf_radius_; }

    protected:

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

      /** \brief Quadrilinear interpolation used when color and shape descriptions are NOT activated simultaneously
        *
        * \param[in] indices the neighborhood point indices
        * \param[in] sqr_dists the neighborhood point distances
        * \param[in] index the index of the point in indices_
        * \param[out] binDistance the resultant distance shape histogram
        * \param[in] nr_bins the number of bins in the shape histogram
        * \param[out] shot the resultant SHOT histogram
        */
      void
      interpolateSingleChannel (const std::vector<int> &indices,
                                const std::vector<float> &sqr_dists,
                                //const int index,
                                std::vector<double> &binDistance,
                                const int nr_bins,
                                Eigen::VectorXf &shot);

      /** \brief Normalize the SHOT histogram.
        * \param[in,out] shot the SHOT histogram
        * \param[in] desc_length the length of the histogram
        */
      void
      normalizeHistogram (Eigen::VectorXf &shot, int desc_length);


      /** \brief Create a binned distance shape histogram
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[out] bin_distance_shape the resultant histogram
        */
      void
      createBinDistanceShape (/*int index, */const std::vector<int> &indices,
                              std::vector<double> &bin_distance_shape);

      /** \brief The number of bins in each shape histogram. */
      int nr_shape_bins_;

      /** \brief Placeholder for a point's SHOT. */
      Eigen::VectorXf shot_;

      /** \brief The radius used for the LRF computation */
      float grf_radius_;

      /** \brief The squared search radius. */
      double sqradius_;

      /** \brief 3/4 of the search radius. */
      double radius3_4_;

      /** \brief 1/4 of the search radius. */
      double radius1_4_;

      /** \brief 1/2 of the search radius. */
      double radius1_2_;

      /** \brief Number of azimuthal sectors. */
      const int nr_grid_sector_;

      /** \brief ... */
      const int maxAngularSectors_;

      /** \brief One SHOT length. */
      int descLength_;
  };

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * The suggested PointOutT is pcl::SHOT352
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
  class GSHOTEstimation : public GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>
  {
    public:
      typedef boost::shared_ptr<GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT> > Ptr;
      typedef boost::shared_ptr<const GSHOTEstimation<PointInT, PointNT, PointOutT, PointRFT> > ConstPtr;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::feature_name_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::getClassName;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::indices_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::k_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_parameter_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_radius_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::surface_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::input_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::normals_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::descLength_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_grid_sector_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_shape_bins_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::sqradius_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius3_4_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_4_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_2_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::maxAngularSectors_;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel;
      using GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::shot_;
      using FeatureWithGlobalReferenceFrame<PointInT, PointRFT>::frames_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor. */
      GSHOTEstimation () : GSHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> (10)
      {
        feature_name_ = "GSHOTEstimation";
      };
      
      /** \brief Empty destructor */
      virtual ~GSHOTEstimation () {}

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
        */
      virtual void
      computePointSHOT (//const int index,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot);
    protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeature (pcl::PointCloud<PointOutT> &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/gshot.hpp>
#endif

#endif  //#ifndef PCL_GSHOT_H_
