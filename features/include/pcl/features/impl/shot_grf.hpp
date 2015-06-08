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

#ifndef PCL_FEATURES_IMPL_SHOT_GRF_H_
#define PCL_FEATURES_IMPL_SHOT_GRF_H_

#include <pcl/features/shot_grf.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <utility>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT>::getRFCenterAndRadius (const PointCloudInConstPtr& input,
                                                                                    const IndicesPtr& indices,
                                                                                    const PointCloudInConstPtr& surface,
                                                                                    int& rf_center,
                                                                                    float& rf_radius)
{
  PointInT query_point;
  Eigen::VectorXf centroid;
  computeNDCentroid<PointInT> (*surface, centroid);
  query_point.getVector4fMap () = centroid;

  typename pcl::search::Search<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT> (true));

  tree->setInputCloud (surface);
  std::vector<int> _indices (surface->size ());
  std::vector<float> sqr_distances (surface->size ());
  int k = tree->nearestKSearch (query_point, surface->size (), _indices, sqr_distances);
  float surface_radius = sqrt (sqr_distances[k-1]);
  int surface_centroid = _indices[0];

  size_t size = 0;
  if (indices->size () > 0)
  {
    tree->setInputCloud (input, indices);
    size = indices->size ();
  }
  else
  {
    tree->setInputCloud (input);
    size = input->size ();
  }
  _indices.resize (size, 0);
  sqr_distances.resize (size, 0.0f);
  k = tree->nearestKSearch (surface->points[surface_centroid], size, _indices, sqr_distances);

  rf_radius = surface_radius + sqrt(sqr_distances[0]);
  rf_center = _indices[0];
}


//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" matrix
template<typename PointInT, typename PointOutT> float
pcl::SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF (const Eigen::Vector4f& central_point, Eigen::Matrix3f &rf)
{
  //const Eigen::Vector4f& central_point = (*input_)[current_point_idx].getVector4fMap ();
  PointInT query_point;
  std::vector<int> n_indices;
  std::vector<float> n_sqr_distances;

  query_point.getVector4fMap () = central_point;
  tree_->radiusSearch (query_point, search_parameter_, n_indices, n_sqr_distances);
  // this->searchForNeighbors (current_point_idx, search_parameter_, n_indices, n_sqr_distances);

  Eigen::Matrix<double, Eigen::Dynamic, 4> vij (n_indices.size (), 4);

  Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();

  double distance = 0.0;
  double sum = 0.0;

  int valid_nn_points = 0;

  for (size_t i_idx = 0; i_idx < n_indices.size (); ++i_idx)
  {
    Eigen::Vector4f pt = surface_->points[n_indices[i_idx]].getVector4fMap ();
    if (pt.head<3> () == central_point.head<3> ())
		  continue;

    // Difference between current point and origin
    vij.row (valid_nn_points).matrix () = (pt - central_point).cast<double> ();
    vij (valid_nn_points, 3) = 0;

    distance = search_parameter_ - sqrt (n_sqr_distances[i_idx]);

    // Multiply vij * vij'
    cov_m += distance * (vij.row (valid_nn_points).head<3> ().transpose () * vij.row (valid_nn_points).head<3> ());

    sum += distance;
    valid_nn_points++;
  }

  if (valid_nn_points < 5)
  {
    //PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  cov_m /= sum;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov_m);

  const double& e1c = solver.eigenvalues ()[0];
  const double& e2c = solver.eigenvalues ()[1];
  const double& e3c = solver.eigenvalues ()[2];

  if (!pcl_isfinite (e1c) || !pcl_isfinite (e2c) || !pcl_isfinite (e3c))
  {
    //PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Eigenvectors are NaN. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  // Disambiguation
  Eigen::Vector4d v1 = Eigen::Vector4d::Zero ();
  Eigen::Vector4d v3 = Eigen::Vector4d::Zero ();
  v1.head<3> ().matrix () = solver.eigenvectors ().col (2);
  v3.head<3> ().matrix () = solver.eigenvectors ().col (0);

  int plusNormal = 0, plusTangentDirection1=0;
  for (int ne = 0; ne < valid_nn_points; ne++)
  {
    double dp = vij.row (ne).dot (v1);
    if (dp >= 0)
      plusTangentDirection1++;

    dp = vij.row (ne).dot (v3);
    if (dp >= 0)
      plusNormal++;
  }

  //TANGENT
  plusTangentDirection1 = 2*plusTangentDirection1 - valid_nn_points;
  if (plusTangentDirection1 == 0)
  {
		int points = 5; //std::min(valid_nn_points*2/2+1, 11);
		int medianIndex = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij.row (medianIndex - i).dot (v1) > 0)
				plusTangentDirection1 ++;

		if (plusTangentDirection1 < points/2+1)
			v1 *= - 1;
	} 
  else if (plusTangentDirection1 < 0)
    v1 *= - 1;

  //Normal
  plusNormal = 2*plusNormal - valid_nn_points;
  if (plusNormal == 0)
  {
		int points = 5; //std::min(valid_nn_points*2/2+1, 11);
		int medianIndex = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij.row (medianIndex - i).dot (v3) > 0)
				plusNormal ++;

		if (plusNormal < points/2+1)
			v3 *= - 1;
	} else if (plusNormal < 0)
    v3 *= - 1;

  rf.row (0).matrix () = v1.head<3> ().cast<float> ();
  rf.row (2).matrix () = v3.head<3> ().cast<float> ();
  rf.row (1).matrix () = rf.row (2).cross (rf.row (0));

  return (0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut& output)
{
  // check whether used with search radius or search k-neighbors
  if (this->getKSearch () != 0)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch(radius) to use this class.\n", getClassName ().c_str ());
    return;
  }
  
  Eigen::Matrix3f rf;
  PointOutT& output_rf = output[0];
  
  if (getLocalRF (central_point_, rf) == std::numeric_limits<float>::max ())
    output.is_dense = false;
  
  for (int d = 0; d < 3; ++d)
  {
    output_rf.x_axis[d] = rf.row (0)[d];
    output_rf.y_axis[d] = rf.row (1)[d];
    output_rf.z_axis[d] = rf.row (2)[d];
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return false;
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return false;
  }
  
  // Global RF cannot work with k-search specific
  if (this->getKSearch () != 0 )
  {
    PCL_ERROR("[pcl::%s::initCompute] Error! Search method set to k-neighborhood. Call setKSearch(0) to use this class.\n", getClassName().c_str ());
    return false;
  }

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

  tree_->setSortedResults (true);

  // float rf_radius = 0.0f;
  // int rf_center = 0;
  // if (search_radius_ == 0 || indices_->size () != 1)
  //   getRFCenterAndRadius (input_, indices_, surface_, rf_center, rf_radius);

  compute3DCentroid<PointInT, float> (*surface_, central_point_);
  Eigen::Vector4f max_pt;
  getMaxDistance<PointInT> (*surface_, central_point_, max_pt);
  if (search_radius_ == 0)
    search_radius_ = (max_pt - central_point_).squaredNorm ();
  search_parameter_ = search_radius_;
    

  // // Index set up to the centroid index of the input
  // if (indices_->size () != 1)
  // {
  //   fake_indices_ = false;
  //   indices_->resize (1);
  //   (*indices_)[0] = rf_center;
  // }
  
  return Feature<PointInT, PointOutT>::initCompute ();
}

#define PCL_INSTANTIATE_SHOTGlobalReferenceFrameEstimation(T,OutT) template class PCL_EXPORTS pcl::SHOTGlobalReferenceFrameEstimation<T,OutT>;

#endif /* PCL_FEATURES_IMPL_SHOT_GRF_H_ */

