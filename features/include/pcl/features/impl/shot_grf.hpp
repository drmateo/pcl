#ifndef PCL_FEATURES_IMPL_SHOT_GRF_H_
#define PCL_FEATURES_IMPL_SHOT_GRF_H_

#include <utility>
#include <pcl/features/shot_grf.h>
#include <pcl/common/centroid.h>

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
  
  if (getGlobalRF (rf) == std::numeric_limits<float>::max ())
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
  
  // Global RF cannot work with k-search or radius-search specific
  if (this->getKSearch () != 0 || this->getRadiusSearch () != 0)
  {
    PCL_ERROR("[pcl::%s::initCompute] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch(0) to use this class.\n", getClassName().c_str ());
    return false;
  }

  // Search surface hasn't been defined, use the input dataset as the search surface itself
  if (surface_)
  {
    // PCL_ERROR ("[pcl::%s::initCompute] Error! Search surface haven't set up. This class just works with the InputCloud.", getClassName ().c_str ());
    // return false;
  }
  else
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
  
  // Find cloud centroid
  PointInT query_point;
  Eigen::VectorXf centroid;
  computeNDCentroid<PointInT> (*surface_, *indices_, centroid);
  query_point.getVector4fMap () = centroid;

  tree_->setSortedResults (true);
  indices_.reset (new std::vector<int> (surface_->size ()));
  std::vector<float> sqr_distances (surface_->size ());
  int k = tree_->nearestKSearch (query_point, surface_->size (), *indices_, sqr_distances);

  // Search radius set up to the distances between centroid and farthest point
  search_radius_ = sqrt (sqr_distances[k-1]);
  search_parameter_ = search_radius_;

  // Index set up to the centroid index of the surface (Not input cloud)
  fake_indices_ = false;
  int centroid_idx = (*indices_)[0];
  indices_->resize (1);
  (*indices_)[0] = centroid_idx;
  
  return Feature<PointInT, PointOutT>::initCompute ();
}

#define PCL_INSTANTIATE_SHOTGlobalReferenceFrameEstimation(T,OutT) template class PCL_EXPORTS pcl::SHOTGlobalReferenceFrameEstimation<T,OutT>;

#endif /* PCL_FEATURES_IMPL_SHOT_GRF_H_ */

