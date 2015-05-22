#ifndef PCL_FEATURES_IMPL_SHOT_GRF_H_
#define PCL_FEATURES_IMPL_SHOT_GRF_H_

#include <utility>
#include <pcl/features/shot_grf.h>
#include <pcl/common/centroid.h>



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

  float rf_radius = 0.0f;
  int rf_center = 0;
  if (search_radius_ == 0 || indices_->size () != 1)
    getRFCenterAndRadius (input_, indices_, surface_, rf_center, rf_radius);

  // Set up Search radius to the distances between centroid and farthest point if that is not already set
  if (search_radius_ == 0)
  {
    search_radius_ = rf_radius;
    search_parameter_ = search_radius_;
  }

  // Index set up to the centroid index of the input
  if (indices_->size () != 1)
  {
    fake_indices_ = false;
    indices_->resize (1);
    (*indices_)[0] = rf_center;
  }
  
  return Feature<PointInT, PointOutT>::initCompute ();
}

#define PCL_INSTANTIATE_SHOTGlobalReferenceFrameEstimation(T,OutT) template class PCL_EXPORTS pcl::SHOTGlobalReferenceFrameEstimation<T,OutT>;

#endif /* PCL_FEATURES_IMPL_SHOT_GRF_H_ */

