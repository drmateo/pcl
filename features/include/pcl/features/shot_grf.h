#ifndef PCL_FEATURES_SHOT_GRF_H_
#define PCL_FEATURES_SHOT_GRF_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/shot_lrf.h>

namespace pcl
{
  template<typename PointInT, typename PointOutT = ReferenceFrame>
  class SHOTGlobalReferenceFrameEstimation : public SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const SHOTGlobalReferenceFrameEstimation<PointInT, PointOutT> > ConstPtr;

      SHOTGlobalReferenceFrameEstimation ()
      {
        feature_name_ = "SHOTGlobalReferenceFrameEstimation";
      }

      virtual ~SHOTGlobalReferenceFrameEstimation ()
      {
      }

    protected:
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::feature_name_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::input_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::indices_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::surface_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::tree_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::fake_surface_;
      using Feature<PointInT, PointOutT>::fake_indices_;
      using Feature<PointInT, PointOutT>::initCompute;
      using Feature<PointInT, PointOutT>::deinitCompute;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getClassName;
      
      typedef typename SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      
      float
      getGlobalRF (Eigen::Matrix3f& rf)
      {
        return SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF ((*indices_)[0], rf);
      }
      
      virtual bool
      initCompute ();

      virtual void
      computeFeature (PointCloudOut& output);

    private:
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::setIndices;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::setRadiusSearch;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::setKSearch;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/shot_grf.hpp>
#endif

#endif /* PCL_FEATURES_SHOT_GRF_H_ */
