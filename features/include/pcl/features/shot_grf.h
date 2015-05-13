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
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getClassName;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::input_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::indices_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::surface_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::tree_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::search_parameter_;
      
      typedef typename SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      
      virtual void
      computeFeature (PointCloudOut& output);

    private:
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/shot_grf.hpp>
#endif

#endif /* PCL_FEATURES_SHOT_GRF_H_ */
