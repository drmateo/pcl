#include <pcl/features/impl/shot_grf.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE_PRODUCT(SHOTGlobalReferenceFrameEstimation, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::ReferenceFrame)))
#else
PCL_INSTANTIATE_PRODUCT(SHOTGlobalReferenceFrameEstimation, (PCL_XYZ_POINT_TYPES)((pcl::ReferenceFrame)))
#endif
#endif /* PCL_NO_PRECOMPILE */

