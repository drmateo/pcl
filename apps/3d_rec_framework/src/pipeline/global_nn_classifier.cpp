/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/impl/global_nn_classifier.hpp>
#include "pcl/apps/3d_rec_framework/utils/metrics.h"

//This stuff is needed to be able to make the SHOT histograms persistent
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<352>,
    (float[352], histogram, histogram352)
)

//Instantiation
template class pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>;
template class pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308>;
template class pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>;
template class pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> >;

template class pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::ESFSignature640>;
template class pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::Histogram<352> >;

template class pcl::rec_3d_framework::GlobalClassifier<pcl::PointXYZ>;
