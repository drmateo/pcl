/*
 * label_volume.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: Carlos M. Mateo
 */

#include <pcl/gpu/kinfu_labeled/label_volume.h>
#include <pcl/gpu/kinfu_labeled/tsdf_volume_internal.h>
#include "internal.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::LabelVolume::LabelVolume(const TsdfVolume& tsdf, int max_weight) : resolution_(tsdf.getResolution()), volume_size_(tsdf.getSize()), max_weight_(1)
{
  max_weight_ = max_weight < 0 ? max_weight_ : max_weight;
  max_weight_ = max_weight_ > 255 ? 255 : max_weight_;

  int volume_x = resolution_(0);
  int volume_y = resolution_(1);
  int volume_z = resolution_(2);

  label_volume_.create (volume_y * volume_z, volume_x);
  reset();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::LabelVolume::reset()
{
  pcl::device::initLabelVolume(label_volume_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::LabelVolume::fetchLabels (const DeviceArray<PointType>& cloud, DeviceArray<Label>& labels) const
{
  labels.create(cloud.size());
  pcl::device::exctractLabels(label_volume_, pcl::device::device_cast<const float3> (volume_size_), cloud, (uint1*)labels.ptr());
}
