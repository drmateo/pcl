/*
 * label_volume.h
 *
 *  Created on: Aug 23, 2018
 *      Author: Carlos M. Mateo
 */

#ifndef PCL_GPU_KINFU_LABELED_LABEL_VOLUME_H_
#define PCL_GPU_KINFU_LABELED_LABEL_VOLUME_H_

#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace pcl
{
  namespace gpu
  {
    class TsdfVolume;

    /** \brief LabelVolume class
     * \author Carlos M. Mateo, Sigma-Clermont, (cmateoagul@sigma-clermont.fr)
     */
    class PCL_EXPORTS LabelVolume
    {
      public:
        typedef boost::shared_ptr<LabelVolume> Ptr;

        /** \brief Supported Point Types */
        typedef PointXYZ PointType;

        /** \brief Constructor
          * \param[in] tsdf tsdf volume to get parameters from
          * \param[in] max_weight max weight for running average. Can be less than 255. Negative means default.
          */
        LabelVolume(const TsdfVolume& tsdf, int max_weight = -1);

        /** \brief Desctructor */
        ~LabelVolume() {}

        /** \brief Resets color volume to uninitialized state */
        void
        reset();

        /** \brief Returns running average length */
        inline int
        getMaxWeight() const
        {
          return max_weight_;
        }

        /** \brief Returns container with color volume in GPU memory */
        inline DeviceArray2D<int>
        data() const
        {
          return label_volume_;
        }

        /** \brief Computes labels from color volume
          * \param[in] cloud Points for which labels are to be computed.
          * \param[out] labels output array for labels
          */
        void
        fetchLabels (const DeviceArray<PointType>& cloud, DeviceArray<Label>& labels) const;

      private:
        /** \brief Volume resolution */
        Eigen::Vector3i resolution_;

        /** \brief Volume size in meters */
        Eigen::Vector3f volume_size_;

        /** \brief Length of running average */
        int max_weight_;

        /** \brief label volume data */
        DeviceArray2D<int> label_volume_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#endif /* PCL_GPU_KINFU_LABELED_LABEL_VOLUME_H_ */
