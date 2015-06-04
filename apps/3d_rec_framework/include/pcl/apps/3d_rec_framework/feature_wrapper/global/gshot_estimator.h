/*
 * gshot_estimator.h
 *
 *  Created on: Jun 03, 2015
 *      Author: Carlos M. Mateo
 */

#ifndef REC_FRAMEWORK_GSHOT_ESTIMATOR_H_
#define REC_FRAMEWORK_GSHOT_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/gshot.h>
#include <pcl/surface/mls.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
    class GSHOTEstimation : public GlobalEstimator<PointInT, FeatureT>
    {
      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;

      using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;
      using GlobalEstimator<PointInT, FeatureT>::normals_;

      public:
        GSHOTEstimation ()
          : adaptative_MLS_ (false)
        {}

        inline void setAdaptativeMLS (bool b) { adaptative_MLS_ = b; }

        inline bool computedNormals () { return true; }

        void
        estimate (PointInTPtr& in, PointInTPtr& processed,
                  typename pcl::PointCloud<FeatureT>::CloudVectorType& signatures,
                  std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& /*centroids*/)
        {
          if (!normal_estimator_)
          {
            PCL_ERROR ("This feature needs normals... please provide a normal estimator\n");
            return;
          }

          pcl::MovingLeastSquares<PointInT, PointInT> mls;
          if (adaptative_MLS_)
          {
            typename search::KdTree<PointInT>::Ptr tree;
            Eigen::Vector4f centroid_cluster;
            pcl::compute3DCentroid (*in, centroid_cluster);
            float dist_to_sensor = centroid_cluster.norm();
            float sigma = dist_to_sensor * 0.01f;
            mls.setSearchMethod (tree);
            mls.setSearchRadius (sigma);
            mls.setUpsamplingMethod (mls.SAMPLE_LOCAL_PLANE);
            mls.setUpsamplingRadius (0.002);
            mls.setUpsamplingStepSize (0.001);
          }

          normals_.reset (new pcl::PointCloud<pcl::Normal>);
          {
            normal_estimator_->estimate (in, processed, normals_);
          }

          if (adaptative_MLS_)
          {
            mls.setInputCloud(processed);

            PointInTPtr filtered(new pcl::PointCloud<PointInT>);
            mls.process (*filtered);

            processed.reset(new pcl::PointCloud<PointInT>);
            normals_.reset (new pcl::PointCloud<pcl::Normal>);
            {
              filtered->is_dense = false;
              normal_estimator_->estimate (filtered, processed, normals_);
            }
          }

          typedef typename pcl::GSHOTEstimation<PointInT, pcl::Normal, pcl::SHOT352> GSHOTEstimation;
          pcl::PointCloud<pcl::SHOT352> gshot_signatures;
          typename pcl::search::KdTree<PointInT>::Ptr gshot_tree (new pcl::search::KdTree<PointInT>);

          GSHOTEstimation gshot;
          gshot.setSearchMethod (gshot_tree);
          gshot.setInputCloud (processed);
          gshot.setInputNormals (normals_);
          gshot.compute (gshot_signatures);

          for (size_t i = 0; i < gshot_signatures.points.size (); i++)
          {
            pcl::PointCloud<FeatureT> shot_signature;
            shot_signature.points.resize (1);
            shot_signature.width = shot_signature.height = 1;
            for (int d = 0; d < gshot_signatures.points.begin ()->descriptorSize (); ++d)
              shot_signature.points[0].histogram[d] = gshot_signatures.points[i].descriptor[d];

            signatures.push_back (shot_signature);
          }

          // gshot.getCentroidClusters (centroids);
        }

      private:
        bool adaptative_MLS_;
    };
  }
}

#endif /* REC_FRAMEWORK_GSHOT_ESTIMATOR_H_ */
