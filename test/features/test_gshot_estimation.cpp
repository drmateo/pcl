/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/features/gshot.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/features/shot_grf.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/transforms.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud_for_lrf;
PointCloud<PointNormal> ground_truth;
PointCloud<PointXYZ> cloud;
PointCloud<PointXYZ> cloud2;
PointCloud<PointXYZ> cloud3;
vector<int> indices_for_lrf;
vector<int> indices;
vector<int> indices2;
vector<int> indices3;
vector<int> indices_local_shot;
KdTreePtr tree;
KdTreePtr tree2;
KdTreePtr tree3;
float radius_local_shot;


///////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
checkDescNear (const pcl::PointCloud<PointT>& d0, const pcl::PointCloud<PointT>& d1, float dist)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < d0.points[i].descriptor.size (); ++j)
      ASSERT_NEAR (d0.points[i].descriptor[j], d1.points[i].descriptor[j], dist);
}

///////////////////////////////////////////////////////////////////////////////////
template <> void
checkDescNear<SHOT352> (const pcl::PointCloud<SHOT352>& d0, const pcl::PointCloud<SHOT352>& d1, float dist)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < 352; ++j)
      ASSERT_NEAR (d0.points[i].descriptor[j], d1.points[i].descriptor[j], dist);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
checkDesc(const pcl::PointCloud<PointT>& d0, const pcl::PointCloud<PointT>& d1)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < d0.points[i].descriptor.size(); ++j)
      ASSERT_EQ (d0.points[i].descriptor[j], d1.points[i].descriptor[j]);
}

///////////////////////////////////////////////////////////////////////////////////
template <> void
checkDesc<SHOT352>(const pcl::PointCloud<SHOT352>& d0, const pcl::PointCloud<SHOT352>& d1)
{
  ASSERT_EQ (d0.size (), d1.size ());
  for (size_t i = 0; i < d1.size (); ++i)
    for (size_t j = 0; j < 352; ++j)
      ASSERT_EQ (d0.points[i].descriptor[j], d1.points[i].descriptor[j]);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT>
struct createSHOTDesc
{
  FeatureEstimation operator ()(const typename PointCloud<NormalT>::Ptr & normals,
                                const int nr_shape_bins_ = 10,
                                const int = 30,
                                const bool = true,
                                const bool = false) const
  {
    FeatureEstimation f(nr_shape_bins_);
    f.setInputNormals (normals);
    return (f);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT>
struct createSHOTDesc<FeatureEstimation, PointT, NormalT, SHOT352>
{
  FeatureEstimation operator ()(const typename PointCloud<NormalT>::Ptr & normals,
                                const int = 10,
                                const int = 30,
                                const bool = true,
                                const bool = false) const
  {
    FeatureEstimation f;
    f.setInputNormals (normals);
    return (f);
  }
};

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> void
testGSHOTIndicesAndSearchSurface (const typename PointCloud<PointT>::Ptr & points,
                                  const typename PointCloud<NormalT>::Ptr & normals,
                                  const boost::shared_ptr<vector<int> > & indxs,
                                  const int nr_shape_bins = 10,
                                  const int nr_color_bins = 30,
                                  const bool describe_shape = true,
                                  const bool describe_color = false)
{
  //
  // Test setIndices and setSearchSurface
  //
  PointCloud<OutputT> full_output, output0, output1, output2;

  // Compute for all points and then subsample the results
  FeatureEstimation est0 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins, describe_shape, describe_color);
  est0.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est0.setInputCloud (points);
  est0.compute (full_output);

  pcl::copyPointCloud<OutputT> (full_output, output0);

  // Compute with all points as "search surface" and the specified sub-cloud as "input"
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT>);
  copyPointCloud (*points, *indxs, *subpoints);
  FeatureEstimation est1 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins, nr_color_bins, describe_shape, describe_color);
  est1.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est1.setInputCloud (subpoints);
  est1.setSearchSurface (points);
  est1.compute (output1);

  // Compute with all points as "input" and the specified indices
  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins, nr_color_bins, describe_shape, describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est2.setInputCloud (points);
  est2.setIndices (indxs);
  est2.compute (output2);

  // All three of the above cases should produce equivalent results
  int num_reference_frames = 1;
  ASSERT_EQ (output0.size (), num_reference_frames);
  checkDesc<OutputT> (output0, output1);
  checkDesc<OutputT> (output0, output2);
  checkDesc<OutputT> (output1, output2);

  //
  // Test the combination of setIndices and setSearchSurface
  //
  PointCloud<OutputT> output3, output4;

  boost::shared_ptr<vector<int> > indxs2 (new vector<int> (0));
  for (size_t i = 0; i < (indxs2->size ()/2); ++i)
    indxs2->push_back (static_cast<int> (i));

  // Compute with all points as search surface + the specified sub-cloud as "input" but for only a subset of indices
  FeatureEstimation est3 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est3.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est3.setSearchSurface (points);
  est3.setInputCloud (subpoints);
  est3.setIndices (indxs2);
  est3.compute (output3);

  // Start with features for each point in "subpoints" and then subsample the results
  copyPointCloud<OutputT> (output0, output4); // (Re-using "output0" from above)

  // The two cases above should produce equivalent results
  checkDesc<OutputT> (output3, output4);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> void
testGSHOTGlobalReferenceFrame (const typename PointCloud<PointT>::Ptr & points,
                               const typename PointCloud<NormalT>::Ptr & normals,
                               const boost::shared_ptr<vector<int> > & indxs,
                               const int nr_shape_bins = 10,
                               const int nr_color_bins = 30,
                               const bool describe_shape = true,
                               const bool describe_color = false)
{
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT> ());
  copyPointCloud (*points, *indxs, *subpoints);

  boost::shared_ptr<vector<int> > indxs2 (new vector<int> (0));
  for (size_t i = 0; i < (indxs->size ()/2); ++i)
    indxs2->push_back (static_cast<int> (i));

  //
  // Test an external computation for the local reference frames
  //
  PointCloud<ReferenceFrame>::Ptr frames (new PointCloud<ReferenceFrame> ());
  SHOTGlobalReferenceFrameEstimation<PointT, pcl::ReferenceFrame> grf_estimator;
  // grf_estimator.setRadiusSearch (radius);
  grf_estimator.setInputCloud (subpoints);
  grf_estimator.setIndices (indxs2);
  grf_estimator.setSearchSurface(points);
  grf_estimator.compute (*frames);

  PointCloud<OutputT> output, output2;

  FeatureEstimation est = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est.setRadiusSearch (radius);
  est.setSearchSurface (points);
  est.setInputCloud (subpoints);
  est.setIndices (indxs2);
  est.compute (output);

  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est2.setRadiusSearch (radius);
  est2.setSearchSurface (points);
  est2.setInputCloud (subpoints);
  est2.setIndices (indxs2);
  est2.setInputReferenceFrames (frames);
  est2.compute (output2);

  // Check frames
  pcl::PointCloud<pcl::ReferenceFrame>::ConstPtr f = est.getInputReferenceFrames ();
  pcl::PointCloud<pcl::ReferenceFrame>::ConstPtr f2 = est2.getInputReferenceFrames ();
  int num_reference_frames = 1;
  ASSERT_EQ (frames->points.size (), num_reference_frames);
  ASSERT_EQ (frames->points.size (), f->points.size ());
  ASSERT_EQ (f2->points.size (), f->points.size ());
  for (int i = 0; i < static_cast<int> (frames->points.size ()); ++i)
  {
    for (unsigned j = 0; j < 9; ++j)
      ASSERT_EQ (frames->points[i].rf[j], f->points[i].rf[j]);

    for (unsigned j = 0; j < 9; ++j)
      ASSERT_EQ (frames->points[i].rf[j], f2->points[i].rf[j]);
  }

  // The two cases above should produce equivalent results
  checkDesc<OutputT> (output, output2);
}

void
add_gaussian_noise (const PointCloud<PointXYZ>::ConstPtr& cloud_in, PointCloud<PointXYZ>::Ptr& cloud_out, double standard_deviation = 0.001)
{
  boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
  boost::normal_distribution<> nd (0, standard_deviation);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  
  cloud_out->resize (cloud_in->size ());
  for (size_t point_i = 0; point_i < cloud_in->points.size (); ++ point_i)
  {
    cloud_out->points[point_i].x = cloud_in->points[point_i].x + static_cast<float> (var_nor ());
    cloud_out->points[point_i].y = cloud_in->points[point_i].y + static_cast<float> (var_nor ());
    cloud_out->points[point_i].z = cloud_in->points[point_i].z + static_cast<float> (var_nor ());
  }
  cloud_out->header = cloud_in->header;
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GSHOTShapeEstimation)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setInputCloud (cloud.makeShared ());
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  n.compute (*normals);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

  // Objects
  PointCloud<SHOT352>::Ptr gshots352 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr shots352 (new PointCloud<SHOT352> ());
  
  // SHOT352 (local)
  SHOTEstimation<PointXYZ, Normal, SHOT352> shot352;
  shot352.setInputNormals (normals);
  shot352.setRadiusSearch (radius_local_shot);
  shot352.setInputCloud (cloud_for_lrf.makeShared ());
  boost::shared_ptr<vector<int> > indices_local_shot_ptr (new vector<int> (indices_local_shot));
  shot352.setIndices (indices_local_shot_ptr);
  shot352.setSearchSurface (cloud.makeShared());
  shot352.compute (*shots352);

  EXPECT_NEAR (shots352->points[0].descriptor[9 ], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[10], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[11], 0.317935f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[19], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[20], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[21], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[42], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[53], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[54], 0.0f, 1E-4);
  EXPECT_NEAR (shots352->points[0].descriptor[55], 0.089004f, 1E-4);

  // SHOT352 (global)
  GSHOTEstimation<PointXYZ, Normal, SHOT352> gshot352;
  gshot352.setSearchMethod (tree);

  gshot352.setInputNormals (normals);
  EXPECT_EQ (gshot352.getInputNormals (), normals);

  // set parameters
  gshot352.setInputCloud (cloud.makeShared ());
  gshot352.setIndices (indicesptr);

  // estimate
  int gshot_size = 1;
  gshot352.compute (*gshots352);
  EXPECT_EQ (gshots352->points.size (), gshot_size);

  checkDescNear (*gshots352, *shots352, 1E-7);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GSHOTIndicesAnSearchSurface)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testGSHOTIndicesAndSearchSurface<GSHOTEstimation<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTGlobalReferenceFrame)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testGSHOTGlobalReferenceFrame<GSHOTEstimation<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GSHOTRadius)
{
  float radius = radius_local_shot / 4.0f;
  
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  // Objects
  PointCloud<SHOT352>::Ptr gshots352 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr shots352 (new PointCloud<SHOT352> ());
  
  // SHOT352 (local)
  SHOTEstimation<PointXYZ, Normal, SHOT352> shot352;
  shot352.setInputNormals (normals);
  shot352.setRadiusSearch (radius);
  shot352.setInputCloud (cloud_for_lrf.makeShared ());
  boost::shared_ptr<vector<int> > indices_local_shot_ptr (new vector<int> (indices_local_shot));
  shot352.setIndices (indices_local_shot_ptr);
  shot352.setSearchSurface (cloud.makeShared());
  shot352.compute (*shots352);

  // SHOT352 (global)
  GSHOTEstimation<PointXYZ, Normal, SHOT352> gshot352;
  gshot352.setInputNormals (normals);
  // set parameters
  gshot352.setInputCloud (cloud.makeShared ());
  gshot352.setIndices (indicesptr);
  gshot352.setSearchMethod (tree);
  gshot352.setRadiusSearch (radius);
  EXPECT_EQ (gshot352.getRadiusSearch (), shot352.getRadiusSearch ());
  // estimate
  gshot352.compute (*gshots352);

  checkDescNear (*gshots352, *shots352, 1E-7);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GSHOTWithRTransNoised)
{
  PointCloud<PointNormal>::Ptr cloud_nr (new PointCloud<PointNormal> ());
  PointCloud<PointNormal>::Ptr cloud_rot (new PointCloud<PointNormal> ());
  PointCloud<PointNormal>::Ptr cloud_trans (new PointCloud<PointNormal> ());
  PointCloud<PointNormal>::Ptr cloud_rot_trans (new PointCloud<PointNormal> ());
  PointCloud<PointXYZ>::Ptr cloud_noise (new PointCloud<PointXYZ> ());

  Eigen::Affine3f rot = Eigen::Affine3f::Identity ();
  float rot_x = static_cast <float> (rand ()) / static_cast <float> (RAND_MAX);
  float rot_y = static_cast <float> (rand ()) / static_cast <float> (RAND_MAX);
  float rot_z = static_cast <float> (rand ()) / static_cast <float> (RAND_MAX);
  rot.prerotate (Eigen::AngleAxisf (rot_x * M_PI, Eigen::Vector3f::UnitX ()));
  rot.prerotate (Eigen::AngleAxisf (rot_y * M_PI, Eigen::Vector3f::UnitY ()));
  rot.prerotate (Eigen::AngleAxisf (rot_z * M_PI, Eigen::Vector3f::UnitZ ()));
  //std::cout << "rot = (" << (rot_x * M_PI) << ", " << (rot_y * M_PI) << ", " << (rot_z * M_PI) << ")" << std::endl;

  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  float HI = 5.0f;
  float LO = -HI;
  float trans_x = LO + static_cast<float> (rand ()) / (static_cast<float> (RAND_MAX / (HI - LO)));
  float trans_y = LO + static_cast<float> (rand ()) / (static_cast<float> (RAND_MAX / (HI - LO)));
  float trans_z = LO + static_cast<float> (rand ()) / (static_cast<float> (RAND_MAX / (HI - LO)));
  //std::cout << "trans = (" << trans_x << ", " << trans_y << ", " << trans_z << ")" << std::endl;
  trans.translate (Eigen::Vector3f (trans_x, trans_y, trans_z));

  // Estimate normals first
  float mr = 0.002;
  NormalEstimation<PointXYZ, pcl::Normal> n;
  PointCloud<Normal>::Ptr normals1 (new PointCloud<Normal> ());
  n.setViewPoint (0.0, 0.0, 1.0);
  n.setInputCloud (cloud.makeShared ());
  n.setRadiusSearch (20 * mr);
  n.compute (*normals1);

  pcl::concatenateFields<PointXYZ, Normal, PointNormal> (cloud, *normals1, *cloud_nr);
  pcl::transformPointCloudWithNormals<PointNormal, float> (*cloud_nr, *cloud_rot, rot);
  pcl::transformPointCloudWithNormals<PointNormal, float> (*cloud_nr, *cloud_trans, trans);
  pcl::transformPointCloudWithNormals<PointNormal, float> (*cloud_rot, *cloud_rot_trans, trans);

  add_gaussian_noise (cloud.makeShared (), cloud_noise, 0.005);

  PointCloud<Normal>::Ptr normals_noise (new PointCloud<Normal> ());
  n.setInputCloud (cloud_noise);
  n.compute (*normals_noise);

  PointCloud<Normal>::Ptr normals2 (new PointCloud<Normal> ());
  n.setInputCloud (cloud2.makeShared ());
  n.compute (*normals2);

  PointCloud<Normal>::Ptr normals3 (new PointCloud<Normal> ());
  n.setInputCloud (cloud3.makeShared ());
  n.compute (*normals3);

  // Objects
  // Descriptors for ground truth (using SHOT)
  PointCloud<SHOT352>::Ptr desc01 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc02 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc03 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc04 (new PointCloud<SHOT352> ());
  // Descriptors for test GSHOT
  PointCloud<SHOT352>::Ptr desc1 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc2 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc3 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc4 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc5 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc6 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr desc7 (new PointCloud<SHOT352> ());

  // SHOT352 (global)
  GSHOTEstimation<PointNormal, PointNormal, SHOT352> gshot1;
  gshot1.setInputNormals (cloud_nr);
  gshot1.setInputCloud (cloud_nr);
  gshot1.compute (*desc1);
  // Eigen::Vector4f center_desc1 = gshot.getCentralPoint ();

  gshot1.setInputNormals (cloud_rot);
  gshot1.setInputCloud (cloud_rot);
  gshot1.compute (*desc2);
  // Eigen::Vector4f center_desc2 = gshot.getCentralPoint ();

  gshot1.setInputNormals (cloud_trans);
  gshot1.setInputCloud (cloud_trans);
  gshot1.compute (*desc3);
  // Eigen::Vector4f center_desc3 = gshot.getCentralPoint ();

  gshot1.setInputNormals (cloud_rot_trans);
  gshot1.setInputCloud (cloud_rot_trans);
  gshot1.compute (*desc4);
  // Eigen::Vector4f center_desc4 = gshot.getCentralPoint ();

  GSHOTEstimation<PointXYZ, Normal, SHOT352> gshot2;
  gshot2.setInputNormals (normals1);
  gshot2.setInputCloud (cloud_noise);
  gshot2.compute (*desc5);

  gshot2.setInputNormals (normals2);
  gshot2.setInputCloud (cloud2.makeShared ());
  gshot2.compute (*desc6);

  gshot2.setInputNormals (normals3);
  gshot2.setInputCloud (cloud3.makeShared ());
  gshot2.compute (*desc7);

  // Eigen::Vector3f distance_desc = (center_desc3.head<3> () - center_desc1.head<3> ());
  // std::cout << "dist of desc0 and desc3 -> (" << distance_desc[0] << ", " << distance_desc[1] << ", " << distance_desc[2] << ")\n";

  // SHOT352 (local)
  GSHOTEstimation<PointNormal, PointNormal, SHOT352> shot;
  shot.setInputNormals (cloud_nr);
  shot.setInputCloud (ground_truth.makeShared());
  shot.setSearchSurface (cloud_nr);
  shot.setRadiusSearch (radius_local_shot);
  shot.compute (*desc01);

  shot.setInputNormals (cloud_rot);
  shot.setInputCloud (ground_truth.makeShared());
  shot.setSearchSurface (cloud_rot);
  shot.setRadiusSearch (radius_local_shot);
  shot.compute (*desc02);

  shot.setInputNormals (cloud_trans);
  shot.setInputCloud (ground_truth.makeShared());
  shot.setSearchSurface (cloud_trans);
  shot.setRadiusSearch (radius_local_shot);
  shot.compute (*desc03);

  shot.setInputNormals (cloud_rot_trans);
  shot.setInputCloud (ground_truth.makeShared());
  shot.setSearchSurface (cloud_rot_trans);
  shot.setRadiusSearch (radius_local_shot);
  shot.compute (*desc04);

  // CHECK GSHOT
  checkDesc(*desc01, *desc1);
  checkDesc(*desc02, *desc2);
  checkDesc(*desc03, *desc3);
  checkDesc(*desc04, *desc4);

  std::vector<float> d0, d1, d2, d3, d4, d5, d6;
  for(int i = 0; i < 352; ++i)
  {
    d0.push_back(desc1->points[0].descriptor[i]);
    d1.push_back(desc2->points[0].descriptor[i]);
    d2.push_back(desc3->points[0].descriptor[i]);
    d3.push_back(desc4->points[0].descriptor[i]);
    d4.push_back(desc5->points[0].descriptor[i]);
    d5.push_back(desc6->points[0].descriptor[i]);
    d6.push_back(desc7->points[0].descriptor[i]);
  }

  float dist_0 = pcl::selectNorm< std::vector<float> > (d0, d0, 352, pcl::HIK) ;
  float dist_1 = pcl::selectNorm< std::vector<float> > (d0, d1, 352, pcl::HIK) ;
  float dist_2 = pcl::selectNorm< std::vector<float> > (d0, d2, 352, pcl::HIK) ;
  float dist_3 = pcl::selectNorm< std::vector<float> > (d0, d3, 352, pcl::HIK) ;
  float dist_4 = pcl::selectNorm< std::vector<float> > (d0, d4, 352, pcl::HIK) ;
  float dist_5 = pcl::selectNorm< std::vector<float> > (d0, d5, 352, pcl::HIK) ;
  float dist_6 = pcl::selectNorm< std::vector<float> > (d0, d6, 352, pcl::HIK) ;
  
  std::cout << ">> Itself[HIK]:      " << dist_0 << std::endl
            << ">> Rotation[HIK]:    " << dist_1 << std::endl
            << ">> Translate[HIK]:   " << dist_2 << std::endl
            << ">> Rot+Trans[HIK]    " << dist_3 << std::endl
            << ">> GaussNoise[HIK]:  " << dist_4 << std::endl
            << ">> bun03[HIK]:       " << dist_5 << std::endl
            << ">> milk[HIK]:        " << dist_6 << std::endl;

  float high_barrier = dist_0 * 0.999f;
  float noise_barrier = dist_0 * 0.75f;
  float cut_barrier = dist_0 * 0.20f;
  float low_barrier = dist_0 * 0.02f;

  EXPECT_GT (dist_1, high_barrier);
  EXPECT_GT (dist_2, high_barrier);
  //EXPECT_GT (dist_3, high_barrier);
  EXPECT_GT (dist_4, noise_barrier);
  EXPECT_GT (dist_5, cut_barrier);
  EXPECT_LT (dist_6, low_barrier);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` `bun03.pcd` `milk.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  //
  // Load first cloud and prepare objets to test 
  //
  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  cloud_for_lrf = cloud;

  indices.resize (cloud.size (), 0);
  indices_for_lrf.resize (cloud.size (), 0);
  for (size_t i = 0; i < indices.size (); ++i)
  {
    indices[i] = static_cast<int> (i);
    indices_for_lrf[i] = static_cast<int> (i);
  }

  tree.reset (new search::KdTree<PointXYZ> ());
  tree->setInputCloud (cloud.makeShared ());
  tree->setSortedResults (true);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<PointXYZ, float> (cloud_for_lrf, centroid);

  Eigen::Vector4f max_pt;
  pcl::getMaxDistance<PointXYZ> (cloud_for_lrf, centroid, max_pt);
  radius_local_shot = (max_pt - centroid).norm();

  PointXYZ p_centroid;
  p_centroid.getVector4fMap () = centroid;
  cloud_for_lrf.push_back (p_centroid);
  PointNormal p_centroid_nr;
  p_centroid_nr.getVector4fMap() = centroid;
  ground_truth.push_back(p_centroid_nr);
  cloud_for_lrf.height = 1;
  cloud_for_lrf.width = cloud_for_lrf.size ();

  indices_for_lrf.push_back (cloud_for_lrf.width - 1);
  indices_local_shot.resize (1);
  indices_local_shot[0] = cloud_for_lrf.width - 1;

  //
  // Load second cloud and prepare objets to test 
  //
  if (loadPCDFile<PointXYZ> (argv[2], cloud2) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun03.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices2.resize (cloud2.size (), 0);
  for (size_t i = 0; i < indices2.size (); ++i)
    indices2[i] = static_cast<int> (i);

  tree2.reset (new search::KdTree<PointXYZ> ());
  tree2->setInputCloud (cloud2.makeShared ());
  tree2->setSortedResults (true);

  //
  // Load third cloud and prepare objets to test 
  //
  if (loadPCDFile<PointXYZ> (argv[3], cloud3) < 0)
  {
    std::cerr << "Failed to read test file. Please download `milk.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices3.resize (cloud3.size (), 0);
  for (size_t i = 0; i < indices3.size (); ++i)
    indices3[i] = static_cast<int> (i);

  tree3.reset (new search::KdTree<PointXYZ> ());
  tree3->setInputCloud (cloud3.makeShared ());
  tree3->setSortedResults (true);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
