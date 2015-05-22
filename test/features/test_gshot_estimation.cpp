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
// #include <pcl/features/shot_lrf.h>
#include <pcl/features/shot_grf.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
PointCloud<PointXYZ> cloud2;
vector<int> indices;
vector<int> indices_local_shot;
float radius_local_shot;
KdTreePtr tree;

// ///////////////////////////////////////////////////////////////////////////////////
// template<typename PointT> void
// shotCopyPointCloud (const PointCloud<PointT> &cloud_in, const std::vector<int> &indices,
//                     PointCloud<PointT> &cloud_out)
// {
//   pcl::copyPointCloud<PointT>(cloud_in, indices, cloud_out);
// }

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
      ASSERT_EQ (static_cast<float> (d0.points[i].descriptor[j]), static_cast<float> (d1.points[i].descriptor[j]));
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
                                  const typename PointCloud<PointT>::Ptr & points2,
                                  const typename PointCloud<NormalT>::Ptr & normals,
                                  const typename PointCloud<NormalT>::Ptr & normals2,
                                  const boost::shared_ptr<vector<int> > & indices,
                                  const int nr_shape_bins = 10,
                                  const int nr_color_bins = 30,
                                  const bool describe_shape = true,
                                  const bool describe_color = false)
{
  // double radius = 0.04;
  //
  // Test setIndices and setSearchSurface
  //
  PointCloud<OutputT> full_output, full_output2, output0, output1, output2;

  // Compute for all points and then subsample the results
  FeatureEstimation est0 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins, describe_shape, describe_color);
  est0.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est0.setRadiusSearch (radius);
  est0.setInputCloud (points);
  est0.compute (full_output);

  // Compute for all points and then subsample the results
  FeatureEstimation est00 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals2, nr_shape_bins,nr_color_bins, describe_shape, describe_color);
  est00.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est0.setRadiusSearch (radius);
  est00.setInputCloud (points2);
  est00.compute (full_output2);

  //shotCopyPointCloud<OutputT> (full_output, *indices, output0);
  pcl::copyPointCloud<OutputT> (full_output, output0);

  // Compute with all points as "search surface" and the specified sub-cloud as "input"
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT>);
  copyPointCloud (*points, *indices, *subpoints);
  //subpoints->push_back (points->at (indices_local_shot[0]));
  FeatureEstimation est1 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins, nr_color_bins, describe_shape, describe_color);
  est1.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est1.setRadiusSearch (radius);
  est1.setInputCloud (subpoints);
  est1.setSearchSurface (points);
  est1.compute (output1);

  // Compute with all points as "input" and the specified indices
  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins, nr_color_bins, describe_shape, describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est2.setRadiusSearch (radius);
  est2.setInputCloud (points);
  est2.setIndices (indices);
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

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (static_cast<int> (i));

  // Compute with all points as search surface + the specified sub-cloud as "input" but for only a subset of indices
  FeatureEstimation est3 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est3.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est3.setRadiusSearch (radius);
  est3.setSearchSurface (points);
  est3.setInputCloud (subpoints);
  est3.setIndices (indices2);
  est3.compute (output3);

  // Start with features for each point in "subpoints" and then subsample the results
  copyPointCloud<OutputT> (output0, output4); // (Re-using "output0" from above)

  // The two cases above should produce equivalent results
  checkDesc<OutputT> (output3, output4);

  std::vector<float> d0, d1, d2, d3, d4;
  for(int i = 0; i < 352; ++i)
  {
    d0.push_back(full_output.points[0].descriptor[i]);
    d1.push_back(full_output2.points[0].descriptor[i]);
    d2.push_back(output1.points[0].descriptor[i]);
    d3.push_back(output2.points[0].descriptor[i]);
    d4.push_back(output3.points[0].descriptor[i]);
  }
  std::cout << pcl::selectNorm< std::vector<float> > (d0, d1, 352, pcl::CS) << std::endl;
  std::cout << pcl::selectNorm< std::vector<float> > (d0, d2, 352, pcl::CS) << std::endl;
  std::cout << pcl::selectNorm< std::vector<float> > (d0, d3, 352, pcl::CS) << std::endl;
  std::cout << pcl::selectNorm< std::vector<float> > (d0, d4, 352, pcl::CS) << std::endl;
  // std::cout << pcl::selectNorm< std::vector<float> > (d0, d1, 352, pcl::L2) << std::endl;
  // std::cout << pcl::selectNorm< std::vector<float> > (d0, d1, 352, pcl::L1) << std::endl;
  // std::cout << pcl::selectNorm< std::vector<float> > (d0, d1, 352, pcl::PF) << std::endl;
  // std::cout << pcl::selectNorm< std::vector<float> > (d0, d1, 352, pcl::JM) << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> void
testGSHOTGlobalReferenceFrame (const typename PointCloud<PointT>::Ptr & points,
                               const typename PointCloud<NormalT>::Ptr & normals,
                               const boost::shared_ptr<vector<int> > & indices,
                               const int nr_shape_bins = 10,
                               const int nr_color_bins = 30,
                               const bool describe_shape = true,
                               const bool describe_color = false)
{
  // double radius = 0.04;

  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT> ());
  copyPointCloud (*points, *indices, *subpoints);

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (static_cast<int> (i));
  //
  // Test an external computation for the local reference frames
  //
  PointCloud<ReferenceFrame>::Ptr frames (new PointCloud<ReferenceFrame> ());
  SHOTGlobalReferenceFrameEstimation<PointT, pcl::ReferenceFrame> grf_estimator;
  // grf_estimator.setRadiusSearch (radius);
  grf_estimator.setInputCloud (subpoints);
  grf_estimator.setIndices (indices2);
  grf_estimator.setSearchSurface(points);
  grf_estimator.compute (*frames);

  PointCloud<OutputT> output, output2;

  FeatureEstimation est = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est.setRadiusSearch (radius);
  est.setSearchSurface (points);
  est.setInputCloud (subpoints);
  est.setIndices (indices2);
  est.compute (output);

  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, OutputT>()(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  // est2.setRadiusSearch (radius);
  est2.setSearchSurface (points);
  est2.setInputCloud (subpoints);
  est2.setIndices (indices2);
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GSHOTShapeEstimation)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  boost::shared_ptr<vector<int> > indices_local_shot_ptr (new vector<int> (indices_local_shot));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  NormalEstimation<PointXYZ, Normal> n2;
  PointCloud<Normal>::Ptr normals2 (new PointCloud<Normal> ());
  // set parameters
  n2.setInputCloud (cloud2.makeShared ());
  n2.setSearchMethod (tree);
  n2.setRadiusSearch (20 * mr);
  n2.compute (*normals2);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

  // Object
  PointCloud<SHOT352>::Ptr gshots352 (new PointCloud<SHOT352> ());
  PointCloud<SHOT352>::Ptr shots352 (new PointCloud<SHOT352> ());
  
  // SHOT352 (local)
  SHOTEstimation<PointXYZ, Normal, SHOT352> shot352;
  shot352.setInputNormals (normals);
  shot352.setRadiusSearch (radius_local_shot);
  shot352.setInputCloud (cloud.makeShared ());
  shot352.setIndices (indices_local_shot_ptr);
  shot352.setSearchMethod (tree);
  shot352.compute (*shots352);

  // SHOT352 (global)
  GSHOTEstimation<PointXYZ, Normal, SHOT352> gshot352;
  gshot352.setInputNormals (normals);
  EXPECT_EQ (gshot352.getInputNormals (), normals);
  // set parameters
  gshot352.setInputCloud (cloud.makeShared ());
  gshot352.setIndices (indicesptr);
  gshot352.setSearchMethod (tree);
  // estimate
  int gshot_size = 1;
  gshot352.compute (*gshots352);
  EXPECT_EQ (gshots352->points.size (), gshot_size);

  // std::cout << "RF radius:" << std::endl;
  // std::cout << gshot352.getGRFRadius () << std::endl;
  // std::cout << shot352.getLRFRadius () << std::endl;

  // std::cout << "Radius search:" << std::endl;
  // std::cout << gshot352.getRadiusSearch () << std::endl;
  // std::cout << shot352.getRadiusSearch () << std::endl;

  // std::cout << "Indices:" << std::endl;
  // std::cout << *gshot352.getIndices ()->begin () << std::endl;
  // std::cout << *shot352.getIndices ()->begin () << std::endl;

  // std::cout << "Input:" << std::endl;
  // std::cout << gshot352.getInputCloud ()->size () << std::endl;
  // std::cout << shot352.getInputCloud ()->size () << std::endl;

  // std::cout << "Surface:" << std::endl;
  // if (gshot352.getSearchSurface ())
  //   std::cout << gshot352.getSearchSurface ()->size () << std::endl;
  // else
  //   std::cout << "NULL" << std::endl;
  // if (shot352.getSearchSurface ())
  //   std::cout << shot352.getSearchSurface ()->size () << std::endl;
  // else
  //   std::cout << "NULL" << std::endl;
  

  // std::cout << shots352->points[0].descriptor[9 ] << std::endl
  //           << shots352->points[0].descriptor[10] << std::endl
  //           << shots352->points[0].descriptor[11] << std::endl
  //           << shots352->points[0].descriptor[19] << std::endl
  //           << shots352->points[0].descriptor[20] << std::endl
  //           << shots352->points[0].descriptor[21] << std::endl
  //           << shots352->points[0].descriptor[42] << std::endl
  //           << shots352->points[0].descriptor[53] << std::endl
  //           << shots352->points[0].descriptor[54] << std::endl
  //           << shots352->points[0].descriptor[55] << std::endl;

  EXPECT_NEAR (shots352->points[0].descriptor[9 ], 0.116438, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[10], 0.00907089, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[11], 0.0108631, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[19], 0.0166472, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[20], 0.198635, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[21], 0.0705247, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[42], 0.13321, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[53], 0.00434722, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[54], 0.0182399, 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[55], 0.00329447, 1e-4);


  EXPECT_NEAR (shots352->points[0].descriptor[9 ], gshots352->points[0].descriptor[9 ], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[10], gshots352->points[0].descriptor[10], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[11], gshots352->points[0].descriptor[11], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[19], gshots352->points[0].descriptor[19], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[20], gshots352->points[0].descriptor[20], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[21], gshots352->points[0].descriptor[21], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[42], gshots352->points[0].descriptor[42], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[53], gshots352->points[0].descriptor[53], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[54], gshots352->points[0].descriptor[54], 1e-4);
  EXPECT_NEAR (shots352->points[0].descriptor[55], gshots352->points[0].descriptor[55], 1e-4);
  // checkDesc (*gshots352, *shots352);

  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (static_cast<int> (i));

  testGSHOTIndicesAndSearchSurface<GSHOTEstimation<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), cloud2.makeShared (), normals, normals2, test_indices);
  testGSHOTGlobalReferenceFrame<GSHOTEstimation<PointXYZ, Normal, SHOT352>, PointXYZ, Normal, SHOT352> (cloud.makeShared (), normals, test_indices);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }


  if (loadPCDFile<PointXYZ> (argv[2], cloud2) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (true));
  tree->setInputCloud (cloud.makeShared ());

  Eigen::VectorXf centroid;
  pcl::computeNDCentroid<PointXYZ> (cloud, centroid);
  PointXYZ p_centroid;
  p_centroid.getVector4fMap () = centroid;

  indices_local_shot.resize (cloud.size ());
  std::vector<float> sqr_dist (cloud.size ());
  int k = tree->nearestKSearch (p_centroid, cloud.size (), indices_local_shot, sqr_dist);
  
  radius_local_shot = sqrt (sqr_dist[k - 1]);
  int centroid_idx = indices_local_shot[0];
  indices_local_shot.resize (1);
  indices_local_shot[0] = centroid_idx;

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
