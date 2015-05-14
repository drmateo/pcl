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
#include <pcl/pcl_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_grf.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::search::KdTree<Point> KdTree;

typedef pcl::ReferenceFrame RFrame;
typedef pcl::PointCloud<RFrame> RFrames;

Cloud::Ptr cloud (new Cloud);
Cloud::Ptr cloud_trans (new Cloud);

boost::shared_ptr<std::vector<int> > indices;
boost::shared_ptr<std::vector<int> > indices_trans;
KdTree::Ptr tree;
KdTree::Ptr tree_trans;
float radius;
float radius_trans;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTGlobalReferenceFrameEstimation)
{
  RFrames bunny_LRF;
  // Compute SHOT LRF
  pcl::SHOTLocalReferenceFrameEstimation<Point, RFrame> lrf_estimator;
  lrf_estimator.setRadiusSearch (radius);
  lrf_estimator.setInputCloud (cloud);
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setIndices (indices);
  lrf_estimator.compute (bunny_LRF);

  RFrames bunny_trans_LRF;
  // Compute SHOT LRF for bunny rotated
  pcl::SHOTLocalReferenceFrameEstimation<Point, RFrame> lrf_estimator_for_bunny_rotated;
  lrf_estimator_for_bunny_rotated.setRadiusSearch (radius_trans);
  lrf_estimator_for_bunny_rotated.setInputCloud (cloud_trans);
  lrf_estimator_for_bunny_rotated.setSearchMethod (tree_trans);
  lrf_estimator_for_bunny_rotated.setIndices (indices_trans);
  lrf_estimator_for_bunny_rotated.compute (bunny_trans_LRF);

  RFrames bunny_GRF;
  // Compute SHOT GRF
  pcl::SHOTGlobalReferenceFrameEstimation<Point, RFrame> grf_estimator;
  grf_estimator.setInputCloud (cloud);
  grf_estimator.setSearchMethod (tree);
  grf_estimator.compute (bunny_GRF);

  // TESTS
  EXPECT_EQ (indices->size (), 1);
  EXPECT_EQ (indices->size (), bunny_GRF.size ());
  EXPECT_EQ (bunny_LRF.size (), bunny_GRF.size ());

  EXPECT_NEAR (radius, radius_trans, 1E-3);

  Eigen::Vector3f point_0_x (0.988621f, -0.0301856f, 0.147371f);
  Eigen::Vector3f point_0_y (0.105291f, 0.838536f, -0.534576f);
  Eigen::Vector3f point_0_z (-0.107439f, 0.54401f, 0.832172f);

  for (int d = 0; d < 3; ++d)
  {
    // std::cout << bunny_LRF.at (0).x_axis[d] << " " << bunny_LRF.at (0).y_axis[d] << " " << bunny_LRF.at (0).z_axis[d] << std::endl;
    EXPECT_NEAR (point_0_x[d], bunny_LRF.at (0).x_axis[d], 1E-3);
    EXPECT_NEAR (point_0_y[d], bunny_LRF.at (0).y_axis[d], 1E-3);
    EXPECT_NEAR (point_0_z[d], bunny_LRF.at (0).z_axis[d], 1E-3);

    EXPECT_NEAR (bunny_GRF.at (0).x_axis[d], bunny_LRF.at (0).x_axis[d], 1E-3);
    EXPECT_NEAR (bunny_GRF.at (0).y_axis[d], bunny_LRF.at (0).y_axis[d], 1E-3);
    EXPECT_NEAR (bunny_GRF.at (0).z_axis[d], bunny_LRF.at (0).z_axis[d], 1E-3);
  }
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

  if (pcl::io::loadPCDFile<Point> (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  pcl::visualization::PCLVisualizer viz;
  viz.addCoordinateSystem (0.1);
  viz.addPointCloud (cloud, "cloud 1");

  Eigen::VectorXf centroid;
  pcl::computeNDCentroid<Point> (*cloud, centroid);
  Point p_centroid;
  p_centroid.getVector4fMap () = centroid;
  std::cout << centroid << std::endl;
  tree.reset (new KdTree (true));
  tree->setInputCloud (cloud);
  indices.reset (new std::vector<int> ());
  std::vector<float> sqr_dist;
  int k = tree->nearestKSearch (p_centroid, cloud->size (), *indices, sqr_dist);
  std::cout << k << " " << cloud->size () << std::endl;
  radius = *std::max_element (sqr_dist.begin (), sqr_dist.end ());//sqr_dist[k - 1];
  indices->resize (1, *indices->begin ());
  viz.addSphere (p_centroid, radius, "sphere 1");

  Eigen::Affine3f rotation = Eigen::Affine3f::Identity ();
  rotation.prerotate (Eigen::AngleAxisf (0.25 * M_PI, Eigen::Vector3f::UnitX ()));
  rotation.prerotate (Eigen::AngleAxisf (0.5 * M_PI, Eigen::Vector3f::UnitZ ()));
  pcl::transformPointCloud<Point> (*cloud, *cloud_trans, rotation);
  viz.addPointCloud (cloud_trans, "cloud 2");

  pcl::computeNDCentroid<Point> (*cloud_trans, centroid);
  p_centroid.getVector4fMap () = centroid;
  std::cout << centroid << std::endl;
  tree_trans.reset (new KdTree (true));
  tree_trans->setInputCloud (cloud_trans);
  indices_trans.reset (new std::vector<int> ());
  sqr_dist.resize (0);
  k = tree->nearestKSearch (p_centroid, cloud_trans->size (), *indices_trans, sqr_dist);
  radius_trans = *std::max_element (sqr_dist.begin (), sqr_dist.end ());//sqr_dist[k -1];
  indices_trans->resize (1, *indices_trans->begin ());
  viz.addSphere (p_centroid, radius, "sphere 2");
  


  viz.spin ();


  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
