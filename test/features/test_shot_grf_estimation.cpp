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

// // #include <pcl/surface/mls.h>
// // #include <pcl/visualization/pcl_visualizer.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::search::KdTree<Point> KdTree;

typedef pcl::ReferenceFrame RFrame;
typedef pcl::PointCloud<RFrame> RFrames;

Cloud::Ptr cloud (new Cloud);
boost::shared_ptr<std::vector<int> > indices;
KdTree::Ptr tree;
float radius;

Cloud::Ptr cloud_trans (new Cloud);
boost::shared_ptr<std::vector<int> > indices_trans;
KdTree::Ptr tree_trans;
float radius_trans;

Cloud::Ptr cloud_noise (new Cloud);
boost::shared_ptr<std::vector<int> > indices_noise;
KdTree::Ptr tree_noise;
float radius_noise;

RFrames bunny_LRF;
RFrames bunny_noise_LRF;
RFrames bunny_noise_GRF;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTGlobalReferenceFrameEstimation)
{
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

  Eigen::Vector3f point_0_x (0.706783f, -0.682006f, 0.187951f);
  Eigen::Vector3f point_0_y (-0.707416f, -0.683025f, 0.181765f);
  Eigen::Vector3f point_0_z (0.00441039f, -0.261428f, -0.965213f);

  for (int d = 0; d < 3; ++d)
  {
    // // std::cout << bunny_LRF.at (0).x_axis[d] << " " << bunny_LRF.at (0).y_axis[d] << " " << bunny_LRF.at (0).z_axis[d] << std::endl;

    EXPECT_NEAR (point_0_x[d], bunny_LRF.at (0).x_axis[d], 1E-3);
    EXPECT_NEAR (point_0_y[d], bunny_LRF.at (0).y_axis[d], 1E-3);
    EXPECT_NEAR (point_0_z[d], bunny_LRF.at (0).z_axis[d], 1E-3);
    
    EXPECT_NEAR (bunny_GRF.at (0).x_axis[d], bunny_LRF.at (0).x_axis[d], 1E-3);
    EXPECT_NEAR (bunny_GRF.at (0).y_axis[d], bunny_LRF.at (0).y_axis[d], 1E-3);
    EXPECT_NEAR (bunny_GRF.at (0).z_axis[d], bunny_LRF.at (0).z_axis[d], 1E-3);

    // // EXPECT_NEAR (bunny_noise_LRF.at (0).x_axis[d], bunny_LRF.at (0).x_axis[d], 1E-2);
    // // EXPECT_NEAR (bunny_noise_LRF.at (0).y_axis[d], bunny_LRF.at (0).y_axis[d], 1E-2);
    // // EXPECT_NEAR (bunny_noise_LRF.at (0).z_axis[d], bunny_LRF.at (0).z_axis[d], 1E-2);

    // EXPECT_NEAR (bunny_noise_GRF.at (0).x_axis[d], bunny_GRF.at (0).x_axis[d], 1E-2);
    // EXPECT_NEAR (bunny_noise_GRF.at (0).y_axis[d], bunny_GRF.at (0).y_axis[d], 1E-2);
    // EXPECT_NEAR (bunny_noise_GRF.at (0).z_axis[d], bunny_GRF.at (0).z_axis[d], 1E-2);
  }
}

void
init_data (const Cloud::ConstPtr& _cloud, boost::shared_ptr<std::vector<int> >& _indices, KdTree::Ptr& _tree, float& _radius)
{
  Eigen::VectorXf centroid;
  pcl::computeNDCentroid<Point> (*_cloud, centroid);
  Point p_centroid;
  p_centroid.getVector4fMap () = centroid;

  _tree.reset (new KdTree (true));
  _tree->setInputCloud (_cloud);
  _indices.reset (new std::vector<int> (_cloud->size ()));
  std::vector<float> sqr_dist (_cloud->size ());
  int k = _tree->nearestKSearch (p_centroid, _cloud->size (), *_indices, sqr_dist);
  
  _radius = sqrt (sqr_dist[k - 1]);
  _indices->resize (1, *_indices->begin ());
}

void
add_gaussian_noise (const Cloud::ConstPtr& cloud_in, Cloud::Ptr& cloud_out)
{
  double standard_deviation = 0.0009;

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

  init_data (cloud, indices, tree, radius);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
  transform.prerotate (Eigen::AngleAxisf (0.25 * M_PI, Eigen::Vector3f::UnitX ()));
  transform.prerotate (Eigen::AngleAxisf (0.5 * M_PI, Eigen::Vector3f::UnitZ ()));
  transform.translate (Eigen::Vector3f (0, 1, 0));
  pcl::transformPointCloud<Point> (*cloud, *cloud_trans, transform);

  init_data (cloud_trans, indices_trans, tree_trans, radius_trans);

  Cloud::Ptr cloud_noise_without_mls (new Cloud);
  add_gaussian_noise (cloud, cloud_noise);

  // // pcl::visualization::PCLVisualizer viz;
  // // viz.addPointCloud (cloud_noise);
  // // viz.spin ();

  init_data (cloud_noise, indices_noise, tree_noise, radius_noise);

  // Compute SHOT LRF
  pcl::SHOTLocalReferenceFrameEstimation<Point, RFrame> lrf_estimator;
  lrf_estimator.setRadiusSearch (radius);
  lrf_estimator.setInputCloud (cloud);
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setIndices (indices);
  lrf_estimator.compute (bunny_LRF);

  // Compute SHOT LRF for bunny rotated
  pcl::SHOTLocalReferenceFrameEstimation<Point, RFrame> lrf_estimator_for_bunny_noised;
  lrf_estimator_for_bunny_noised.setRadiusSearch (radius_noise);
  lrf_estimator_for_bunny_noised.setInputCloud (cloud_noise);
  lrf_estimator_for_bunny_noised.setSearchMethod (tree_noise);
  lrf_estimator_for_bunny_noised.setIndices (indices_noise);
  lrf_estimator_for_bunny_noised.compute (bunny_noise_LRF);

  // Compute SHOT LRF for bunny rotated
  pcl::SHOTGlobalReferenceFrameEstimation<Point, RFrame> grf_estimator_for_bunny_noised;
  grf_estimator_for_bunny_noised.setInputCloud (cloud_noise);
  grf_estimator_for_bunny_noised.setSearchMethod (tree_noise);
  grf_estimator_for_bunny_noised.compute (bunny_noise_GRF);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
