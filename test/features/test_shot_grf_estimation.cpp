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
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::search::KdTree<Point> KdTree;

typedef pcl::ReferenceFrame RFrame;
typedef pcl::PointCloud<RFrame> RFrames;

Cloud::Ptr cloud (new Cloud);
boost::shared_ptr<std::vector<int> > indices (new std::vector<int> ());
KdTree::Ptr tree;
float radius;

Cloud::Ptr cloud_noise (new Cloud);
boost::shared_ptr<std::vector<int> > indices_noise (new std::vector<int> ());
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
  grf_estimator.setSearchMethod (tree);

  grf_estimator.setInputCloud (cloud);
  grf_estimator.compute (bunny_GRF);

  /// TESTS
  // Check number of frames that return SHOTGlobalReferenceFrameEstimation
  EXPECT_EQ (indices->size (), 1);
  EXPECT_EQ (indices->size (), bunny_GRF.size ());
  EXPECT_EQ (bunny_LRF.size (), bunny_GRF.size ());

  // Check central point of the global reference frame
  Eigen::Vector4f central_pt = grf_estimator.getCentralPoint();
  EXPECT_NEAR ((float)central_pt[0], -0.0290809f, 1E-4);
  EXPECT_NEAR ((float)central_pt[1], 0.102653f, 1E-4);
  EXPECT_NEAR ((float)central_pt[2], 0.027302f, 1E-4);

  // Check global radius
  EXPECT_EQ (radius, grf_estimator.getRadiusSearch ());

  Eigen::Vector3f point_x (0.719954f, -0.66465f, 0.199765f);
  Eigen::Vector3f point_y (0.693964f, 0.685693f, -0.219634f);
  Eigen::Vector3f point_z (0.00900238f, 0.296756f, 0.954911f);
  for (int d = 0; d < 3; ++d)
  {
//    std::cout << bunny_LRF.at (0).x_axis[d] << " " << bunny_LRF.at (0).y_axis[d] << " " << bunny_LRF.at (0).z_axis[d] << std::endl;

    EXPECT_NEAR ((float)point_x[d], bunny_LRF.at (0).x_axis[d], 1E-4);
    EXPECT_NEAR ((float)point_y[d], bunny_LRF.at (0).y_axis[d], 1E-4);
    EXPECT_NEAR ((float)point_z[d], bunny_LRF.at (0).z_axis[d], 1E-4);

    EXPECT_NEAR (bunny_GRF.at (0).x_axis[d], bunny_LRF.at (0).x_axis[d], 1E-4);
    EXPECT_NEAR (bunny_GRF.at (0).y_axis[d], bunny_LRF.at (0).y_axis[d], 1E-4);
    EXPECT_NEAR (bunny_GRF.at (0).z_axis[d], bunny_LRF.at (0).z_axis[d], 1E-4);

    EXPECT_NEAR (bunny_noise_GRF.at (0).x_axis[d], bunny_GRF.at (0).x_axis[d], 1E-3);
    EXPECT_NEAR (bunny_noise_GRF.at (0).y_axis[d], bunny_GRF.at (0).y_axis[d], 1E-3);
    EXPECT_NEAR (bunny_noise_GRF.at (0).z_axis[d], bunny_GRF.at (0).z_axis[d], 1E-3);
  }
}

void
init_data (const Cloud::ConstPtr& _cloud, Eigen::Vector4f& _central_point, float& _radius)
{
  pcl::compute3DCentroid<Point, float> (*_cloud, _central_point);
  Eigen::Vector4f max_pt;
  pcl::getMaxDistance<Point> (*cloud, _central_point, max_pt);
  _radius = (max_pt - _central_point).norm();
}

void
add_gaussian_noise (const Cloud::ConstPtr& cloud_in, Cloud::Ptr& cloud_out, const double& standard_deviation = 0.001)
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

  Eigen::Vector4f central_point (0, 0, 0, 0);
  init_data (cloud, central_point, radius);

  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (cloud);
  Point center;
  center.getVector4fMap() = central_point;
  viz.addSphere (center, radius);
  viz.addCoordinateSystem (0.2f, "world ref");

  // Compute SHOT LRF
  pcl::SHOTLocalReferenceFrameEstimation<Point, RFrame> lrf_estimator;
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setRadiusSearch (radius);

  pcl::PointCloud<Point>::Ptr cloud_tmp (new pcl::PointCloud<Point> (*cloud));
  Point virtual_point;
  virtual_point.getVector4fMap () = central_point;
  cloud_tmp->points.push_back (virtual_point);
  cloud_tmp->height = 1;
  cloud_tmp->width = cloud_tmp->size ();
  indices->push_back (cloud_tmp->width - 1);

  lrf_estimator.setInputCloud (cloud_tmp);
  lrf_estimator.setIndices (indices);
  lrf_estimator.compute (bunny_LRF);
  viz.addSphere (cloud_tmp->points[(*indices)[0]], radius * 0.05, 1, 0, 1, "central_point");

  Eigen::Affine3f ref = Eigen::Affine3f::Identity();
  ref(0,0) = bunny_LRF.points[0].x_axis[0];
  ref(0,1) = bunny_LRF.points[0].x_axis[1];
  ref(0,2) = bunny_LRF.points[0].x_axis[2];
  ref(1,0) = bunny_LRF.points[0].y_axis[0];
  ref(1,1) = bunny_LRF.points[0].y_axis[1];
  ref(1,2) = bunny_LRF.points[0].y_axis[2];
  ref(2,0) = bunny_LRF.points[0].z_axis[0];
  ref(2,1) = bunny_LRF.points[0].z_axis[1];
  ref(2,2) = bunny_LRF.points[0].z_axis[2];
  ref.pretranslate(Eigen::Vector3f (central_point[0], central_point[1], central_point[2]));
  viz.addCoordinateSystem(radius, ref, "Ref");
  // viz.spin ();

  Cloud::Ptr cloud_noise_without_mls (new Cloud);
  add_gaussian_noise (cloud, cloud_noise, 0.0001);

  Eigen::Vector4f central_point_noise;
  init_data (cloud_noise, central_point_noise, radius_noise);

  // Compute SHOT GRF for bunny with noise
  pcl::SHOTGlobalReferenceFrameEstimation<Point, RFrame> grf_estimator_for_bunny_noised;
  grf_estimator_for_bunny_noised.setInputCloud (cloud_noise);
  grf_estimator_for_bunny_noised.setSearchMethod (tree_noise);
  grf_estimator_for_bunny_noised.compute (bunny_noise_GRF);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
