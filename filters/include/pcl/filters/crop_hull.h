 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2011, Willow Garage, Inc.
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
  */

#ifndef PCL_FILTERS_CROP_HULL_H_
#define PCL_FILTERS_CROP_HULL_H_

#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief Filter points that lie inside or outside a 3D closed surface or 2D
    * closed polygon, as generated by the ConvexHull or ConcaveHull classes.
    * \author James Crosby
    * \ingroup filters
    */
  template<typename PointT>
  class CropHull: public FilterIndices<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::input_;
    
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

      typedef boost::shared_ptr< CropHull<PointT> > Ptr;
      typedef boost::shared_ptr< const CropHull<PointT> > ConstPtr;

      /** \brief Empty Constructor. */
      CropHull () :
        hull_polygons_(),
        hull_cloud_(),
        dim_(3),
        crop_outside_(true)
      {
        filter_name_ = "CropHull";
      }

      /** \brief Set the vertices of the hull used to filter points.
        * \param[in] polygons Vector of polygons (Vertices structures) forming
        * the hull used for filtering points.
        */
      inline void
      setHullIndices (const std::vector<Vertices>& polygons)
      {
        hull_polygons_ = polygons;
      }

      /** \brief Get the vertices of the hull used to filter points.
        */
      std::vector<Vertices>
      getHullIndices () const
      {
        return (hull_polygons_);
      }
      
      /** \brief Set the point cloud that the hull indices refer to
        * \param[in] points the point cloud that the hull indices refer to
        */
      inline void
      setHullCloud (PointCloudConstPtr points)
      {
        hull_cloud_ = points;
      }

      /** \brief Get the point cloud that the hull indices refer to. */
      PointCloudConstPtr
      getHullCloud () const
      {
        return (hull_cloud_);
      }
    
      /** \brief Set the dimensionality of the hull to be used.
        * This should be set to correspond to the dimensionality of the
        * convex/concave hull produced by the pcl::ConvexHull and
        * pcl::ConcaveHull classes.
        * \param[in] dim Dimensionailty of the hull used to filter points.
        */
      inline void
      setDim (int dim)
      {
        dim_ = dim;
      }
      
      /** \brief Remove points outside the hull (default), or those inside the hull.
        * \param[in] crop_outside If true, the filter will remove points
        * outside the hull. If false, those inside will be removed.
        */
      inline void
      setCropOutside(bool crop_outside)
      {
        crop_outside_ = crop_outside;
      }

    protected:
      /** \brief Filter the input points using the 2D or 3D polygon hull.
        * \param[out] output The set of points that passed the filter
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Filter the input points using the 2D or 3D polygon hull.
        * \param[out] indices the indices of the set of points that passed the filter.
        */
      void        
      applyFilter (std::vector<int> &indices);

    private:  
      /** \brief Return the size of the hull point cloud in line with coordinate axes.
        * This is used to choose the 2D projection to use when cropping to a 2d
        * polygon.
        */
      Eigen::Vector3f
      getHullCloudRange ();
      
      /** \brief Apply the two-dimensional hull filter.
        * All points are assumed to lie in the same plane as the 2D hull, an
        * axis-aligned 2D coordinate system using the two dimensions specified
        * (PlaneDim1, PlaneDim2) is used for calculations.
        * \param[out] output The set of points that pass the 2D polygon filter.
        */
      template<unsigned PlaneDim1, unsigned PlaneDim2> void
      applyFilter2D (PointCloud &output); 

      /** \brief Apply the two-dimensional hull filter.
        * All points are assumed to lie in the same plane as the 2D hull, an
        * axis-aligned 2D coordinate system using the two dimensions specified
        * (PlaneDim1, PlaneDim2) is used for calculations.
        * \param[out] indices The indices of the set of points that pass the
        *                     2D polygon filter.
        */
      template<unsigned PlaneDim1, unsigned PlaneDim2> void
      applyFilter2D (std::vector<int> &indices);

       /** \brief Apply the three-dimensional hull filter.
         * Polygon-ray crossings are used for three rays cast from each point
         * being tested, and a  majority vote of the resulting
         * polygon-crossings is used to decide  whether the point lies inside
         * or outside the hull.
         * \param[out] output The set of points that pass the 3D polygon hull
         *                    filter.
         */
      void
      applyFilter3D (PointCloud &output);

      /** \brief Apply the three-dimensional hull filter.
        *  Polygon-ray crossings are used for three rays cast from each point
        *  being tested, and a  majority vote of the resulting
        *  polygon-crossings is used to decide  whether the point lies inside
        *  or outside the hull.
        * \param[out] indices The indices of the set of points that pass the 3D
        *                     polygon hull filter.
        */
      void
      applyFilter3D (std::vector<int> &indices);

      /** \brief Test an individual point against a 2D polygon.
        * PlaneDim1 and PlaneDim2 specify the x/y/z coordinate axes to use.
        * \param[in] point Point to test against the polygon.
        * \param[in] verts Vertex indices of polygon.
        * \param[in] cloud Cloud from which the vertex indices are drawn.
        */
      template<unsigned PlaneDim1, unsigned PlaneDim2> inline static bool
      isPointIn2DPolyWithVertIndices (const PointT& point,
                                      const Vertices& verts,
                                      const PointCloud& cloud);

      /** \brief Does a ray cast from a point intersect with an arbitrary
        * triangle in 3D?
        * See: http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
        * \param[in] point Point from which the ray is cast.
        * \param[in] ray   Vector in direction of ray.
        * \param[in] verts Indices of vertices making the polygon.
        * \param[in] cloud Cloud from which the vertex indices are drawn.
        */
      inline static bool
      rayTriangleIntersect (const PointT& point,
                            const Eigen::Vector3f& ray,
                            const Vertices& verts,
                            const PointCloud& cloud);


      /** \brief The vertices of the hull used to filter points. */
      std::vector<pcl::Vertices> hull_polygons_;

      /** \brief The point cloud that the hull indices refer to. */
      PointCloudConstPtr hull_cloud_;

      /** \brief The dimensionality of the hull to be used. */
      int dim_;

      /** \brief If true, the filter will remove points outside the hull. If
       * false, those inside will be removed.
       */
      bool crop_outside_;
  };

} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/crop_hull.hpp>
#endif

#endif // ifndef PCL_FILTERS_CROP_HULL_H_
