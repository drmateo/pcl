/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * Author: Bastian Steder
 */

#include <pcl/range_image/range_image.h>

#include <iostream>
using std::cout;
using std::cerr;
#include <map>
#include <set>
#include <cmath>
#include <pcl/pcl_macros.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/point_cloud.h>
#include <pcl/common/vector_average.h>
#include <pcl/features/eigen.h>
#include <pcl/features/range_image_border_extractor.h>

namespace pcl {

////////// STATIC //////////
float RangeImageBorderExtractor::getObstacleBorderAngle(const BorderTraits& border_traits)
{
  float x=0.0f, y=0.0f;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT])
    ++x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT])
    --x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP])
    --y;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM])
    ++y;
  
  return atan2f(y, x);
}

inline std::ostream& operator << (std::ostream& os, const RangeImageBorderExtractor::Parameters& p)
{
  os << PVARC(p.pixel_radius_borders)<<PVARC(p.pixel_radius_plane_extraction)<<PVARC(p.pixel_radius_border_direction)
     << PVARC(p.minimum_border_probability)<<PVARN(p.pixel_radius_principal_curvature);
  return (os);
}

////////// NON-STATIC //////////


float RangeImageBorderExtractor::getNeighborDistanceChangeScore(
    const RangeImageBorderExtractor::LocalSurface& local_surface,
    int x, int y, int offset_x, int offset_y, int pixel_radius) const
{
  const PointWithRange& point = range_image_->getPoint(x, y);
  PointWithRange neighbor;
  range_image_->get1dPointAverage(x+offset_x, y+offset_y, offset_x, offset_y, pixel_radius, neighbor);
  if (pcl_isinf(neighbor.range))
  {
    if (neighbor.range < 0.0f)
      return 0.0f;
    else
    {
      //cout << "INF edge -> Setting to 1.0\n";
      return 1.0f;  // TODO: Something more intelligent
    }
  }
  
  float neighbor_distance_squared = squaredEuclideanDistance(neighbor, point);
  if (neighbor_distance_squared <= local_surface.max_neighbor_distance_squared)
    return 0.0f;
  float ret = 1.0f - sqrtf(local_surface.max_neighbor_distance_squared / neighbor_distance_squared);
  if (neighbor.range < point.range)
    ret = -ret;
  return ret;
}

//float RangeImageBorderExtractor::getNormalBasedBorderScore(const RangeImageBorderExtractor::LocalSurface& local_surface,
                                                           //int x, int y, int offset_x, int offset_y) const
//{
  //PointWithRange neighbor;
  //range_image_->get1dPointAverage(x+offset_x, y+offset_y, offset_x, offset_y, parameters_.pixel_radius_borders, neighbor);
  //if (pcl_isinf(neighbor.range))
  //{
    //if (neighbor.range < 0.0f)
      //return 0.0f;
    //else
      //return 1.0f;  // TODO: Something more intelligent (Compare normal with viewing direction)
  //}
  
  //float normal_distance_to_plane_squared = local_surface.smallest_eigenvalue_no_jumps;
  //float distance_to_plane = local_surface.normal_no_jumps.dot(local_surface.neighborhood_mean_no_jumps-neighbor.getVector3fMap());
  //bool shadow_side = distance_to_plane < 0.0f;
  //float distance_to_plane_squared = pow(distance_to_plane, 2);
  //if (distance_to_plane_squared <= normal_distance_to_plane_squared)
    //return 0.0f;
  //float ret = 1.0f - (normal_distance_to_plane_squared/distance_to_plane_squared);
  //if (shadow_side)
    //ret = -ret;
  ////cout << PVARC(normal_distance_to_plane_squared)<<PVAR(distance_to_plane_squared)<<" => "<<ret<<"\n";
  //return ret;
//}

bool RangeImageBorderExtractor::get3dDirection(const BorderDescription& border_description, Eigen::Vector3f& direction,
                                               const LocalSurface* local_surface)
{
  const BorderTraits border_traits = border_description.traits;
  
  int delta_x=0, delta_y=0;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT])
    ++delta_x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT])
    --delta_x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP])
    --delta_y;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM])
    ++delta_y;
  
  if (delta_x==0 && delta_y==0)
    return false;
  
  int x=border_description.x, y=border_description.y;
  const PointWithRange& point = range_image_->getPoint(x, y);
  Eigen::Vector3f neighbor_point;
  range_image_->calculate3DPoint(static_cast<float> (x+delta_x), static_cast<float> (y+delta_y), point.range, neighbor_point);
  //cout << "Neighborhood point is "<<neighbor_point[0]<<", "<<neighbor_point[1]<<", "<<neighbor_point[2]<<".\n";
  
  if (local_surface!=NULL)
  {
    // Get the point that lies on the local plane approximation
    Eigen::Vector3f sensor_pos = range_image_->getSensorPos(),
                    viewing_direction = neighbor_point-sensor_pos;

    float lambda = (local_surface->normal_no_jumps.dot(local_surface->neighborhood_mean_no_jumps-sensor_pos)/
                   local_surface->normal_no_jumps.dot(viewing_direction));
    neighbor_point = lambda*viewing_direction + sensor_pos;
    //cout << "Neighborhood point projected onto plane is "<<neighbor_point[0]<<", "<<neighbor_point[1]<<", "<<neighbor_point[2]<<".\n";
  }
  //cout << point.x<<","<< point.y<<","<< point.z<<" -> "<< direction[0]<<","<< direction[1]<<","<< direction[2]<<"\n";
  direction = neighbor_point-point.getVector3fMap();
  direction.normalize();
  
  return true;
}

void RangeImageBorderExtractor::calculateBorderDirection(int x, int y)
{
  int index = y*range_image_->width + x;
  Eigen::Vector3f*& border_direction = border_directions_[index];
  border_direction = NULL;
  const BorderDescription& border_description = border_descriptions_->points[index];
  const BorderTraits& border_traits = border_description.traits;
  if (!border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
    return;
  border_direction = new Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  if (!get3dDirection(border_description, *border_direction, surface_structure_[index]))
  {
    delete border_direction;
    border_direction = NULL;
    return;
  }
}

bool RangeImageBorderExtractor::changeScoreAccordingToShadowBorderValue(int x, int y, int offset_x, int offset_y, float* border_scores,
                                                                        float* border_scores_other_direction, int& shadow_border_idx) const
{
  float& border_score = border_scores[y*range_image_->width+x];

  shadow_border_idx = -1;
  if (border_score<parameters_.minimum_border_probability)
    return false;
  
  if (border_score == 1.0f) 
  {  // INF neighbor?
    if (range_image_->isMaxRange(x+offset_x, y+offset_y))
    {
      shadow_border_idx = (y+offset_y)*range_image_->width + x+offset_x;
      return true;
    }
  }
  
  float best_shadow_border_score = 0.0f;
  
  for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  {
    int neighbor_x=x+neighbor_distance*offset_x, neighbor_y=y+neighbor_distance*offset_y;
    if (!range_image_->isInImage(neighbor_x, neighbor_y))
      continue;
    float neighbor_shadow_border_score = border_scores_other_direction[neighbor_y*range_image_->width+neighbor_x];
    
    if (neighbor_shadow_border_score < best_shadow_border_score)
    {
      shadow_border_idx = neighbor_y*range_image_->width + neighbor_x;
      best_shadow_border_score = neighbor_shadow_border_score;
    }
  }
  if (shadow_border_idx >= 0)
  {
    //cout << PVARC(border_score)<<PVARN(best_shadow_border_score);
    //border_score *= (std::max)(0.9f, powf(-best_shadow_border_score, 0.1f));  // TODO: Something better
    border_score *= (std::max)(0.9f, 1-powf(1+best_shadow_border_score, 3));
    if (border_score>=parameters_.minimum_border_probability)
      return true;
  }
  shadow_border_idx = -1;
  border_score = 0.0f;  // Since there was no shadow border found we set this value to zero, so that it does not influence the maximum search
  return false;
}

float RangeImageBorderExtractor::updatedScoreAccordingToNeighborValues(int x, int y, const float* border_scores) const
{
  float max_score_bonus = 0.5f;
  
  float border_score = border_scores[y*range_image_->width+x];
  
  // Check if an update can bring the score to a value higher than the minimum
  if (border_score + max_score_bonus*(1.0f-border_score) < parameters_.minimum_border_probability)
    return border_score;
  
  float average_neighbor_score=0.0f, weight_sum=0.0f;
  for (int y2=y-1; y2<=y+1; ++y2)
  {
    for (int x2=x-1; x2<=x+1; ++x2)
    {
      if (!range_image_->isInImage(x2, y2) || (x2==x&&y2==y))
        continue;
      average_neighbor_score += border_scores[y2*range_image_->width+x2];
      weight_sum += 1.0f;
    }
  }
  average_neighbor_score /=weight_sum;
  
  if (average_neighbor_score*border_score < 0.0f)
    return border_score;
  
  float new_border_score = border_score + max_score_bonus * average_neighbor_score * (1.0f-fabsf(border_score));
  
  //std::cout << PVARC(border_score)<<PVARN(new_border_score);
  return new_border_score;
}

bool RangeImageBorderExtractor::checkPotentialBorder(int x, int y, int offset_x, int offset_y, float* border_scores,
                                                     float* border_scores_other_direction, int& shadow_border_idx) const
{
  float& border_score = border_scores[y*range_image_->width+x];
  if (border_score<parameters_.minimum_border_probability)
    return false;
  
  shadow_border_idx = -1;
  float best_shadow_border_score = -0.5f*parameters_.minimum_border_probability;
  
  for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  {
    int neighbor_x=x+neighbor_distance*offset_x, neighbor_y=y+neighbor_distance*offset_y;
    if (!range_image_->isInImage(neighbor_x, neighbor_y))
      continue;
    float neighbor_shadow_border_score = border_scores_other_direction[neighbor_y*range_image_->width+neighbor_x];
    
    if (neighbor_shadow_border_score < best_shadow_border_score)
    {
      shadow_border_idx = neighbor_y*range_image_->width + neighbor_x;
      best_shadow_border_score = neighbor_shadow_border_score;
    }
  }
  if (shadow_border_idx >= 0)
  {
    return true;
  }
  border_score = 0.0f;  // Since there was no shadow border found we set this value to zero, so that it does not influence the maximum search
  return false;
}

bool RangeImageBorderExtractor::checkIfMaximum(int x, int y, int offset_x, int offset_y, float* border_scores, int shadow_border_idx) const
{
  float border_score = border_scores[y*range_image_->width+x];
  int neighbor_x=x-offset_x, neighbor_y=y-offset_y;
  if (range_image_->isInImage(neighbor_x, neighbor_y) && border_scores[neighbor_y*range_image_->width+neighbor_x] > border_score)
    return false;
  
  for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  {
    neighbor_x=x+neighbor_distance*offset_x; neighbor_y=y+neighbor_distance*offset_y;
    if (!range_image_->isInImage(neighbor_x, neighbor_y))
      continue;
    int neighbor_index = neighbor_y*range_image_->width + neighbor_x;
    if (neighbor_index==shadow_border_idx)
      return true;
    
    float neighbor_border_score = border_scores[neighbor_index];
    if (neighbor_border_score > border_score)
      return false;
  }
  return true;
}

bool RangeImageBorderExtractor::calculateMainPrincipalCurvature(int x, int y, int radius, float& magnitude,
                                                                Eigen::Vector3f& main_direction) const
{
  magnitude = 0.0f;
  int index = y*range_image_->width+x;
  LocalSurface* local_surface = surface_structure_[index];
  if (local_surface==NULL)
    return false;
  //const PointWithRange& point = range_image_->getPointNoCheck(x,y);
  
  //Eigen::Vector3f& normal = local_surface->normal_no_jumps;
  //Eigen::Matrix3f to_tangent_plane = Eigen::Matrix3f::Identity() - normal*normal.transpose();
  
  VectorAverage3f vector_average;
  bool beams_valid[9];
  for (int step=1; step<=radius; ++step)
  {
    int beam_idx = 0;
    for (int y2=y-step; y2<=y+step; y2+=step)
    {
      for (int x2=x-step; x2<=x+step; x2+=step)
      {
        bool& beam_valid = beams_valid[beam_idx++];
        if (step==1)
        {
          if (x2==x && y2==y)
            beam_valid = false;
          else
            beam_valid = true;
        }
        else
          if (!beam_valid)
            continue;
        //std::cout << x2-x<<","<<y2-y<<"  ";
        
        if (!range_image_->isValid(x2,y2))
          continue;
        
        int index2 = y2*range_image_->width + x2;
        
        const BorderTraits& border_traits = border_descriptions_->points[index2].traits;
        if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        {
          beam_valid = false;
          continue;
        }
        
        //const PointWithRange& point2 = range_image_->getPoint(index2);
        LocalSurface* local_surface2 = surface_structure_[index2];
        if (local_surface2==NULL)
          continue;
        Eigen::Vector3f& normal2 = local_surface2->normal_no_jumps;
        //float distance_squared = squaredEuclideanDistance(point, point2);
        //vector_average.add(to_tangent_plane*normal2);
        vector_average.add(normal2);
      }
    }
  }
  //std::cout << "\n";
  if (vector_average.getNoOfSamples() < 3)
    return false;
  
  Eigen::Vector3f eigen_values, eigen_vector1, eigen_vector2;
  vector_average.doPCA(eigen_values, eigen_vector1, eigen_vector2, main_direction);
  magnitude = sqrtf(eigen_values[2]);
  //magnitude = eigen_values[2];
  //magnitude = 1.0f - powf(1.0f-magnitude, 5);
  //magnitude = 1.0f - powf(1.0f-magnitude, 10);
  //magnitude += magnitude - powf(magnitude,2);
  //magnitude += magnitude - powf(magnitude,2);
  
  //magnitude = sqrtf(local_surface->eigen_values[0]/local_surface->eigen_values.sum());
  //magnitude = sqrtf(local_surface->eigen_values_no_jumps[0]/local_surface->eigen_values_no_jumps.sum());

  //if (surface_structure_[y*range_image_->width+x+1]==NULL||surface_structure_[y*range_image_->width+x-1]==NULL)
  //{
    //magnitude = -std::numeric_limits<float>::infinity ();
    //return false;
  //}
  //float angle2 = acosf(surface_structure_[y*range_image_->width+x+1]->normal.dot(local_surface->normal)),
        //angle1 = acosf(surface_structure_[y*range_image_->width+x-1]->normal.dot(local_surface->normal));
  //magnitude = angle2-angle1;

  if (!pcl_isfinite(magnitude))
    return false;
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
RangeImageBorderExtractor::RangeImageBorderExtractor(const RangeImage* range_image) : BaseClass(),
  parameters_ (), range_image_(range_image), range_image_size_during_extraction_(0),
  border_scores_left_(NULL), border_scores_right_(NULL), border_scores_top_(NULL), border_scores_bottom_(NULL),
  surface_structure_(NULL), border_descriptions_(NULL), shadow_border_informations_(NULL), border_directions_(NULL),
  surface_change_scores_(NULL), surface_change_directions_(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
RangeImageBorderExtractor::~RangeImageBorderExtractor()
{
  clearData ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::setRangeImage (const RangeImage* range_image)
{
  clearData ();
  range_image_ = range_image;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::clearData ()
{
  delete[] border_scores_left_;    border_scores_left_   = NULL;
  delete[] border_scores_right_;   border_scores_right_  = NULL;
  delete[] border_scores_top_;     border_scores_top_    = NULL;
  delete[] border_scores_bottom_;  border_scores_bottom_ = NULL;
  //cout << PVARC(range_image_size_during_extraction_)<<PVARN((void*)this);
  for (int i=0; i<range_image_size_during_extraction_; ++i)
  {
    if (surface_structure_!=NULL)
      delete surface_structure_[i];
    if (shadow_border_informations_!=NULL)
      delete shadow_border_informations_[i];
    if (border_directions_!=NULL)
      delete border_directions_[i];
  }
  delete[] surface_structure_; surface_structure_ = NULL;
  delete border_descriptions_; border_descriptions_ = NULL;
  delete[] shadow_border_informations_; shadow_border_informations_ = NULL;
  delete[] border_directions_; border_directions_ = NULL;

  delete[] surface_change_scores_;  surface_change_scores_ = NULL;
  delete[] surface_change_directions_;  surface_change_directions_ = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::extractLocalSurfaceStructure ()
{
  if (surface_structure_ != NULL)
    return;
  //cerr << __PRETTY_FUNCTION__<<" called (this="<<(void*)this<<").\n";
  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height;
  range_image_size_during_extraction_ = width*height;
  int array_size = range_image_size_during_extraction_;
  surface_structure_ = new LocalSurface*[array_size];
  int step_size = (std::max)(1, parameters_.pixel_radius_plane_extraction/2);
  //cout << PVARN(step_size);
  int no_of_nearest_neighbors = static_cast<int> (pow (static_cast<double> (parameters_.pixel_radius_plane_extraction/step_size + 1), 2.0));

# pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(dynamic, 10)
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      LocalSurface*& local_surface = surface_structure_[index];
      local_surface = NULL;
      if (!range_image_->isValid(index))
        continue;
      local_surface = new LocalSurface;
      Eigen::Vector3f point;
      range_image_->getPoint(x, y, point);
      //cout << PVARN(point);
      if (!range_image_->getSurfaceInformation(x, y, parameters_.pixel_radius_plane_extraction, point,
                                  no_of_nearest_neighbors, step_size, local_surface->max_neighbor_distance_squared,
                                  local_surface->normal_no_jumps, local_surface->neighborhood_mean_no_jumps,
                                  local_surface->eigen_values_no_jumps,  &local_surface->normal,
                                  &local_surface->neighborhood_mean, &local_surface->eigen_values))
      {
        delete local_surface;
        local_surface = NULL;
      }

      //cout << x<<","<<y<<": ("<<local_surface->normal_no_jumps[0]<<","<<local_surface->normal_no_jumps[1]<<","<<local_surface->normal_no_jumps[2]<<")\n";
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::extractBorderScoreImages ()
{
  if (border_scores_left_ != NULL)
    return;

  extractLocalSurfaceStructure();

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  border_scores_left_   = new float[size];
  border_scores_right_  = new float[size];
  border_scores_top_    = new float[size];
  border_scores_bottom_ = new float[size];

# pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(dynamic, 10)
  for (int y=0; y<height; ++y) {
    for (int x=0; x<width; ++x) {
      int index = y*width + x;
      float& left=border_scores_left_[index]; float& right=border_scores_right_[index];
      float& top=border_scores_top_[index]; float& bottom=border_scores_bottom_[index];
      LocalSurface* local_surface_ptr = surface_structure_[index];
      if (local_surface_ptr==NULL)
      {
        left=right=top=bottom = 0.0f;
        continue;
      }

      left   = getNeighborDistanceChangeScore(*local_surface_ptr, x, y, -1,  0, parameters_.pixel_radius_borders);
      right  = getNeighborDistanceChangeScore(*local_surface_ptr, x, y,  1,  0, parameters_.pixel_radius_borders);
      top    = getNeighborDistanceChangeScore(*local_surface_ptr, x, y,  0, -1, parameters_.pixel_radius_borders);
      bottom = getNeighborDistanceChangeScore(*local_surface_ptr, x, y,  0,  1, parameters_.pixel_radius_borders);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
float*
RangeImageBorderExtractor::updatedScoresAccordingToNeighborValues (const float* border_scores) const
{
  float* new_scores = new float[range_image_->width*range_image_->height];
  float* new_scores_ptr = new_scores;
  for (int y=0; y < static_cast<int> (range_image_->height); ++y)
    for (int x=0; x < static_cast<int> (range_image_->width); ++x)
      *(new_scores_ptr++) = updatedScoreAccordingToNeighborValues(x, y, border_scores);
  return (new_scores);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::updateScoresAccordingToNeighborValues ()
{
  extractBorderScoreImages();

  //MEASURE_FUNCTION_TIME;

  float* left_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_left_);
  delete[] border_scores_left_;
  border_scores_left_ = left_with_propagated_neighbors;
  float* right_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_right_);
  delete[] border_scores_right_;
  border_scores_right_ = right_with_propagated_neighbors;
  float* top_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_top_);
  delete[] border_scores_top_;
  border_scores_top_ = top_with_propagated_neighbors;
  float* bottom_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_bottom_);
  delete[] border_scores_bottom_;
  border_scores_bottom_ = bottom_with_propagated_neighbors;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::findAndEvaluateShadowBorders ()
{
  if (shadow_border_informations_ != NULL)
    return;

  if (border_scores_left_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__<<": border score images not available!\n";
  }

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height;
  shadow_border_informations_ = new ShadowBorderIndices*[width*height];
  for (int y = 0; y < static_cast<int> (height); ++y)
  {
    for (int x = 0; x < static_cast<int> (width); ++x)
    {
      int index = y*width+x;
      ShadowBorderIndices*& shadow_border_indices = shadow_border_informations_[index];
      shadow_border_indices = NULL;
      int shadow_border_idx;

      if (changeScoreAccordingToShadowBorderValue(x, y, -1, 0, border_scores_left_, border_scores_right_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->left = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 1, 0, border_scores_right_, border_scores_left_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->right = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 0, -1, border_scores_top_, border_scores_bottom_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->top = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 0, 1, border_scores_bottom_, border_scores_top_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->bottom = shadow_border_idx;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
float*
RangeImageBorderExtractor::getAnglesImageForBorderDirections ()
{
  calculateBorderDirections();

  int width  = range_image_->width,
      height = range_image_->height,
      array_size = width*height;
  float* angles_image = new float[array_size];

  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& angle = angles_image[index];
      angle = -std::numeric_limits<float>::infinity ();
      const Eigen::Vector3f* border_direction_ptr = border_directions_[index];
      if (border_direction_ptr == NULL)
        continue;
      const Eigen::Vector3f& border_direction = *border_direction_ptr;
      const PointWithRange& point = range_image_->getPoint(index);

      float border_direction_in_image_x, border_direction_in_image_y;
      float tmp_factor = point.range*range_image_->getAngularResolution();
      range_image_->getImagePoint(point.x+tmp_factor*border_direction[0], point.y+tmp_factor*border_direction[1], point.z+tmp_factor*border_direction[2],
                                border_direction_in_image_x, border_direction_in_image_y);
      border_direction_in_image_x -= static_cast<float> (x);  border_direction_in_image_y -= static_cast<float> (y);
      angle = atan2f (border_direction_in_image_y, border_direction_in_image_x);
    }
  }
  return angles_image;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
float*
RangeImageBorderExtractor::getAnglesImageForSurfaceChangeDirections ()
{
  //MEASURE_FUNCTION_TIME;

  calculateSurfaceChanges();

  int width  = range_image_->width,
      height = range_image_->height,
      array_size = width*height;
  float* angles_image = new float[array_size];

  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& angle = angles_image[index];
      angle = -std::numeric_limits<float>::infinity ();
      float surface_change_score = surface_change_scores_[index];
      if (surface_change_score <= 0.1f)
        continue;
      const Eigen::Vector3f& direction = surface_change_directions_[index];
      const PointWithRange& point = range_image_->getPoint(index);

      float border_direction_in_image_x, border_direction_in_image_y;
      float tmp_factor = point.range*range_image_->getAngularResolution();
      range_image_->getImagePoint(point.x+tmp_factor*direction[0], point.y+tmp_factor*direction[1], point.z+tmp_factor*direction[2],
                                border_direction_in_image_x, border_direction_in_image_y);
      border_direction_in_image_x -= static_cast<float> (x);  border_direction_in_image_y -= static_cast<float> (y);
      angle = atan2f (border_direction_in_image_y, border_direction_in_image_x);
      if (angle <= deg2rad (-90.0f))
        angle += static_cast<float> (M_PI);
      else if (angle > deg2rad (90.0f))
        angle -= static_cast<float> (M_PI);
    }
  }
  return (angles_image);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::classifyBorders ()
{
  if (border_descriptions_ != NULL)
    return;

  // Get local plane approximations
  extractLocalSurfaceStructure();

  // Get scores for every point, describing how probable a border in that direction is
  extractBorderScoreImages();

  // Propagate values to neighboring pixels
  updateScoresAccordingToNeighborValues();

  // Change border score according to the existence of a shadow border
  findAndEvaluateShadowBorders();

  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;

  BorderDescription initial_border_description;
  initial_border_description.traits = 0;
  border_descriptions_ = new PointCloudOut;
  border_descriptions_->width = width;
  border_descriptions_->height = height;
  border_descriptions_->is_dense = true;
  border_descriptions_->points.resize(size, initial_border_description);

  for (int y = 0; y < static_cast<int> (height); ++y)
  {
    for (int x = 0; x < static_cast<int> (width); ++x)
    {
      int index = y*width+x;
      BorderDescription& border_description = border_descriptions_->points[index];
      border_description.x = x;
      border_description.y = y;
      BorderTraits& border_traits = border_description.traits;

      ShadowBorderIndices* shadow_border_indices = shadow_border_informations_[index];
      if (shadow_border_indices == NULL)
        continue;

      int shadow_border_index = shadow_border_indices->left;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, -1, 0, border_scores_left_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_RIGHT] = true;
        for (int index3=index-1; index3>shadow_border_index; --index3)
        {
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_RIGHT] = true;
        }
      }

      shadow_border_index = shadow_border_indices->right;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 1, 0, border_scores_right_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_LEFT] = true;
        for (int index3=index+1; index3<shadow_border_index; ++index3)
        {
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_LEFT] = true;
        }
      }

      shadow_border_index = shadow_border_indices->top;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 0, -1, border_scores_top_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_BOTTOM] = true;
        for (int index3=index-width; index3>shadow_border_index; index3-=width)
        {
          //cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_BOTTOM] = true;
        }
      }

      shadow_border_index = shadow_border_indices->bottom;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 0, 1, border_scores_bottom_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_TOP] = true;
        for (int index3=index+width; index3<shadow_border_index; index3+=width)
        {
          //cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_TOP] = true;
        }
      }

      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
      //{
        //border_points.push_back(&border_description);
      //}
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::calculateBorderDirections ()
{
  if (border_directions_!=NULL)
    return;
  classifyBorders();

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  border_directions_ = new Eigen::Vector3f*[size];
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      calculateBorderDirection(x, y);
    }
  }

  Eigen::Vector3f** average_border_directions = new Eigen::Vector3f*[size];
  int radius = parameters_.pixel_radius_border_direction;
  int minimum_weight = radius+1;
  float min_cos_angle=cosf(deg2rad(120.0f));
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      Eigen::Vector3f*& average_border_direction = average_border_directions[index];
      average_border_direction = NULL;
      const Eigen::Vector3f* border_direction = border_directions_[index];
      if (border_direction==NULL)
        continue;
      average_border_direction = new Eigen::Vector3f(*border_direction);
      float weight_sum = 1.0f;
      for (int y2=(std::max)(0, y-radius); y2<=(std::min)(y+radius, height-1); ++y2)
      {
        for (int x2=(std::max)(0, x-radius); x2<=(std::min)(x+radius, width-1); ++x2)
        {
          int index2 = y2*width + x2;
          const Eigen::Vector3f* neighbor_border_direction = border_directions_[index2];
          if (neighbor_border_direction==NULL || index2==index)
            continue;

          // Oposite directions?
          float cos_angle = neighbor_border_direction->dot(*border_direction);
          if (cos_angle<min_cos_angle)
          {
            //cout << "Reject. "<<PVARC(min_cos_angle)<<PVARC(cos_angle)<<PVARAN(acosf(cos_angle));
            continue;
          }
          //else
            //cout << "No reject\n";

          // Border in between?
          float border_between_points_score = getNeighborDistanceChangeScore(*surface_structure_[index], x, y, x2-x,  y2-y, 1);
          if (fabsf(border_between_points_score) >= 0.95f*parameters_.minimum_border_probability)
            continue;

          *average_border_direction += *neighbor_border_direction;
          weight_sum += 1.0f;
        }
      }
      if (pcl_lrint (weight_sum) < minimum_weight)
      {
        delete average_border_direction;
        average_border_direction=NULL;
      }
      else
        average_border_direction->normalize();
    }
  }

  for (int i=0; i<size; ++i)
    delete border_directions_[i];
  delete[] border_directions_;
  border_directions_ = average_border_directions;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::calculateSurfaceChanges ()
{
  if (surface_change_scores_!=NULL)
    return;

  calculateBorderDirections();

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  surface_change_scores_ = new float[size];
  surface_change_directions_ = new Eigen::Vector3f[size];
# pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(dynamic, 10)
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& surface_change_score = surface_change_scores_[index];
      surface_change_score = 0.0f;
      Eigen::Vector3f& surface_change_direction = surface_change_directions_[index];
      surface_change_direction.setZero();

      const BorderTraits& border_traits = border_descriptions_->points[index].traits;
      if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        continue;
      if (border_directions_[index]!=NULL)
      {
        surface_change_score = 1.0f;
        surface_change_direction = *border_directions_[index];
      }
      else
      {
        if (!calculateMainPrincipalCurvature(x, y, parameters_.pixel_radius_principal_curvature,
                                             surface_change_score, surface_change_direction))
        {
          surface_change_score = 0.0f;
          continue;
        }
      }
    }
  }
  //blurSurfaceChanges();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::blurSurfaceChanges ()
{
  int blur_radius = 1;
  if (blur_radius==0)
    return;

  const RangeImage& range_image = *range_image_;

  Eigen::Vector3f* blurred_directions = new Eigen::Vector3f[range_image.width*range_image.height];
  float* blurred_scores = new float[range_image.width*range_image.height];
  for (int y=0; y<int(range_image.height); ++y)
  {
    for (int x=0; x<int(range_image.width); ++x)
    {
      int index = y*range_image.width + x;
      Eigen::Vector3f& new_point = blurred_directions[index];
      new_point.setZero();
      float& new_score = blurred_scores[index];
      new_score = 0.0f;
      if (!range_image.isValid(index))
        continue;
      const BorderTraits& border_traits = border_descriptions_->points[index].traits;
      if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        continue;
      const Eigen::Vector3f& point = surface_change_directions_[index];
      float counter = 0.0f;
      for (int y2=y-blur_radius; y2<y+blur_radius; ++y2)
      {
        for (int x2=x-blur_radius; x2<x+blur_radius; ++x2)
        {
          if (!range_image.isInImage(x2,y2))
            continue;
          int index2 = y2*range_image.width + x2;
          float score = surface_change_scores_[index2];
          //if (score == 0.0f)
            //continue;
            //
          if (score > 0.0f)
          {
            Eigen::Vector3f& neighbor = surface_change_directions_[index2];
            //if (fabs(neighbor.norm()-1) > 1e-4)
              //cerr<<PVARN(neighbor)<<PVARN(score);
            if (point.dot(neighbor)<0.0f)
              neighbor *= -1.0f;
            new_point += score*neighbor;
          }
          new_score += score;
          counter += 1.0f;
        }
      }
      new_point.normalize();
      if (counter > 0.0f)
        new_score /= counter;
    }
  }
  delete[] surface_change_directions_;
  surface_change_directions_ = blurred_directions;
  delete[] surface_change_scores_;
  surface_change_scores_ = blurred_scores;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::computeFeature (PointCloudOut& output)
{
  output.points.clear();

  if (indices_)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": Sorry, usage of indices for the extraction is not supported for range image border extraction (yet).\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  if (range_image_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the border extraction works on range images, not on normal point clouds."
              << " Use setRangeImage(...).\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  output = getBorderDescriptions ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::compute (PointCloudOut& output)
{
  computeFeature (output);
}

}  // namespace end
