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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#ifndef PCL_GPU_KINFU_CUDA_UTILS_HPP_
#define PCL_GPU_KINFU_CUDA_UTILS_HPP_

#include <pcl/gpu/utils/device/block.hpp>

namespace pcl
{
  namespace device
  {   
    struct Emulation
	{
      static __device__ __forceinline__ int
      warp_reduce ( volatile int *ptr , const unsigned int tid)
      {
        const unsigned int lane = tid & 31; // index of thread in warp (0..31)

        if (lane < 16)
        {
          int partial = ptr[tid];

          ptr[tid] = partial = partial + ptr[tid + 16];
          ptr[tid] = partial = partial + ptr[tid + 8];
          ptr[tid] = partial = partial + ptr[tid + 4];
          ptr[tid] = partial = partial + ptr[tid + 2];
          ptr[tid] = partial = partial + ptr[tid + 1];
        }
        return ptr[tid - lane];
      }

	  static __forceinline__ __device__ int
      Ballot(int predicate, volatile int* cta_buffer)
	  {
#if __CUDA_ARCH__ >= 200
	    (void)cta_buffer;
		return __ballot(predicate);
#else
        int tid = Block::flattenedThreadId();
		cta_buffer[tid] = predicate ? (1 << (tid & 31)) : 0;
		return warp_reduce(cta_buffer, tid);
#endif
      }

      static __forceinline__ __device__ bool
      All(int predicate, volatile int* cta_buffer)
      {
#if __CUDA_ARCH__ >= 200
	    (void)cta_buffer;
		return __all(predicate);
#else
        int tid = Block::flattenedThreadId();
		cta_buffer[tid] = predicate ? 1 : 0;
        return warp_reduce(cta_buffer, tid) == 32;
#endif
      }
    };
  }
}

#endif /* PCL_GPU_KINFU_CUDA_UTILS_HPP_ */
