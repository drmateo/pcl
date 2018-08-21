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

#include "device.hpp"

using namespace pcl::device;

namespace pcl
{
namespace device
{
struct ImageGenerator
{
	enum
	{
		CTA_SIZE_X = 32, CTA_SIZE_Y = 8
	};

	PtrStep<float> vmap;
	PtrStep<float> nmap;

	LightSource light;

	mutable PtrStepSz<uchar3> dst;

	__device__ __forceinline__ void
	operator () () const
	{
		int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
		int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

		if (x >= dst.cols || y >= dst.rows)
			return;

		float3 v, n;
		v.x = vmap.ptr (y)[x];
		n.x = nmap.ptr (y)[x];

		uchar3 color = make_uchar3 (0, 0, 0);

		if (!isnan (v.x) && !isnan (n.x))
		{
			v.y = vmap.ptr (y + dst.rows)[x];
			v.z = vmap.ptr (y + 2 * dst.rows)[x];

			n.y = nmap.ptr (y + dst.rows)[x];
			n.z = nmap.ptr (y + 2 * dst.rows)[x];

			float weight = 1.f;

			for (int i = 0; i < light.number; ++i)
			{
				float3 vec = normalized (light.pos[i] - v);

				weight *= fabs (dot (vec, n));
			}

			int br = (int)(205 * weight) + 50;
			br = max (0, min (255, br));
			color = make_uchar3 (br, br, br);
		}
		dst.ptr (y)[x] = color;
	}
};

__global__ void
generateImageKernel (const ImageGenerator ig) {
	ig ();
}
}
}


void
pcl::device::generateImage (const MapArr& vmap, const MapArr& nmap, const LightSource& light, 
		PtrStepSz<uchar3> dst)
{
	ImageGenerator ig;
	ig.vmap = vmap;
	ig.nmap = nmap;
	ig.light = light;
	ig.dst = dst;

	dim3 block (ImageGenerator::CTA_SIZE_X, ImageGenerator::CTA_SIZE_Y);
	dim3 grid (divUp (dst.cols, block.x), divUp (dst.rows, block.y));

	generateImageKernel<<<grid, block>>>(ig);
	cudaSafeCall (cudaGetLastError ());
	cudaSafeCall (cudaDeviceSynchronize ());
} 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
namespace device
{
__global__ void generateDepthKernel(const float3 R_inv_row3, const float3 t, const PtrStep<float> vmap, PtrStepSz<unsigned short> depth)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < depth.cols && y < depth.rows)
	{
		unsigned short result = 0;

		float3 v_g;
		v_g.x = vmap.ptr (y)[x];
		if (!isnan (v_g.x))
		{
			v_g.y = vmap.ptr (y +     depth.rows)[x];
			v_g.z = vmap.ptr (y + 2 * depth.rows)[x];

			float v_z = dot(R_inv_row3, v_g - t);

			result = static_cast<unsigned short>(v_z * 1000);
		}
		depth.ptr(y)[x] = result;
	}
}
}
}

void
pcl::device::generateDepth (const Mat33& R_inv, const float3& t, const MapArr& vmap, DepthMap& dst)
{
	dim3 block(32, 8);
	dim3 grid(divUp(dst.cols(), block.x), divUp(dst.rows(), block.y));

	generateDepthKernel<<<grid, block>>>(R_inv.data[2], t, vmap, dst);
	cudaSafeCall (cudaGetLastError ());
	cudaSafeCall (cudaDeviceSynchronize ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
namespace device
{
struct GenerateDepth
{
	Mat33 R_inv;
	float3 R_inv_row3;

	float3 t;
	PtrStep<float> vmap;
	mutable PtrStepSz<unsigned short> depth;



	Intr intr;

	Mat33 RBase;
	float3 tBase;



	float z_min ;
	float z_max ;

	__device__ __forceinline__ void
	operator () () const
	{
		int x = threadIdx.x + blockIdx.x * blockDim.x;
		int y = threadIdx.y + blockIdx.y * blockDim.y;

		if (x < depth.cols && y < depth.rows)
		{
			unsigned short result = 0;
			float v_z;
			float3 v_g;
			v_g.x = vmap.ptr (y)[x];
			if (!isnan (v_g.x))
			{
				v_g.y = vmap.ptr (y +     depth.rows)[x];
				v_g.z = vmap.ptr (y + 2 * depth.rows)[x];

				v_z = dot(R_inv_row3, v_g - t);

				result = static_cast<unsigned short>(v_z * 1000);
			}

			if (result != 0)
			{
				float3 pt ;

				pt.z = v_z * 1000.f;
				pt.x = ((float)x - intr.cx) * pt.z / intr.fx;
				pt.y = ((float)y - intr.cy) * pt.z / intr.fy;

				pt *= 0.001f;

				//            				pt.x = v_g.x ;
				//            				pt.y = v_g.y ;
				//            				pt.z = v_z ;

				//            				float3 pt_t = R_inv * (pt - t);
				float3 pt_t = RBase * (pt - tBase);


				if (pt_t.z > z_max || pt_t.z < z_min)
					result = 0;
			}

			depth.ptr(y)[x] = result;
		}
	}       /* operator() */

};

struct GenerateDepthV2 : GenerateDepth
{
	float3 pt1;
	float3 pt2;
	float lengthsq;
	float radius_sq;


	__device__ __forceinline__ float
	CylTest( float3 testpt ) const
	{
		float dx, dy, dz;	// vector d  from line segment point 1 to point 2
		float pdx, pdy, pdz;	// vector pd from point 1 to test point
		float dot, dsq;

		dx = pt2.x - pt1.x;	// translate so pt1 is origin.  Make vector from
		dy = pt2.y - pt1.y;     // pt1 to pt2.  Need for this is easily eliminated
		dz = pt2.z - pt1.z;

		pdx = testpt.x - pt1.x;		// vector from pt1 to test point.
		pdy = testpt.y - pt1.y;
		pdz = testpt.z - pt1.z;

		dot = pdx * dx + pdy * dy + pdz * dz;

		if( dot < 0.0f || dot > lengthsq )
		{
			return( -1.0f );
		}
		else
		{
			dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

			if( dsq > radius_sq )
			{
				return( -1.0f );
			}
			else
			{
				return( dsq );		// return distance squared to axis
			}
		}
	}

	__device__ __forceinline__ void
	operator () () const
	{
		int x = threadIdx.x + blockIdx.x * blockDim.x;
		int y = threadIdx.y + blockIdx.y * blockDim.y;

		if (x < depth.cols && y < depth.rows)
		{
			unsigned short result = 0;
			float v_z;
			float3 v_g;
			v_g.x = vmap.ptr (y)[x];
			if (!isnan (v_g.x))
			{
				v_g.y = vmap.ptr (y +     depth.rows)[x];
				v_g.z = vmap.ptr (y + 2 * depth.rows)[x];

				v_z = dot(R_inv_row3, v_g - t);

				result = static_cast<unsigned short>(v_z * 1000);
			}

			if (result != 0)
			{
				float3 pt ;

				pt.z = v_z * 1000.f;
				pt.x = ((float)x - intr.cx) * pt.z / intr.fx;
				pt.y = ((float)y - intr.cy) * pt.z / intr.fy;

				pt *= 0.001f;

				//            				pt.x = v_g.x ;
				//            				pt.y = v_g.y ;
				//            				pt.z = v_z ;

				//            				float3 pt_t = R_inv * (pt - t);
				float3 pt_t = RBase * pt + tBase;


				if (CylTest( pt_t ) != -1.f || pt_t.z > z_max || pt_t.z < z_min)
					result = 0;
			}

			depth.ptr(y)[x] = result;
		}
	}       /* operator() */
};

__global__ void
generateDepthKernel (const GenerateDepth gd) {
	gd ();
}

__global__ void
generateDepthKernel (const GenerateDepthV2 gd) {
	gd ();
}
}
}

void
pcl::device::generateDepth (const Intr& intr, const Mat33& R_inv, const float3& t, const Mat33& R_base, const float3& t_base, float z_min, float z_max, const MapArr& vmap, DepthMap& dst)
{
	dim3 block(32, 8);
	dim3 grid(divUp(dst.cols(), block.x), divUp(dst.rows(), block.y));

	GenerateDepth gd;
	gd.R_inv = R_inv;
	gd.R_inv_row3 = R_inv.data[2];
	gd.t = t;
	gd.RBase = R_base;
	gd.tBase = t_base;
	gd.vmap = vmap;
	gd.depth = dst;
	gd.intr = intr;
	gd.z_min = z_min;
	gd.z_max = z_max;

	generateDepthKernel<<<grid, block>>>(gd);
	cudaSafeCall (cudaGetLastError ());
	cudaSafeCall (cudaDeviceSynchronize ());
}

void
pcl::device::generateDepth (const Intr& intr, const Mat33& R_inv, const float3& t, const Mat33& R_base, const float3& t_base, float z_min, float z_max,
		   const float3& pt1, const float3& pt2, float lengthsq, float radius_sq,  const MapArr& vmap, DepthMap& dst)
{
	dim3 block(32, 8);
	dim3 grid(divUp(dst.cols(), block.x), divUp(dst.rows(), block.y));

	GenerateDepthV2 gd;
	gd.R_inv = R_inv;
	gd.R_inv_row3 = R_inv.data[2];
	gd.t = t;
	gd.RBase = R_base;
	gd.tBase = t_base;
	gd.vmap = vmap;
	gd.depth = dst;
	gd.intr = intr;
	gd.z_min = z_min;
	gd.z_max = z_max;


	gd.pt1 = pt1;
	gd.pt2 = pt2;
	gd.lengthsq = lengthsq;
	gd.radius_sq = radius_sq;

	generateDepthKernel<<<grid, block>>>(gd);
	cudaSafeCall (cudaGetLastError ());
	cudaSafeCall (cudaDeviceSynchronize ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
namespace device
{
__global__ void
paint3DViewKernel(const PtrStep<uchar3> colors, PtrStepSz<uchar3> dst, float colors_weight)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < dst.cols && y < dst.rows)
	{
		uchar3 value = dst.ptr(y)[x];
		uchar3 color = colors.ptr(y)[x];

		if (value.x != 0 || value.y != 0 || value.z != 0)
		{
			float cx = value.x * (1.f - colors_weight) + color.x * colors_weight;
			float cy = value.y * (1.f - colors_weight) + color.y * colors_weight;
			float cz = value.z * (1.f - colors_weight) + color.z * colors_weight;

			value.x = min(255, max(0, __float2int_rn(cx)));
			value.y = min(255, max(0, __float2int_rn(cy)));
			value.z = min(255, max(0, __float2int_rn(cz)));
		}

		dst.ptr(y)[x] = value;
	}
}
}
}

void 
pcl::device::paint3DView(const PtrStep<uchar3>& colors, PtrStepSz<uchar3> dst, float colors_weight)
{
	dim3 block(32, 8);
	dim3 grid(divUp(dst.cols, block.x), divUp(dst.rows, block.y));

	colors_weight = min(1.f, max(0.f, colors_weight));

	paint3DViewKernel<<<grid, block>>>(colors, dst, colors_weight);
	cudaSafeCall (cudaGetLastError ());
	cudaSafeCall (cudaDeviceSynchronize ());
}
