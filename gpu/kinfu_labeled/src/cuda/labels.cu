/*
 * labels.cu
 *
 *  Created on: Aug 23, 2018
 *      Author: Carlos M. Mateo
 */

#include <cuda/device.hpp>

namespace pcl
{
  namespace device
  {
    __global__ void
    initLabelVolumeKernel (PtrStep<uint1> volume)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x < VOLUME_X && y < VOLUME_Y)
      {
        uint1 *pos = volume.ptr (y) + x;
        int z_step = VOLUME_Y * volume.step / sizeof(*pos);

		#pragma unroll
        for (int z = 0; z < VOLUME_Z; ++z, pos += z_step)
          *pos = make_uint1 (0);
      }
    }
  }
}

void
pcl::device::initLabelVolume (PtrStep<uint1> label_volume)
{
  dim3 block (32, 16);
  dim3 grid (1, 1, 1);
  grid.x = divUp (VOLUME_X, block.x);
  grid.y = divUp (VOLUME_Y, block.y);

  initLabelVolumeKernel<<<grid, block>>>(label_volume);
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
}

namespace pcl
{
  namespace device
  {
    __global__ void
    extractLabelsKernel (const PtrStep<uint1> label_volume, const float3 cell_size, const PtrSz<PointType> points, uint1 *labels)
    {
      int idx = threadIdx.x + blockIdx.x * blockDim.x;

      if (idx < points.size)
      {
        int3 v;
        float3 p = *(const float3*)(points.data + idx);
        v.x = __float2int_rd (p.x / cell_size.x);        // round to negative infinity
        v.y = __float2int_rd (p.y / cell_size.y);
        v.z = __float2int_rd (p.z / cell_size.z);

        uint1 label = label_volume.ptr (VOLUME_Y * v.z + v.y)[v.x];
        labels[idx] = make_uint1 (label.x);
      }
    }
  }
}

void
pcl::device::exctractLabels (const PtrStep<uint1>& label_volume, const float3& volume_size, const PtrSz<PointType>& points, uint1* labels)
{
  const int block = 256;
  float3 cell_size = make_float3 (volume_size.x / VOLUME_X, volume_size.y / VOLUME_Y, volume_size.z / VOLUME_Z);
  extractLabelsKernel<<<divUp (points.size, block), block>>>(label_volume, cell_size, points, labels);
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
};

namespace pcl
{
  namespace device
  {
    struct LabelVolumeImpl
    {
      enum
      {
        CTA_SIZE_X = 32,
        CTA_SIZE_Y = 8
      };

      Mat33 R_inv;
      float3 t;

      float3 cell_size;

      mutable PtrStep<uint1> label_volume;

      float3 pt1;
      float3 pt2;
      float3 dpt21;
      float3 dpt12;
      float lengthsq;
      float radius_sq;

      __device__ __forceinline__ float
      cylinderTest( float3 pt0 ) const
      {
        float3 dpt01 = pt0 - pt1;
        float3 dpt10 = pt1 - pt0;

        // Dot the d and pd vectors to see if point lies behind the
        // cylinder cap at pt1.x, pt1.y, pt1.z
        float d_l2 = dot(dpt01, dpt12);
        float d_l1 = dot(dpt10, dpt21);

        if(  d_l2 > lengthsq || d_l1 > lengthsq)
          return -1.0f;
        else
        {
          // radial distance to mayor axis
          float d = norm(cross(dpt21, dpt10)) / norm(dpt21);

          if( d > radius_sq )
            return -1.0f;
          else
            return d;
        }
      }

      __device__ __forceinline__ int3
      getVoxel (float3 point) const
      {
        int vx = __float2int_rd (point.x / cell_size.x);                // round to negative infinity
        int vy = __float2int_rd (point.y / cell_size.y);
        int vz = __float2int_rd (point.z / cell_size.z);

        return make_int3 (vx, vy, vz);
      }

      __device__ __forceinline__ float3
      getVoxelGCoo (int x, int y, int z) const
      {
        float3 coo = make_float3 (x, y, z);
        coo += 0.5f;                 //shift to cell center;

        coo.x *= cell_size.x;
        coo.y *= cell_size.y;
        coo.z *= cell_size.z;

        return coo;
      }

      __device__ __forceinline__ void
      operator () () const
      {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= VOLUME_X || y >= VOLUME_Y)
          return;

        uint1 *pos = label_volume.ptr (y) + x;
        int elem_step = label_volume.step * VOLUME_Z / sizeof(*pos);

        for (int z = 0; z < VOLUME_Z; ++z,  pos += elem_step)
        {
          float3 v_g = getVoxelGCoo (x, y, z);
//           float3 v = R_inv * (v_g - t); // with respect to camera frame
          float3 v = v_g ; // with respect to volume frame

//          if (v.z <= 0)
//            continue;

          unsigned int label = 0 ;
          if (cylinderTest( v ) != -1.f)
            label = 1;

          *pos =  make_uint1 ( label );
        }         /* for(int z = 0; z < VOLUME_X; ++z) */
      }       /* void operator() */
    };

    __global__ void
    updateLabelVolumeKernel (const LabelVolumeImpl cvi) {
      cvi ();
    }
  }
}

void
pcl::device::updateLabelVolume (const Mat33& R_inv, const float3& t, const float3& pt1, const float3& pt2, const float& lengthsq, const float& radius_sq,
                                const float3& volume_size, PtrStep<uint1> label_volume)
{
  LabelVolumeImpl lvi;
  lvi.label_volume = label_volume;

  lvi.R_inv = R_inv;
  lvi.t = t;

  lvi.cell_size.x = volume_size.x / VOLUME_X;
  lvi.cell_size.y = volume_size.y / VOLUME_Y;
  lvi.cell_size.z = volume_size.z / VOLUME_Z;

  lvi.pt1 = pt1;
  lvi.pt2 = pt2;
  lvi.dpt21 = pt2 - pt1;
  lvi.dpt12 = pt1 - pt2;
  lvi.lengthsq = lengthsq;
  lvi.radius_sq = radius_sq;

  dim3 block (LabelVolumeImpl::CTA_SIZE_X, LabelVolumeImpl::CTA_SIZE_Y);
  dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

  updateLabelVolumeKernel<<<grid, block>>>(lvi);
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
}
