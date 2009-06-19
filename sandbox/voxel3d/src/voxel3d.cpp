/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include "voxel3d/voxel3d.h"

#define USE_SSE 1

//#if USE_SSE
#include <emmintrin.h>
#include <xmmintrin.h>
//#endif

#include <cmath>
#include <malloc.h>

#include <string.h> // for memset(3)

const unsigned char Voxel3d::CLEAR = 0xff;

Voxel3d::Voxel3d(int size1, int size2, int size3)
  : size1_(size1), size2_(size2), size3_(size3),
    stride1_(size1_), stride2_(size1_*size2_)
{
  data_.resize(size1_*size2_*size3_);
  reset();

  // Constructs the kernel
  //kernel_.resize(16*17*17);
  kernel_ = (unsigned char*)memalign(16, 16*17*17);
  for (int k = 0; k < 17; ++k) {
    for (int j = 0; j < 17; ++j) {
      for (int i = 0; i < 16; ++i) {
        kernel_[i + 16*(j + 17*k)] =
          (char)round(sqrt(pow(i-8,2) + pow(j-8,2) + pow(k-8,2)));
      }
    }
  }
}

Voxel3d::~Voxel3d()
{
  //delete [] kernel_;
  free(kernel_);
}


void Voxel3d::reset()
{
  memset(&data_[0], CLEAR, data_.size());
}

void Voxel3d::putObstacle(int i, int j, int k)
{
  // Doesn't do points near the edges
  if (i < 8 || i >= size1_ - 8 ||
      j < 8 || j >= size2_ - 8 ||
      k < 8 || k >= size3_ - 8)
    return;

  if ((*this)(i,j,k) == 0)
    return;

  // (i,j,k) corresponds to (8,8,8) in the kernel
  unsigned char *p = &data_[ref(i-8,j-8,k-8)];
  unsigned char *r = &kernel_[0];
  for (int kk = 0; kk < 17; ++kk)
  {
    unsigned char *jj_start = p;

#if USE_SSE
    for (int jj = 0; jj < 17; ++jj)
    {
      unsigned char *ii_start = p;
      __m128i vp, vr;
      vp = _mm_loadu_si128((__m128i*)p);
      vr = _mm_load_si128((__m128i*)r);
      _mm_storeu_si128((__m128i*)p, _mm_min_epu8(vp, vr));
      r += 16;
      p = ii_start + stride1_;
    }
//     for (int jj = 0; jj < 16; jj += 2)
//     {
//       __m128i vp1, vr1, vp2, vr2;
//       vp1 = _mm_loadu_si128((__m128i*)p);
//       vr1 = _mm_load_si128((__m128i*)r);
//       vp2 = _mm_loadu_si128((__m128i*)(p+stride1_));
//       vr2 = _mm_load_si128((__m128i*)(r+16));
//       vp1 = _mm_min_epu8(vp1, vr1);
//       vp2 = _mm_min_epu8(vp2, vr2);
//       _mm_storeu_si128((__m128i*)p, vp1);
//       _mm_storeu_si128((__m128i*)(p+stride1_), vp2);

//       r += 32;
//       p += 2*stride1_;// + stride1_;
//     }
//     INCOMPLETE!!!
#else
    for (int jj = 0; jj < 17; ++jj)
    {
      unsigned char *ii_start = p;
      for (int ii = 0; ii < 16; ++ii)
      {
        if (*p > *r)
          *p = *r;
        ++r;
        ++p;
      }
      p = ii_start + stride1_;
    }
#endif

    p = jj_start + stride2_;
  }
}

