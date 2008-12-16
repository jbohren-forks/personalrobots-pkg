/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


#include "mcpdf_pos_vel.h"
#include <assert.h>
#include <vector>


  using namespace MatrixWrapper;
  using namespace BFL;
  using namespace tf;
  
  static const unsigned int NUM_CONDARG   = 1;


  MCPdfPosVel::MCPdfPosVel (unsigned int num_samples) 
    : MCPdf<StatePosVel> ( num_samples, NUM_CONDARG )
  {}

  MCPdfPosVel::~MCPdfPosVel(){}


  WeightedSample<StatePosVel>
  MCPdfPosVel::SampleGet(unsigned int particle) const
  {
    assert ((int)particle >= 0 && particle < _listOfSamples.size());
    return _listOfSamples[particle];
  }


  /// Get histogram from pos
  MatrixWrapper::Matrix MCPdfPosVel::getHistogramPos(const Vector3& m, const Vector3& M, const Vector3& step) const
  { 
    return getHistogram(m, M, step, true);
  }


  /// Get histogram from vel
  MatrixWrapper::Matrix MCPdfPosVel::getHistogramVel(const Vector3& m, const Vector3& M, const Vector3& step) const
  { 
    return getHistogram(m, M, step, false);
  }


  /// Get histogram from certain area
MatrixWrapper::Matrix MCPdfPosVel::getHistogram(const Vector3& m, const Vector3& M, const Vector3& step, bool pos_hist) const
  {  
    unsigned int num_smaples = _listOfSamples.size();
    unsigned int rows = trunc((M[0]-m[0])/step[0]);
    unsigned int cols = trunc((M[1]-m[1])/step[1]);
    Matrix hist(rows, cols);
    hist = 0;

    // calculate histogram
    for (unsigned int i=0; i<num_smaples; i++){
      Vector3 rel;
      if (pos_hist)
	rel = _listOfSamples[i].ValueGet().pos_ - m;
      else
	rel = _listOfSamples[i].ValueGet().vel_ - m;

      unsigned int r = trunc(rel[0] * rows / (M[0] - m[0]));
      unsigned int c = trunc(rel[1] * cols / (M[1] - m[0]));
      if (r >= 1 && c >= 1 && r <= rows && c <= cols)
	hist(r,c) += _listOfSamples[i].WeightGet();
    }

    return hist;
  }



  unsigned int
  MCPdfPosVel::numParticlesGet() const
  {
    return _listOfSamples.size();
  }
  

