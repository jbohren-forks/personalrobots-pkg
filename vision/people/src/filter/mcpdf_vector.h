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
*   * Redistributions of source code must retain the above copyright
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

/* Author: Wim Meeussen */

#ifndef MCPDF_VECTOR_H
#define MCPDF_VECTOR_H

#include <pdf/mcpdf.h>
#include "state_vector.h"
#include <tf/tf.h>
#include <std_msgs/PointCloud.h>

namespace BFL
{
  /// Class representing a vector mcpdf
  class MCPdfVector: public MCPdf<StateVector>
    {
    public:
      /// Constructor
      MCPdfVector (unsigned int num_samples);

      /// Destructor
      virtual ~MCPdfVector();

      /// Get evenly distributed particle cloud
      void getParticleCloud(const StateVector& step, double threshold, std_msgs::PointCloud& cloud) const;

      /// Get pos histogram from certain area
      MatrixWrapper::Matrix getHistogramPos(const StateVector& min, const StateVector& max, const StateVector& step) const;

      virtual StateVector ExpectedValueGet() const;
      virtual WeightedSample<StateVector> SampleGet(unsigned int particle) const;
      virtual unsigned int numParticlesGet() const;

    private:
      /// Get histogram from certain area
      MatrixWrapper::Matrix getHistogram(const StateVector& min, const StateVector& max, const StateVector& step) const;

    };



} // end namespace
#endif
