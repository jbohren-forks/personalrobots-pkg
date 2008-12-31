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

#include "mcpdf_vector.h"
#include <assert.h>
#include <vector>
#include <std_msgs/Float64.h>
#include "rgb.h"


  using namespace MatrixWrapper;
  using namespace BFL;
  using namespace tf;
  
  static const unsigned int NUM_CONDARG   = 1;


  MCPdfVector::MCPdfVector (unsigned int num_samples) 
    : MCPdf<StateVector> ( num_samples, NUM_CONDARG )
  {}

  MCPdfVector::~MCPdfVector(){}


  WeightedSample<StateVector>
  MCPdfVector::SampleGet(unsigned int particle) const
  {
    assert ((int)particle >= 0 && particle < _listOfSamples.size());
    return _listOfSamples[particle];
  }


  StateVector MCPdfVector::ExpectedValueGet() const
  {
    StateVector pos(0,0,0); 
    double current_weight;
    std::vector<WeightedSample<StateVector> >::const_iterator it_los;
    for ( it_los = _listOfSamples.begin() ; it_los != _listOfSamples.end() ; it_los++ ){
      current_weight = it_los->WeightGet();
      pos += (it_los->ValueGet() * current_weight);
    }
    return StateVector(pos);
  }


  /// Get evenly distributed particle cloud
  void MCPdfVector::getParticleCloud(const StateVector& step, double threshold, std_msgs::PointCloud& cloud) const
  { 
    unsigned int num_samples = _listOfSamples.size();
    assert(num_samples > 0);
    StateVector m = _listOfSamples[0].ValueGet();
    StateVector M = _listOfSamples[0].ValueGet();

    // calculate min and max
    for (unsigned int s=0; s<num_samples; s++){
      StateVector v = _listOfSamples[s].ValueGet();
      for (unsigned int i=0; i<3; i++){
	if (v[i] < m[i]) m[i] = v[i];
	if (v[i] > M[i]) M[i] = v[i];
      }
    }

    // get point cloud from histogram
    Matrix hist = getHistogramPos(m, M, step);
    unsigned int row = hist.rows();
    unsigned int col = hist.columns();
    unsigned int total = 0;
    unsigned int t = 0;
    for (unsigned int r=1; r<= row; r++)
      for (unsigned int c=1; c<= col; c++)
	if (hist(r,c) > threshold) total++;

    vector<std_msgs::Point32> points(total);
    vector<float> weights(total);
    std_msgs::ChannelFloat32 channel;
    for (unsigned int r=1; r<= row; r++)
      for (unsigned int c=1; c<= col; c++)
	if (hist(r,c) > threshold){
	  for (unsigned int i=0; i<3; i++)
	  points[t].x = m[0] + (step[0] * r);
	  points[t].y = m[1] + (step[1] * c);
	  points[t].z = m[2];
	  weights[t] = rgb[999-(int)trunc(max(0.0,min(999.0,hist(r,c)*2*total*total)))];
	  t++;
	}
    cloud.header.frame_id = "odom";
    cloud.pts  = points;
    channel.name = "rgb";
    channel.vals = weights;
    cloud.chan.push_back(channel);
  }


  /// Get histogram from pos
  MatrixWrapper::Matrix MCPdfVector::getHistogramPos(const StateVector& m, const StateVector& M, const StateVector& step) const
  { 
    return getHistogram(m, M, step);
  }


  /// Get histogram from certain area
  MatrixWrapper::Matrix MCPdfVector::getHistogram(const StateVector& m, const StateVector& M, const StateVector& step) const
  {  
    unsigned int num_samples = _listOfSamples.size();
    unsigned int rows = round((M[0]-m[0])/step[0]);
    unsigned int cols = round((M[1]-m[1])/step[1]);
    Matrix hist(rows, cols);
    hist = 0;

    // calculate histogram
    for (unsigned int i=0; i<num_samples; i++){
      StateVector rel(_listOfSamples[i].ValueGet() - m);
      unsigned int r = round(rel[0] / step[0]);
      unsigned int c = round(rel[1] / step[1]);
      if (r >= 1 && c >= 1 && r <= rows && c <= cols)
	hist(r,c) += _listOfSamples[i].WeightGet();
    }

    return hist;
  }



  unsigned int
  MCPdfVector::numParticlesGet() const
  {
    return _listOfSamples.size();
  }
  

