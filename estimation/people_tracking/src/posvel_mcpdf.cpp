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


#include "posvel_mcpdf.hpp"
#include <assert.h>
#include <vector>


  using namespace MatrixWrapper;
  using namespace KDL;
  using namespace BFL;
  

  PosVelMCPdf::PosVelMCPdf (unsigned int num_samples) 
    : MCPdf<Posvel_State> ( num_samples, 7 )
  {}

  PosVelMCPdf::~PosVelMCPdf(){}


  void 
  PosVelMCPdf::ContactformationProbabilityGet(std::vector<double>& cf_probability, std::vector<unsigned int>& cf_num_samples) const
  {
    assert(cf_num_samples.size() == cf_probability.size());

    unsigned int num_nodes   = cf_probability.size();
    unsigned int num_smaples = _listOfSamples.size();
    
    // set to zero
    for (unsigned int id=0; id<num_nodes; id++){
      cf_probability[id] = 0;
      cf_num_samples[id] = 0;
    }

    // calculate probability
    for (unsigned int i=0; i<num_smaples; i++){
      unsigned int id = _listOfSamples[i].ValueGet().ContactFormationGet()->IdGet();
      //cout << _listOfSamples[i].ValueGet().ContactFormationGet()->LabelGet() << " with probability of " << _listOfSamples[i].WeightGet() / _SumWeights << endl;
      assert(id <= num_nodes);
      cf_probability[id-1] += (_listOfSamples[i].WeightGet() / _SumWeights);
      cf_num_samples[id-1] += 1;
    }
  }


  WeightedSample<Posvel_State>
  PosVelMCPdf::SampleGet(unsigned int particle) const
  {
    assert (particle >= 0 && particle < _listOfSamples.size());
    return _listOfSamples[particle];
  }


  unsigned int
  PosVelMCPdf::numParticlesGet() const
  {
    return _listOfSamples.size();
  }
  

