/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "filters/mean.h"
#include <cstring>
#include <stdio.h>

MeanFilter::MeanFilter(uint32_t number_of_observations, uint32_t elements_per_observation):
  last_updated_row_(number_of_observations),
  iterations_(0),
  number_of_observations_(number_of_observations),
  elements_per_observation_(elements_per_observation)
{
  data_storage_ = new double[number_of_observations_ * elements_per_observation];
}

MeanFilter::~MeanFilter()
{
  delete [] data_storage_;
}

bool MeanFilter::update(double * data)
{
  return update (data, data);
}

bool MeanFilter::update(double const* const data_in, double* data_out)
{
  //update active row
  if (last_updated_row_ >= number_of_observations_ - 1)
    last_updated_row_ = 0;
  else 
    last_updated_row_++;

  //copy incoming data into perminant storage
  memcpy(&data_storage_[elements_per_observation_ * last_updated_row_],
         data_in, 
         sizeof(double) * elements_per_observation_);
  
  //Return values
  
  //keep track of number of rows used while starting up
  uint32_t length;
  if (iterations_ < number_of_observations_ )
  {
    iterations_++;
    length = iterations_;
  }
  else //all rows are allocated
  {
    length = number_of_observations_;
  }

  double sum;
  
  //Return each value
  for (uint32_t i = 0; i < number_of_observations_; i++)
  {
    data_out[i] = 0;
    for (uint32_t row = 0; row < length; row ++)
    {
      data_out[i] += data_storage_[i + row * elements_per_observation_];
    }
    data_out[i] /= length;
  }    
  
  return true;
}

