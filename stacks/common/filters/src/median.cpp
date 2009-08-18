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


#include "filters/median.h"
#include <pluginlib/plugin_macros.h>

namespace filters
{

MedianDoubleFilter::MedianDoubleFilter():
  number_of_observations_(0)
{
  
};

MedianDoubleFilter::~MedianDoubleFilter()
{
};


bool MedianDoubleFilter::configure()
{
  int no_obs;
  if (!FilterBase<double>::getIntParam(std::string("number_of_observations"), no_obs, 0))
  {
    fprintf(stderr, "Error: MedianDoubleFilter was not given params.\n");
    return false;
  }
  number_of_observations_ = no_obs;
    
  temp.resize(this->number_of_channels_);
  data_storage_.reset( new RealtimeCircularBuffer<std::vector<double> >(number_of_observations_, temp));
  temp_storage_.resize(number_of_observations_);
  
  return true;
};

bool MedianDoubleFilter::update(const std::vector<double>& data_in, std::vector<double>& data_out)
{
  //  printf("Expecting width %d, got %d and %d\n", width_, data_in.size(),data_out.size());
  if (data_in.size() != this->number_of_channels_ || data_out.size() != this->number_of_channels_)
    return false;
  if (!this->configured_)
    return false;

  data_storage_->push_back(data_in);


  unsigned int length = data_storage_->size();
 

  for (uint32_t i = 0; i < this->number_of_channels_; i++)
  {
    for (uint32_t row = 0; row < length; row ++)
    {
      temp_storage_[row] = (*data_storage_)[row][i];
    }
    data_out[i] = median(&temp_storage_[0], length);
  }

  return true;
};

bool MedianDoubleFilter::update(const double& data_in, double& data_out)
{
  //  printf("Expecting width %d, got %d and %d\n", width_, data_in.size(),data_out.size());
  if (this->number_of_channels_ != 1)
    return false;
  if (!this->configured_)
    return false;

  temp[0] = data_in;

  data_storage_->push_back(temp);
  ///\todo standardize on calling to vector class

  unsigned int length = data_storage_->size();
 

  for (uint32_t row = 0; row < length; row ++)
  {
    temp_storage_[row] = (*data_storage_)[row][0];
  }
  data_out = median(&temp_storage_[0], length);


  return true;
};

}
BEGIN_PLUGIN_LIST(filters::FilterBase<double>)
REGISTER_PLUGIN(filters::MedianDoubleFilter)
END_PLUGIN_LIST





