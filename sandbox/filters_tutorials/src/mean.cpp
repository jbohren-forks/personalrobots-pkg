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


#include <stdint.h>
#include <cstring>
#include <stdio.h>

#include <boost/scoped_ptr.hpp>

#include "filters/filter_base.h"
#include "ros/assert.h"

#include "filters/realtime_circular_buffer.h"

//macros
#include <pluginlib/plugin_macros.h>

namespace filters
{

/** \brief A median filter which works on double arrays.
 *
 */
class DoubleMeanFilter: public DoubleFilter
{
public:
  /** \brief Construct the filter with the expected width and height */
  DoubleMeanFilter();

  /** \brief Destructor to clean up
   */
  ~DoubleMeanFilter();

  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in The reference to the input
   * \param data_out The reference to the output
   */
  virtual bool update( const double & data_in, double& data_out);
  
protected:
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector< double > > > data_storage_; ///< Storage for data between updates
  uint32_t last_updated_row_;                     ///< The last row to have been updated by the filter

  std::vector< double > temp;  //used for preallocation and copying from non vector source

  uint32_t number_of_observations_;             ///< Number of observations over which to filter
  uint32_t number_of_channels_;           ///< Number of elements per observation

  
  
};



DoubleMeanFilter::DoubleMeanFilter():
  number_of_observations_(0),
  number_of_channels_(0)
{
}

bool DoubleMeanFilter::configure()
{
  
  if (!FilterBase<double>::getUIntParam("number_of_observations", number_of_observations_, 0))
  {
    ROS_ERROR("DoubleMeanFilter did not find param number_of_observations");
    return false;
  }
  
  temp.resize(number_of_channels_);
  data_storage_.reset(new RealtimeCircularBuffer<std::vector<double> >(number_of_observations_, temp));

  return true;
}

DoubleMeanFilter::~DoubleMeanFilter()
{
}


bool DoubleMeanFilter::update(const double & data_in, double& data_out)
{
  //  ROS_ASSERT(data_in.size() == width_);
  //ROS_ASSERT(data_out.size() == width_);
  if (number_of_channels_ != 1)
    return false;

  temp[0] = data_in;

  //update active row
  if (last_updated_row_ >= number_of_observations_ - 1)
    last_updated_row_ = 0;
  else
    last_updated_row_++;

  data_storage_->push_back(temp);


  unsigned int length = data_storage_->size();
  
  //Return each value
  for (uint32_t i = 0; i < number_of_channels_; i++)
  {
    data_out = 0;
    for (uint32_t row = 0; row < length; row ++)
    {
      data_out += data_storage_->at(row)[i];
    }
    data_out /= length;
  }

  return true;
};





/*
bool DoubleMeanFilter<T>::update(const std::vector<T> & data_in, std::vector<T>& data_out)
{
  //  ROS_ASSERT(data_in.size() == width_);
  //ROS_ASSERT(data_out.size() == width_);
  if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_)
    return false;

  //update active row
  if (last_updated_row_ >= number_of_observations_ - 1)
    last_updated_row_ = 0;
  else
    last_updated_row_++;

  data_storage_->push_back(data_in);


  unsigned int length = data_storage_->size();
  
  //Return each value
  for (uint32_t i = 0; i < number_of_channels_; i++)
  {
    data_out[i] = 0;
    for (uint32_t row = 0; row < length; row ++)
    {
      data_out[i] += data_storage_->at(row)[i];
    }
    data_out[i] /= length;
  }

  return true;
};
*/
}


BEGIN_PLUGIN_LIST(filters::DoubleFilter)
REGISTER_PLUGIN(filters::DoubleMeanFilter)
END_PLUGIN_LIST
