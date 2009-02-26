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

#ifndef FILTERS_MEAN_H_
#define FILTERS_MEAN_H_

#include <stdint.h>
#include <cstring>
#include <stdio.h>

#include "filters/filter_base.h"
#include "ros/assert.h"

#include "filters/realtime_vector_circular_buffer.h"

namespace filters
{

/** \brief A median filter which works on double arrays.
 *
 */
template <typename T>
class MeanFilter: public FilterBase <T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MeanFilter();

  /** \brief Destructor to clean up
   */
  ~MeanFilter();

  virtual bool configure(unsigned int number_of_channels, TiXmlElement *config);

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update( const T & data_in, T& data_out);
  virtual bool update( const std::vector<T> & data_in, std::vector<T>& data_out);
  
protected:
  RealtimeVectorCircularBuffer<std::vector<T> >* data_storage_; ///< Storage for data between updates
  uint32_t last_updated_row_;                     ///< The last row to have been updated by the filter

  std::vector<T> temp;  //used for preallocation and copying from non vector source

  uint32_t number_of_observations_;             ///< Number of observations over which to filter
  uint32_t number_of_channels_;           ///< Number of elements per observation

  bool configured_;

  
  
};


template <typename T>
MeanFilter<T>::MeanFilter():
  number_of_observations_(0),
  number_of_channels_(0),
  configured_(false)
{
}

template <typename T>
bool MeanFilter<T>::configure(unsigned int number_of_channels, TiXmlElement *config)
{
  if (configured_)
    return false;
  
  // Parse the name of the filter from the xml.  
  if (!FilterBase<T>::setName(config))
  {
    fprintf(stderr, "Error: TransferFunctionFilter was not given a name.\n");
    return false;
  }

  // Parse the params of the filter from the xml.
  TiXmlElement *p = config->FirstChildElement("params");
  if (!p)
  {
    fprintf(stderr, "Error: TransferFunctionFilter was not given params.\n");
    return false;
  }
  
  number_of_observations_ = atof(p->Attribute("number_of_observations"));
  number_of_channels_ = number_of_channels;
  
  temp.resize(number_of_channels);
  data_storage_ = new RealtimeVectorCircularBuffer<std::vector<T> >(number_of_observations_, temp);

  configured_ = true;
  return true;
}

template <typename T>
MeanFilter<T>::~MeanFilter()
{
  if (data_storage_) delete data_storage_;
}


template <typename T>
bool MeanFilter<T>::update(const T & data_in, T& data_out)
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
template <typename T>
bool MeanFilter<T>::update(const std::vector<T> & data_in, std::vector<T>& data_out)
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

ROS_REGISTER_FILTER(MeanFilter, double)
ROS_REGISTER_FILTER(MeanFilter, float)

}

#endif //#ifndef FILTERS_MEDIAN_H_
