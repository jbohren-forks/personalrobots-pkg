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

#include "laser_scan/median_filter.h"
#include <algorithm>

namespace laser_scan{

LaserMedianFilter::LaserMedianFilter(unsigned int filter_length, MedianMode_t mode):
  current_packet_num_(0),
  range_data_(filter_length,1),
  intensity_data_(filter_length,1),
  filter_length_(filter_length),
  num_ranges_(1),
  mode_(mode)
{
};

bool LaserMedianFilter::addScan(const std_msgs::LaserScan& scan_in)
{
  temp_scan_ = scan_in; //HACK to store all metadata 

  if (scan_in.get_ranges_size() != num_ranges_) //Reallocating
  {
    ROS_INFO("Median filter clearning and reallocating due to larger scan size");
    range_data_ = NEWMAT::Matrix(filter_length_, scan_in.get_ranges_size());
    intensity_data_ = NEWMAT::Matrix(filter_length_, scan_in.get_ranges_size());
    current_packet_num_ = 0;
  }

  /** \todo check for length of intensities too */
  for (unsigned int index = 0; index < scan_in.get_ranges_size(); index ++)
  {
    range_data_(current_packet_num_+1, index+1)= (double) scan_in.ranges[index];
    intensity_data_(current_packet_num_+1, index+1)= (double) scan_in.intensities[index];
  }
  current_packet_num_++;
  if (current_packet_num_ == filter_length_)
  {
    current_packet_num_ = 0;
    return true;
  }
  if (mode_ == MEDIAN_TRAILING)
    return true;
  return false;
}

void LaserMedianFilter::getMedian(std_msgs::LaserScan& scan_result)
{
  scan_result = temp_scan_; //Fill in latest scan data
    
  NEWMAT::ColumnVector dColumn;
  NEWMAT::ColumnVector iColumn;


  unsigned int iterations = std::min(scan_result.get_ranges_size(), num_ranges_);
  /** \todo Resize output cloud/check length */
  for (unsigned int index = 0; index < iterations; index ++)
  {
    dColumn = range_data_.Column(index+1);
    NEWMAT::SortAscending(dColumn);
    scan_result.ranges[index] = (float) dColumn((size_t)ceil(filter_length_/2));
    iColumn = intensity_data_.Column(index+1);
    NEWMAT::SortAscending(iColumn);
    scan_result.intensities[index] = (float) iColumn((size_t)ceil(filter_length_/2));
  }
  //reset to beginning
  current_packet_num_ = 0;
}

}
