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

#ifndef LASER_SCAN_MEDIAN_FILTER_H
#define LASER_SCAN_MEDIAN_FILTER_H

#include <map>
#include <iostream>
#include <sstream>

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <newmat10/newmatap.h>

#include "rosthread/mutex.h"
#include "std_msgs/LaserScan.h"

/* \mainpage 
 * This is a class for laser scan utilities.  
 * \todo The first goal will be to project laser scans into point clouds efficiently.  
 * The second goal is to provide median filtering.  
 * \todo Other potential additions are upsampling and downsampling algorithms for the scans.
 */

namespace laser_scan{

/** \brief A class to provide median filtering of laser scans */
class LaserMedianFilter
{
public:
  enum MedianMode_t { MEDIAN_TRAILING, MEDIAN_DOWNSAMPLE};
      
  /** \brief Constructor
   * \param averaging_length How many scans to average over.
   * \param num_ranges Whether to downsample and return or compute a rolling median over the last n scans
   * \param mode What mode to operate in Trailing or Downsampling (Effectively changes returning true every time or every 3)
   */
  LaserMedianFilter(unsigned int averaging_length, MedianMode_t mode = MEDIAN_DOWNSAMPLE);
  /** \brief Add a scan to the filter
   * \param scan_in The new scan to filter
   * return whether there is a new output to get */
  bool addScan(const std_msgs::LaserScan& scan_in);
  /** \brief get the Filtered results
   * \param scan_result The scan to fill with the median results */
  void getMedian(std_msgs::LaserScan& scan_result);
      
      
private:
  unsigned int current_packet_num_; ///The number of scans recieved
  NEWMAT::Matrix range_data_; ///Storage for range_data
  NEWMAT::Matrix intensity_data_; ///Storage for intensity data
  unsigned int filter_length_; ///How many scans to average over
  unsigned int num_ranges_; /// How many data point are in each row
  MedianMode_t mode_; ///Whether to return true every time or once every 3
      
  ros::thread::mutex data_lock; /// Protection from multi threaded programs
  std_msgs::LaserScan temp_scan_; /** \todo cache only shallow info not full scan */
      
};
  
  
}
#endif //LASER_SCAN_UTILS_LASERSCAN_H
