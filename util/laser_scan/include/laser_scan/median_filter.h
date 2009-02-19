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

#include "boost/thread/mutex.hpp"
#include "laser_scan/LaserScan.h"

#include "filters/median.h"
#include "filters/mean.h"
#include "filters/filter_chain.h"
#include "boost/thread/mutex.hpp"

namespace laser_scan{

/** \brief A class to provide median filtering of laser scans */
class LaserMedianFilter
{
public:
  /** \brief Constructor
   * \param averaging_length How many scans to average over.
   */
  LaserMedianFilter();//const std::string & xml_parameters);
  ~LaserMedianFilter();

  bool configure(const std::string & xml);

  /** \brief Update the filter and get the response
   * \param scan_in The new scan to filter
   * \param scan_out The filtered scan
   */
  bool update(const laser_scan::LaserScan& scan_in, laser_scan::LaserScan& scan_out);


private:
  unsigned int filter_length_; ///How many scans to average over
  unsigned int num_ranges_; /// How many data point are in each row

  boost::mutex data_lock; /// Protection from multi threaded programs
  laser_scan::LaserScan temp_scan_; /** \todo cache only shallow info not full scan */

  filters::FilterChain<std_vector_float > * range_filter_;
  filters::FilterChain<std_vector_float > * intensity_filter_;

  std::string latest_xml_;
};


}
#endif //LASER_SCAN_UTILS_LASERSCAN_H
