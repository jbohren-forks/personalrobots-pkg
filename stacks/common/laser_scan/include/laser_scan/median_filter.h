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
#include "boost/scoped_ptr.hpp"
#include "laser_scan/LaserScan.h"

#include "filters/median.h"
#include "filters/mean.h"
#include "filters/filter_chain.h"
#include "boost/thread/mutex.hpp"

namespace laser_scan{

/** \brief A class to provide median filtering of laser scans in time*/
template <typename T>
class LaserMedianFilter : public filters::FilterBase<T> 
{
public:
  /** \brief Constructor
   * \param averaging_length How many scans to average over.
   */
  LaserMedianFilter();
  ~LaserMedianFilter();

  bool configure();

  /** \brief Update the filter and get the response
   * \param scan_in The new scan to filter
   * \param scan_out The filtered scan
   */
  bool update(const std::vector<laser_scan::LaserScan>& scan_in, std::vector<laser_scan::LaserScan>& scan_out);


private:
  unsigned int filter_length_; ///How many scans to average over
  unsigned int num_ranges_; /// How many data point are in each row

  boost::mutex data_lock; /// Protection from multi threaded programs
  laser_scan::LaserScan temp_scan_; /** \todo cache only shallow info not full scan */

  filters::FilterChain<float> * range_filter_;
  filters::FilterChain<float> * intensity_filter_;

  boost::scoped_ptr<TiXmlElement>  latest_xml_;
};

typedef laser_scan::LaserScan laser_scan_laser_scan;

FILTERS_REGISTER_FILTER(LaserMedianFilter, laser_scan_laser_scan);

template <typename T>
LaserMedianFilter<T>::LaserMedianFilter():
  num_ranges_(1)
{
  
};

template <typename T>
bool LaserMedianFilter<T>::configure()
{
  ROS_ASSERT(this->number_of_channels_ == 1);

  TiXmlNode * child_node = this->raw_xml_.get()->FirstChild("filters");
  if (!child_node)
  {
    ROS_ERROR("Cannot Configure LaserMedianFilter: Didn't find filters tag within LaserMedianFilter. Filter definitions needed inside for processing range and intensity");
    return false;
  }
  TiXmlElement * child = child_node->ToElement();
  
  latest_xml_.reset( child->Clone()->ToElement());
  
  if (range_filter_) delete range_filter_;
  range_filter_ = new filters::FilterChain<float>();
  if (!range_filter_->configure(num_ranges_, latest_xml_.get())) return false;
  
  if (intensity_filter_) delete intensity_filter_;
  intensity_filter_ = new filters::FilterChain<float>();
  if (!intensity_filter_->configure(num_ranges_, latest_xml_.get())) return false;
  return true;
};

template <typename T>
LaserMedianFilter<T>::~LaserMedianFilter()
{
  delete range_filter_;
  delete intensity_filter_;
};

template <typename T>
bool LaserMedianFilter<T>::update(const std::vector<laser_scan::LaserScan>& data_in, std::vector<laser_scan::LaserScan>& data_out)
{
  if (!this->configured_) 
  {
    ROS_ERROR("LaserMedianFilter not configured");
    return false;
  }
  if (data_in.size() != 1 || data_out.size() != 1)
  {
    ROS_ERROR("LaserMedianFilter is not vectorized");
    return false;
  }
  const laser_scan::LaserScan & scan_in = data_in[0];
  laser_scan::LaserScan & scan_out = data_out[0];
  
  boost::mutex::scoped_lock lock(data_lock);
  scan_out = scan_in; ///Quickly pass through all data \todo don't copy data too


  if (scan_in.get_ranges_size() != num_ranges_) //Reallocating
  {
    ROS_INFO("Laser filter clearning and reallocating due to larger scan size");
    delete range_filter_;
    delete intensity_filter_;


    num_ranges_ = scan_in.get_ranges_size();
    
    range_filter_ = new filters::FilterChain<float>();
    if (!range_filter_->configure(num_ranges_, latest_xml_.get())) return false;
  
    intensity_filter_ = new filters::FilterChain<float>();
    if (!intensity_filter_->configure(num_ranges_, latest_xml_.get())) return false;
    
  }

  /** \todo check for length of intensities too */
  range_filter_->update(scan_in.ranges, scan_out.ranges);
  intensity_filter_->update(scan_in.intensities, scan_out.intensities);


  return true;
}



}


#endif //LASER_SCAN_UTILS_LASERSCAN_H
