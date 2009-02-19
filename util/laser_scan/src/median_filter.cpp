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

using namespace filters ;


namespace laser_scan{


LaserMedianFilter::LaserMedianFilter():
  num_ranges_(1)
{
  
};

bool LaserMedianFilter::configure(const std::string & xml_parameters)
{
  latest_xml_ = xml_parameters;
  range_filter_ = new FilterChain<std_vector_float >();
  range_filter_->add(latest_xml_);
  range_filter_->configure(num_ranges_);
  
  intensity_filter_ = new FilterChain<std_vector_float >();
  intensity_filter_->add(latest_xml_);
  intensity_filter_->configure(num_ranges_);
};

LaserMedianFilter::~LaserMedianFilter()
{
  delete range_filter_;
  delete intensity_filter_;
};

bool LaserMedianFilter::update(const laser_scan::LaserScan& scan_in, laser_scan::LaserScan& scan_out)
{
  boost::mutex::scoped_lock lock(data_lock);
  scan_out = scan_in; ///Quickly pass through all data \todo don't copy data too

  if (scan_in.get_ranges_size() != num_ranges_) //Reallocating
  {
    ROS_INFO("Laser filter clearning and reallocating due to larger scan size");
    delete range_filter_;
    delete intensity_filter_;


    num_ranges_ = scan_in.get_ranges_size();
    
    range_filter_ = new FilterChain<std_vector_float >();
    range_filter_->add(latest_xml_);
    range_filter_->configure(num_ranges_);
    
    intensity_filter_ = new FilterChain<std_vector_float >();
    intensity_filter_->add(latest_xml_);
    intensity_filter_->configure(num_ranges_);
    
  }

  /** \todo check for length of intensities too */
  range_filter_->update(scan_in.ranges, scan_out.ranges);
  intensity_filter_->update(scan_in.intensities, scan_out.intensities);


  return true;
}

}
