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
*   * Redistributions of source code must retain the above copyright
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


/**

@mainpage

@htmlinclude manifest.html

\author Vijay Pradeep
@b ScanIntensityFilter takes input scans and fiters out that are not within the specified range. The filtered out readings are set at >max_range \
in order to invalidate them.

**/


#include "ros/node.h"
#include "std_msgs/LaserScan.h"

using namespace std_msgs;

class ScanIntensityFilter : public ros::Node
{
public:

  LaserScan cur_scan_ ;
  double lower_threshold_ ;
  double upper_threshold_ ;
  int disp_hist_ ;

  ScanIntensityFilter() : ros::Node("scan_intensity_filter")
  {
    advertise<LaserScan>("filtered_scan", 1) ;
    subscribe("input_scan", cur_scan_, &ScanIntensityFilter::scans_callback, 40) ;

    param("~lower_threshold", lower_threshold_, 8000.0) ;
    param("~upper_threshold", upper_threshold_, 100000.0) ;
    param("~disp_histogram",  disp_hist_, 1) ;
  }

  virtual ~ScanIntensityFilter()
  { 

  }

  void scans_callback()
  {
    const double hist_max = 4*12000.0 ;
    const int num_buckets = 24 ;
    int histogram[num_buckets] ;
    for (int i=0; i < num_buckets; i++)
      histogram[i] = 0 ;

    LaserScan filtered_scan ;    
    filtered_scan = cur_scan_ ;

    for (unsigned int i=0; i < cur_scan_.ranges.size(); i++)                            // Need to check ever reading in the current scan
    {
      if (filtered_scan.intensities[i] <= lower_threshold_ ||                           // Is this reading below our lower threshold?
          filtered_scan.intensities[i] >= upper_threshold_)                             // Is this reading above our upper threshold?
      {                                                                                 
        filtered_scan.ranges[i] = cur_scan_.range_max + 1.0 ;                           // If so, then make it a value bigger than the max range
      }

      int cur_bucket = (int) ((filtered_scan.intensities[i]/hist_max)*num_buckets) ;
      if (cur_bucket >= num_buckets-1)
	cur_bucket = num_buckets-1 ;
      histogram[cur_bucket]++ ;
    }

    if (disp_hist_ > 0)                                                                 // Display Histogram
    {
      printf("********** SCAN **********\n") ;
      for (int i=0; i < num_buckets; i++)
      {
        printf("%u - %u: %u\n", (unsigned int) hist_max/num_buckets*i,
                                (unsigned int) hist_max/num_buckets*(i+1),
                                histogram[i]) ;
      }
    }

    publish("filtered_scan", filtered_scan) ;                                           // Publish the filtered data
  }
} ;

int main(int argc, char **argv)
{
  ros::init(argc, argv) ;
  ScanIntensityFilter filter ;
  filter.spin() ;
  ros::fini() ;
  return 0 ;
}

