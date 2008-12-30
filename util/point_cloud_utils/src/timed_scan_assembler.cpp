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


#include "point_cloud_utils/timed_scan_assembler.h"

using namespace std_msgs ;
using namespace point_cloud_utils ;

TimedScanAssembler::TimedScanAssembler(ros::node& rosnode) : rosnode_(rosnode), scan_assembler_(rosnode)
{

}

TimedScanAssembler::~TimedScanAssembler()
{

}

void TimedScanAssembler::getScansBlocking(const std::string topic, const ros::Duration duration,
                                          const std::string target_frame, std_msgs::PointCloud& cloud_out)
{
  got_first_scan_ = false ;                              // Must occur before subscription to prevent race condition.
  done_getting_scans_ = false ;
  duration_ = duration ;                                 // Assembly duration must be put in a location that scansCallback() can reach it
  
  scan_assembler_.startNewCloud(target_frame) ;
  
  rosnode_.subscribe(topic, scan_, &TimedScanAssembler::scansCallback, this, 40) ;

  bool done_getting_scans = false ;
  while(rosnode_.ok() && !done_getting_scans)            // Need to keep checking if we need to exit
  {
    done_condition_.timed_wait(1) ;                      // We could replace this with a usleep, but we don't want to waste any time to return back

    done_lock_.lock() ;
    done_getting_scans = done_getting_scans_ ;           // Copy to threadsafe variable
    done_lock_.unlock() ;
  }
  
  rosnode_.unsubscribe(topic) ;
  
  scan_assembler_.getPointCloud(cloud_out) ;
}

void TimedScanAssembler::scansCallback()
{ 
  if (!got_first_scan_) 
  {
    got_first_scan_ = true ;
    exit_time_ = scan_.header.stamp + duration_ ;       // Specifies our final time based off of the first scan that we receive
  }
  
  if (scan_.header.stamp < exit_time_)                  //! \todo Abstract this time criteria to a criteria that can be passed in by the user
  {
    scan_assembler_.addScan(scan_) ;
  }
  else
  {
    done_lock_.lock() ;
    done_getting_scans_ = true ;
    done_lock_.unlock() ;
    done_condition_.signal() ;
  }
}









