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

#ifndef POINT_CLOUD_UTILS_TIMED_SCAN_ASSEMBLER_H_
#define POINT_CLOUD_UTILS_TIMED_SCAN_ASSEMBLER_H_

#include "ros/node.h"
#include "boost/thread/condition_variable.hpp"
#include "boost/thread/mutex.hpp"

#include "std_msgs/PointCloud.h"
#include "std_msgs/LaserScan.h"

#include "point_cloud_utils/scan_assembler.h"

namespace point_cloud_utils
{

class TimedScanAssembler
{
public:
  TimedScanAssembler(ros::Node& rosnode) ;
  ~TimedScanAssembler() ;

  /**
   * \brief Accumulates scans while blocking
   * Accumulates scans for the specified duration. Note that this method NEVER checks the system-clock. It scans until difference in
   * timestamps between the first and last accumulated scans exceeds the specified duration.
   * \param topic The ros topic that should be subscribed to on which laser scans should be processed
   * \param duration Minimum time between the first and last scans accumulated
   * \param target_frame the libTF frame that the laser data should be transformed into
   * \param cloud_out (Output) Stores the assembled point cloud
   */
  void getScansBlocking(const std::string topic, const ros::Duration duration, const std::string target_frame, std_msgs::PointCloud& cloud_out) ;

private:
  void scansCallback() ;

  std_msgs::LaserScan scan_ ;

  bool got_first_scan_ ;
  bool done_getting_scans_ ;

  ros::Duration duration_ ;
  ros::Time exit_time_ ;

  ros::Node& rosnode_ ;

  boost::condition_variable done_condition_ ;
  boost::mutex done_lock_ ;

  ScanAssembler scan_assembler_ ;
} ;

}

#endif // POINT_CLOUD_UTILS_TIMED_SCAN_ASSEMBLER_H_
