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

/* author: Matei Ciocarlie */

#ifndef _scanlistennode_h_
#define _scanlistennode_h_

#include <ros/node.h>
#include "boost/thread/mutex.hpp"

#include <std_msgs/PointCloud.h>
#include <std_msgs/Empty.h>

/*! \file

  This is a thin ROS node that can create instances of the
  SmartScan class in the library Scan Utils from point clouds received
  as ROS messages. See Scan Utils package and its SmartScan class for
  details of the available tools.
 */

class SmartScan;

/*!
  This class listens for point clouds coming out of ROS and processes them into a
  scan_utils::SmartScan. It is a thin ROS wrapper for the Scan Utils library.
*/
class ScanListenNode : public ros::Node
{
 private:
	std_msgs::PointCloud mNewCloud,mLastCloud;
	std_msgs::PointCloud mNewLine,mCurrentCloud;
	std_msgs::Empty mEmptyMsg;

	boost::mutex mMutex;

	void fullCloudCallback();
	void cloudCallback();
	void shutterCallback();

	SmartScan* getScan(const std_msgs::PointCloud &cloud);

 public:
	ScanListenNode();
	virtual ~ScanListenNode();

	SmartScan* getFullScan();
	SmartScan* getCurrentScan();
}; 

#endif
