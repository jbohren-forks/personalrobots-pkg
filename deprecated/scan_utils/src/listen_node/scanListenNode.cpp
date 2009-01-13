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

#include "listen_node/scanListenNode.h"

#include "smartScan.h"

#include <stdio.h>
#include <vector>

ScanListenNode::ScanListenNode() : ros::node("ros_graspit")
{
	subscribe("full_cloud", mNewCloud, &ScanListenNode::fullCloudCallback);
	subscribe("cloud", mNewLine, &ScanListenNode::cloudCallback);
	subscribe("shutter", mEmptyMsg, &ScanListenNode::shutterCallback);
	fprintf(stderr,"ROS Scanner created and subscribed!\n");
}

ScanListenNode::~ScanListenNode()
{
	fprintf(stderr,"ScanListenNode destructed\n");
}


void ScanListenNode::fullCloudCallback()
{
	mMutex.lock();
	mLastCloud = mNewCloud;
	mMutex.unlock();
}

void ScanListenNode::cloudCallback()
{
	mMutex.lock();
	std::vector<std_msgs::Point32> line;
	std::vector<std_msgs::Point32> cloud;

	mNewLine.get_pts_vec(line);
	mCurrentCloud.get_pts_vec(cloud);
	cloud.insert( cloud.end(), line.begin(), line.end() );
	mCurrentCloud.set_pts_vec(cloud);

	mMutex.unlock();
}

void ScanListenNode::shutterCallback()
{
	mMutex.lock();
	std_msgs::PointCloud empty;
	mCurrentCloud = empty;
	mMutex.unlock();
}

SmartScan* ScanListenNode::getScan(const std_msgs::PointCloud &cloud)
{
	if ( cloud.get_pts_size() == 0 ) {
		return NULL;
	}

	SmartScan *retScan = new SmartScan();
	retScan->setPoints( cloud.get_pts_size(), &cloud.pts[0]);

	return retScan;
}

SmartScan* ScanListenNode::getFullScan()
{
	mMutex.lock();
	SmartScan *s =  getScan(mLastCloud);
	mMutex.unlock();
	return s;
}

SmartScan* ScanListenNode::getCurrentScan()
{
	mMutex.lock();
	SmartScan *s = getScan(mCurrentCloud);
	mMutex.unlock();
	return s;
}

