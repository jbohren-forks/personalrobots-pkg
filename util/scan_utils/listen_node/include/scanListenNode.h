#ifndef _scanlistennode_h_
#define _scanlistennode_h_

#include <ros/node.h>
#include "rosthread/mutex.h"

#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Empty.h>


/*
  This class listens for point clouds coming out of ROS and processes them into a
  scan_utils::SmartScan
*/

class SmartScan;

class ScanListenNode : public ros::node
{
 private:
	std_msgs::PointCloudFloat32 mNewCloud,mLastCloud;
	std_msgs::PointCloudFloat32 mNewLine,mCurrentCloud;
	std_msgs::Empty mEmptyMsg;

	ros::thread::mutex mMutex;

	void fullCloudCallback();
	void cloudCallback();
	void shutterCallback();

	SmartScan* getScan(const std_msgs::PointCloudFloat32 &cloud);

 public:
	ScanListenNode();
	virtual ~ScanListenNode();

	SmartScan* getFullScan();
	SmartScan* getCurrentScan();
}; 

#endif
