#ifndef _scanlistennode_h_
#define _scanlistennode_h_

#include <ros/node.h>
#include "rosthread/mutex.h"

#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Empty.h>

/**
   @mainpage 

   @b This is a thin ROS node that can create instances of the
   SmartScan class in the library Scan Utils from point clouds received
   as ROS messages. See Scan Utils package and its SmartScan class for
   details of the available tools.
 **/

class SmartScan;

/*!
  This class listens for point clouds coming out of ROS and processes them into a
  scan_utils::SmartScan. It is a thin ROS wrapper for the Scan Utils library.
*/
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
