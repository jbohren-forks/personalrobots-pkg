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
	std::vector<std_msgs::Point3DFloat32> line;
	std::vector<std_msgs::Point3DFloat32> cloud;

	mNewLine.get_pts_vec(line);
	mCurrentCloud.get_pts_vec(cloud);
	cloud.insert( cloud.end(), line.begin(), line.end() );
	mCurrentCloud.set_pts_vec(cloud);

	mMutex.unlock();
}

void ScanListenNode::shutterCallback()
{
	mMutex.lock();
	std_msgs::PointCloudFloat32 empty;
	mCurrentCloud = empty;
	mMutex.unlock();
}

SmartScan* ScanListenNode::getScan(const std_msgs::PointCloudFloat32 &cloud)
{
	if ( cloud.get_pts_size() == 0 ) {
		return NULL;
	}

	SmartScan *retScan = new SmartScan();
	retScan->setPoints( cloud.get_pts_size(), cloud.pts);

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

