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

// Author: Marius Muja


#include <string>

#include "ros/node.h"
#include "opencv_latest/CvBridge.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "prosilica_cam/PolledImage.h"

#include <point_cloud_mapping/cloud_io.h>

static char wndname[] = "Captured image";


using namespace std;

class ProsilicaCapture {

	image_msgs::CvBridge bridge;
	IplImage* image;

	prosilica_cam::PolledImage::Request req;
	prosilica_cam::PolledImage::Response res;

	unsigned int index;

	robot_msgs::PointCloud base_cloud_;
	robot_msgs::PointCloud base_cloud_fetch_;
	string base_laser_topic_;

	ros::Node* node;

	boost::mutex clound_point_mutex;
	boost::condition_variable cloud_point_cv;
	bool capture_cloud_point_;

public:
	ProsilicaCapture() : index(0)
	{
		cvNamedWindow(wndname, 0);

		node = ros::Node::instance();

		node->param<string>("base_laser", base_laser_topic_, "base_scan_marking");
		node->subscribe<robot_msgs::PointCloud,ProsilicaCapture>(base_laser_topic_, base_cloud_, &ProsilicaCapture::laser_callback, this, 1);
	}

	void captureFromProsilica(int timeout)
	{
		req.timeout_ms = timeout;
		ROS_INFO("Capturing prosilica image");
		if (!ros::service::call("prosilica/poll", req, res)) {
			ROS_ERROR("Service call failed");
			image = NULL;
		}
		else {
			if (!bridge.fromImage(res.image, "bgr")) {
				ROS_ERROR("CvBridge::fromImage failed");
				image = NULL;
			}
			else {
				image = bridge.toIpl();
			}
		}
	}

	void laser_callback()
	{
		if (!capture_cloud_point_) {
			return;
		}
		boost::lock_guard<boost::mutex> lock(clound_point_mutex);
		capture_cloud_point_ = false;
		base_cloud_ = base_cloud_fetch_;

		cloud_point_cv.notify_all();
	}



	void captureFromBaseLaser()
	{
		boost::unique_lock<boost::mutex> lock(clound_point_mutex);
		capture_cloud_point_ = true;
		// waiting for the cloud point
		ROS_INFO("Waiting for base laser");
		while (capture_cloud_point_) {
			cloud_point_cv.wait(lock);
		}
	}

	void capture()
	{
		ROS_INFO("Capturing data");
		captureFromProsilica(100);
		captureFromBaseLaser();
	}

	void saveData()
	{
		ROS_INFO("Saving data");
		if (image!=NULL) {
			char filename[16];
			// saving image
			sprintf(filename, "frame%04u.jpg", index);
			cvSaveImage( filename, image );
			ROS_INFO("Saved image %s", filename);
			sprintf(filename, "base_laser%04u.jpg", index);
			cloud_io::savePCDFile (filename, base_cloud_, true);
			ROS_INFO("Saved base point cloud %s", filename);
			index++;
		}

	}

	void spin()
	{

		while (true)
		{
			int k = cvWaitKey(0);
			switch( (char) k)
			{
			case 'c':
				capture();
				if (image)
					cvShowImage(wndname, image);
				break;
			case 's':
				saveData();
				break;
			}
		}

	}



};




int main(int argc, char** argv)
{
	ros::init(argc, argv);
	ros::Node n("prosilica_capture");

	ProsilicaCapture capture;
	capture.spin();


}
