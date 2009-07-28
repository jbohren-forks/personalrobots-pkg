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

#include <vector>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <limits>


// opencv
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "opencv_latest/CvBridge.h"

// boost
#include <boost/thread.hpp>

// ros & ros messages & ros services
#include "ros/node.h"
#include "sensor_msgs/StereoInfo.h"
#include "sensor_msgs/DisparityInfo.h"
#include "sensor_msgs/CamInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"

#include <point_cloud_mapping/cloud_io.h>


#include "tf/transform_listener.h"
#include "topic_synchronizer/topic_synchronizer.h"

using namespace std;
using namespace robot_msgs;


#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)

class PeriodicCapture : public ros::Node
{
public:

	sensor_msgs::Image limage_;
	sensor_msgs::Image rimage_;
	sensor_msgs::Image dimage_;
	sensor_msgs::StereoInfo stinfo_;
	sensor_msgs::DisparityInfo dispinfo_;
	sensor_msgs::CamInfo lcinfo_;
	sensor_msgs::CamInfo rcinfo_;

	sensor_msgs::CvBridge lbridge_;
	sensor_msgs::CvBridge rbridge_;
	sensor_msgs::CvBridge dbridge_;

	sensor_msgs::PointCloud cloud_;
	sensor_msgs::PointCloud cloud_fetch_;

	sensor_msgs::PointCloud base_cloud_;
	sensor_msgs::PointCloud base_cloud_fetch_;

	IplImage* left_;
	IplImage* right_;
	IplImage* disp_;

	bool display_;

	TopicSynchronizer<PeriodicCapture> sync_;


	boost::mutex data_lock_;
	boost::condition_variable data_cv_;
	bool have_cloud_point_;
	bool have_images_;

	tf::TransformListener* tf_;
	string base_scan_topic_;
	string prefix_;
	double period_;
	string save_directory_;

	PeriodicCapture() : ros::Node("periodic_capture"), left_(NULL), right_(NULL), disp_(NULL),
		sync_(this, &PeriodicCapture::image_cb_all, ros::Duration().fromSec(0.1), &PeriodicCapture::image_cb_timeout), have_cloud_point_(false), have_images_(false)
	{
        tf_ = new tf::TransformListener(*this);

		param("~display", display_, false);
		param<string> ("~base_scan_topic", base_scan_topic_, "base_scan_marking");
		param("~period", period_, 1.0);
		param<string> ("~save_directory", save_directory_, "data");
		param<string> ("~prefix", prefix_, "capture");

		if (display_) {
			ROS_INFO("Displaying images\n");
			cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
		}
		subscribeToData();
	}

	~PeriodicCapture()
	{
		if (left_)
			cvReleaseImage(&left_);
		if (right_)
			cvReleaseImage(&right_);
		if (disp_)
			cvReleaseImage(&disp_);

		unsubscribeFromData();
	}

private:


    void subscribeToData()
    {
    	sync_.reset();
		std::list<std::string> left_list;
		left_list.push_back(std::string("stereo/left/image_rect_color"));
		left_list.push_back(std::string("stereo/left/image_rect"));
		sync_.subscribe(left_list,  limage_, 1);

		std::list<std::string> right_list;
		right_list.push_back(std::string("stereo/right/image_rect_color"));
		right_list.push_back(std::string("stereo/right/image_rect"));
		sync_.subscribe(right_list, rimage_, 1);

		sync_.subscribe("stereo/disparity", dimage_, 1);
		sync_.subscribe("stereo/stereo_info", stinfo_, 1);
		sync_.subscribe("stereo/disparity_info", dispinfo_, 1);
		sync_.subscribe("stereo/left/cam_info", lcinfo_, 1);
		sync_.subscribe("stereo/right/cam_info", rcinfo_, 1);

		sync_.subscribe("stereo/cloud", cloud_fetch_, 1);

		sync_.ready();

		subscribe(base_scan_topic_, base_cloud_fetch_, &PeriodicCapture::laser_callback, 1);
    }

    void unsubscribeFromData()
    {
        unsubscribe("stereo/left/image_rect_color");
        unsubscribe("stereo/left/image_rect");
        unsubscribe("stereo/right/image_rect_color");
        unsubscribe("stereo/right/image_rect");
        unsubscribe("stereo/disparity");
        unsubscribe("stereo/stereo_info");
        unsubscribe("stereo/disparity_info");
        unsubscribe("stereo/left/cam_info");
        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");

		unsubscribe(base_scan_topic_);
}



    /**
     *
     */
	void laser_callback()
	{
		boost::lock_guard<boost::mutex> lock(data_lock_);
		base_cloud_ = base_cloud_fetch_;
		have_cloud_point_ = true;
		data_cv_.notify_all();
	}


	void image_cb_all(ros::Time t)
	{
        boost::lock_guard<boost::mutex> lock(data_lock_);

		if (lbridge_.fromImage(limage_, "bgr"))
		{
			if(left != NULL)
				cvReleaseImage(&left_);
			left_ = cvCloneImage(lbridge_.toIpl());
		}

		if (rbridge_.fromImage(rimage_, "bgr"))
		{
			if(right_ != NULL)
				cvReleaseImage(&right_);

			right_ = cvCloneImage(rbridge_.toIpl());
		}

		if (dbridge_.fromImage(dimage_))
		{
			if(disp_ != NULL)
				cvReleaseImage(&disp_);

			disp_ = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
			cvCvtScale(dbridge_.toIpl(), disp_, 4.0/dispinfo_.dpp);
		}

		cloud_ = cloud_fetch_;

		have_images_ = true;
		data_cv_.notify_all();
	}

	void image_cb_timeout(ros::Time t)
	{
		if (limage_.header.stamp != t)
			printf("Timed out waiting for left image\n");

		if (rimage_.header.stamp != t)
			printf("Timed out waiting for right image\n");

		if (dimage_.header.stamp != t)
			printf("Timed out waiting for disparity image\n");

		if (stinfo_.header.stamp != t)
			printf("Timed out waiting for stereo info\n");

		if (cloud_fetch_.header.stamp != t)
			printf("Timed out waiting for point cloud\n");

	}


	void saveData()
	{
//		subscribeToData();

		{
			boost::unique_lock<boost::mutex> lock(data_lock_);

			ROS_INFO("PeriodicCapture: waiting for images and base laser data");
			while (!(have_images_ && have_cloud_point_)) {
				data_cv_.wait(lock);
			}

			// transfrom base_cloud to stereo frame
			PointCloud base_cloud_in_stereo_frame;
			tf_->transformPointCloud(cloud_.header.frame_id, base_cloud_, base_cloud_in_stereo_frame);

			char name[1024];
			name[0] = 0;
			time_t t = ::time(NULL);
			struct tm *tms = localtime(&t);
			snprintf(name, sizeof(name), "%s-%d-%02d-%02d-%02d-%02d-%02d", prefix_.c_str(),
					tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
					tms->tm_hour     , tms->tm_min  , tms->tm_sec);

			string filename_prefix = name;


			if (display_) {
				cvShowImage("left", left_);
				cvShowImage("right", right_);
				cvShowImage("disparity", disp_);
			}



			string filename;
			filename = save_directory_+ "/" + filename_prefix+"-left.png";
			ROS_INFO("Saving %s", filename.c_str());
			cvSaveImage(filename.c_str(), left_);

			filename = save_directory_+ "/" + filename_prefix+"-right.png";
			ROS_INFO("Saving %s", filename.c_str());
			cvSaveImage(filename.c_str(), right_);

			filename = save_directory_+ "/" + filename_prefix+"-disparity.png";
			ROS_INFO("Saving %s", filename.c_str());
			cvSaveImage(filename.c_str(), disp_);

			filename = save_directory_+ "/" + filename_prefix+"-stereo_cloud.pcd";
			ROS_INFO("Saving %s", filename.c_str());
			cloud_io::savePCDFile(filename.c_str(), cloud_, true);

			filename = save_directory_+ "/" + filename_prefix+"-base_cloud.pcd";
			ROS_INFO("Saving %s", filename.c_str());
			cloud_io::savePCDFile(filename.c_str(), base_cloud_in_stereo_frame, true);

			have_images_ = false;
			have_cloud_point_ = false;
		}
//        unsubscribeFromData();

        ros::Duration(period_).sleep();
	}

public:

	bool spin()
	{
		while (ok())
		{
			data_lock_.lock();
			int key = cvWaitKey(3)&0x00FF;
			data_lock_.unlock();
			if(key == 27) //ESC
				break;

			try {
				saveData();
			}
			catch(tf::TransformException& e) {
				ROS_ERROR("Failed to save data, reason: %s", e.what());
			}
			usleep(10000);
		}

		return true;
	}


};




int main(int argc, char **argv)
{
	ros::init(argc, argv);
	PeriodicCapture capture;

	capture.spin();

	return 0;
}

