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

#include <vector>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <queue>


#include "opencv_latest/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"

#include <string>

// transform library
#include <tf/transform_listener.h>

#include "topic_synchronizer/topic_synchronizer.h"

#include "CvStereoCamModel.h"

#include <boost/thread.hpp>

#include "recognition_lambertian/chamfer_matching.h"

using namespace std;


void on_edges_low(int);
void on_edges_high(int);


#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)






class RecognitionLambertian : public ros::Node
{
public:


	image_msgs::Image limage;
	image_msgs::Image rimage;
	image_msgs::Image dimage;
	image_msgs::StereoInfo stinfo;
	image_msgs::DisparityInfo dispinfo;
	image_msgs::CamInfo rcinfo;
	image_msgs::CvBridge lbridge;
	image_msgs::CvBridge rbridge;
	image_msgs::CvBridge dbridge;

	robot_msgs::PointCloud cloud_fetch;
	robot_msgs::PointCloud cloud;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

	TopicSynchronizer<RecognitionLambertian> sync;

	boost::mutex cv_mutex;
	boost::condition_variable images_ready;

	tf::TransformListener *tf_;


	// minimum height to look at (in base_link frame)
	double min_height;
	// maximum height to look at (in base_link frame)
	double max_height;
	// no. of frames to detect handle in
	int frames_no;
	// display stereo images ?
	bool display;

	int edges_low;
	int edges_high;


	ChamferMatching* cm;

    RecognitionLambertian()
    :ros::Node("stereo_view"), left(NULL), right(NULL), disp(NULL), disp_clone(NULL), sync(this, &RecognitionLambertian::image_cb_all, ros::Duration().fromSec(0.1), &RecognitionLambertian::image_cb_timeout)
    {
        tf_ = new tf::TransformListener(*this);
        // define node parameters


        param("~min_height", min_height, 0.7);
        param("~max_height", max_height, 1.0);
        param("~frames_no", frames_no, 7);


        param("~display", display, false);
        stringstream ss;
        ss << getenv("ROS_ROOT") << "/../ros-pkg/vision/recognition_lambertian/data/";
        string path = ss.str();
        string template_path;
        param<string>("template_path", template_path, path + "template.png");

        edges_low = 50;
        edges_high = 170;

        if(display){
            cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
//            cvNamedWindow("disparity_original", CV_WINDOW_AUTOSIZE);
        	cvNamedWindow("edges",1);
        	cvCreateTrackbar("edges_low","edges",&edges_low, 500, &on_edges_low);
        	cvCreateTrackbar("edges_high","edges",&edges_high, 500, &on_edges_high);
        }


//        advertise<robot_msgs::PointStamped>("handle_detector/handle_location", 1);
        advertise<visualization_msgs::Marker>("visualization_marker", 1);

        subscribeStereoData();

        cm = new ChamferMatching();

        loadTemplate(template_path);
    }

    ~RecognitionLambertian()
    {
        if(left){
            cvReleaseImage(&left);
        }
        if(right){
            cvReleaseImage(&right);
        }
        if(disp){
            cvReleaseImage(&disp);
        }

        delete cm;

        unsubscribeStereoData();
    }

private:

    void subscribeStereoData()
    {

    	sync.reset();
        std::list<std::string> left_list;
        left_list.push_back(std::string("stereo/left/image_rect_color"));
        left_list.push_back(std::string("stereo/left/image_rect"));
        sync.subscribe(left_list, limage, 1);

        std::list<std::string> right_list;
        right_list.push_back(std::string("stereo/right/image_rect_color"));
        right_list.push_back(std::string("stereo/right/image_rect"));
        sync.subscribe(right_list, rimage, 1);

        sync.subscribe("stereo/disparity", dimage, 1);
//        sync.subscribe("stereo/stereo_info", stinfo, 1);
//        sync.subscribe("stereo/disparity_info", dispinfo, 1);
//        sync.subscribe("stereo/right/cam_info", rcinfo, 1);
        sync.subscribe("stereo/cloud", cloud_fetch, 1);
        sync.ready();
//        sleep(1);
    }

    void unsubscribeStereoData()
    {
        unsubscribe("stereo/left/image_rect_color");
        unsubscribe("stereo/left/image_rect");
        unsubscribe("stereo/right/image_rect_color");
        unsubscribe("stereo/right/image_rect");
        unsubscribe("stereo/disparity");
//        unsubscribe("stereo/stereo_info");
//        unsubscribe("stereo/disparity_info");
//        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");
    }


    void loadTemplate(string path)
    {
    	IplImage* templ = cvLoadImage(path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);

        cm->addTemplateFromImage(templ);

    	cvReleaseImage(&templ);
	}



    void showMatch(IplImage* img, ChamferMatch& match)
    {
    	match.show(img);
    }


    /**
     * \brief Finds edges in an image
     * @param img
     */
    void doChamferMatching(IplImage *img)
    {
    	// edge detection
        IplImage *gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
        cvCvtColor(img, gray, CV_RGB2GRAY);
        cvCanny(gray, gray, edges_high/2, edges_high);

        if (display) {
        	cvShowImage("edges", gray);
        }


        ChamferMatch match = cm->matchEdgeImage(gray);

        IplImage* left_clone = cvCloneImage(left);
        showMatch(left,match);
        if(display){
        	// show filtered disparity
        	cvShowImage("disparity", disp);
        	// show left image
        	cvShowImage("left", left);
        	cvShowImage("right", right);
        }
        cvCopy(left_clone, left);
        cvReleaseImage(&left_clone);

        cvReleaseImage(&gray);
    }





    void runRecognitionLambertian()
    {
        // acquire cv_mutex lock
//        boost::unique_lock<boost::mutex> images_lock(cv_mutex);

        // goes to sleep until some images arrive
//        images_ready.wait(images_lock);
//        printf("Woke up, processing images\n");


        // do useful stuff
    	doChamferMatching(left);

    }





    /**
     * \brief Filters a cloud point, retains only points coming from a specific region in the disparity image
     *
     * @param rect Region in disparity image
     * @return Filtered point cloud
     */
    robot_msgs::PointCloud filterPointCloud(const CvRect & rect)
    {
        robot_msgs::PointCloud result;
        result.header.frame_id = cloud.header.frame_id;
        result.header.stamp = cloud.header.stamp;
        int xchan = -1;
        int ychan = -1;
        for(size_t i = 0;i < cloud.chan.size();++i){
            if(cloud.chan[i].name == "x"){
                xchan = i;
            }
            if(cloud.chan[i].name == "y"){
                ychan = i;
            }
        }

        if(xchan != -1 && ychan != -1){
            for(size_t i = 0;i < cloud.pts.size();++i){
                int x = (int)(cloud.chan[xchan].vals[i]);
                int y = (int)(cloud.chan[ychan].vals[i]);
                if(x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height){
                    result.pts.push_back(cloud.pts[i]);
                }
            }

        }

        return result;
    }


    /**
     * Callback from topic synchronizer, timeout
     * @param t
     */
    void image_cb_timeout(ros::Time t)
    {
        if(limage.header.stamp != t) {
            printf("Timed out waiting for left image\n");
        }

        if(dimage.header.stamp != t) {
            printf("Timed out waiting for disparity image\n");
        }

//        if(stinfo.header.stamp != t) {
//            printf("Timed out waiting for stereo info\n");
//        }

        if(cloud_fetch.header.stamp != t) {
        	printf("Timed out waiting for point cloud\n");
        }
    }


    /**
     * Callback from topic synchronizer, images ready to be consumed
     * @param t
     */
    void image_cb_all(ros::Time t)
    {
        // obtain lock on vision data
        boost::lock_guard<boost::mutex> lock(cv_mutex);

        if(lbridge.fromImage(limage, "bgr")){
            if(left != NULL)
                cvReleaseImage(&left);

            left = cvCloneImage(lbridge.toIpl());
        }
        if(rbridge.fromImage(rimage, "bgr")){
            if(right != NULL)
                cvReleaseImage(&right);

            right = cvCloneImage(rbridge.toIpl());
        }
        if(dbridge.fromImage(dimage)){
            if(disp != NULL)
                cvReleaseImage(&disp);

//            disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
            disp = cvCloneImage(dbridge.toIpl());
//            cvCvtScale(dbridge.toIpl(), disp, 4.0 / dispinfo.dpp);
        }

        cloud = cloud_fetch;

//        images_ready.notify_all();
        runRecognitionLambertian();
    }



public:
	/**
	 * Needed for OpenCV event loop, to show images
	 * @return
	 */
	/**
	 * Needed for OpenCV event loop, to show images
	 * @return
	 */
	bool spin()
	{
		while (ok())
		{
			cv_mutex.lock();
			int key = cvWaitKey(3)&0x00FF;
			if(key == 27) //ESC
				break;

			cv_mutex.unlock();
			usleep(10000);
		}

		return true;
	}

	void triggerEdgeDetection()
	{
		doChamferMatching(left);
	}
};

RecognitionLambertian* node;

void on_edges_low(int value)
{
	node->edges_low = value;
	node->triggerEdgeDetection();
}

void on_edges_high(int value)
{
	node->edges_high = value;
	node->triggerEdgeDetection();
}


int main(int argc, char **argv)
{
	for(int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;

	ros::init(argc, argv);
	node = new RecognitionLambertian();
	node->spin();

	delete node;

	return 0;
}

