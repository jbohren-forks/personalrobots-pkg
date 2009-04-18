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
#include "robot_msgs/Door.h"
#include "robot_msgs/VisualizationMarker.h"
#include "recognition_lambertian/DoorsDetector.h"

#include <string>

// transform library
#include <tf/transform_listener.h>

#include "topic_synchronizer/topic_synchronizer.h"

#include "CvStereoCamModel.h"

#include <boost/thread.hpp>

using namespace std;

template <typename T>
class IndexedIplImage
{
public:
	IplImage* img_;
	T* p;

	IndexedIplImage(IplImage* img) : img_(img)
	{
		p = (T*)img_->imageData;
	}

	operator IplImage*()
	{
		return img_;
	}

	T at(int x, int y, int chan = 0)
	{
		return *(p+y*img_->width+x*img_->nChannels+chan);
	}

	T integral_sum(const CvRect &r)
	{
		return at(r.x+r.width+1,r.y+r.height+1)-at(r.x+r.width+1,r.y)-at(r.x,r.y+r.height+1)+at(r.x,r.y);
	}

};


template<typename T>
class Mat2D
{
public:
	int width_;
	int height_;
	T* data_;

	Mat2D(int width, int height) : width_(width), height_(height)
	{
		data_ = new T[width_*height_];
	}

	~Mat2D()
	{
		delete[] data_;
	}

	T* operator[](int index)
	{
		return data_+index*width_;
	}

};

void on_edges_low(int);
void on_edges_high(int);








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


	typedef pair<int,int> coordinate;
	typedef vector<coordinate> template_coords_t;

	vector<CvSize> template_sizes;
	vector<template_coords_t> template_coords;

	float min_scale;
	float max_scale;
	int count_scale;


	CvHaarClassifierCascade* cascade;
	CvMemStorage* storage;

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


        min_scale = 0.7;
        max_scale = 1.2;
        count_scale = 7;

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
        advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 1);

        subscribeStereoData();

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
        if(storage){
            cvReleaseMemStorage(&storage);
        }

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

    	ROS_INFO("Loading templates");
    	for(int i = 0; i < count_scale; ++i) {
    		float scale = min_scale + (max_scale - min_scale)*i/count_scale;
    		int width = int(templ->width*scale);
    		int height = int(templ->height*scale);

    		printf("Level: %d, scale: %f, width: %d, height: %d\n", i, scale, width, height);

    		IplImage* templ_scale = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    		cvResize(templ, templ_scale, CV_INTER_NN);


    		template_coords_t coords;
        	extractTemplateCoords(templ_scale, coords);
        	template_coords.push_back(coords);
        	template_sizes.push_back(cvSize(width, height));


        	CvPoint offs;
        	offs.x = 0;
        	offs.y = 0;
//        	showMatch(templ_scale, offs,i);

    		cvReleaseImage(&templ_scale);
    	}

    	cvReleaseImage(&templ);
	}


    void extractTemplateCoords(IplImage* templ_img, template_coords_t& coords)
    {
    	coords.clear();
    	unsigned char* ptr = (unsigned char*) templ_img->imageData;
    	for (int y=0;y<templ_img->height;++y) {
    		for (int x=0;x<templ_img->width;++x) {
    			if (*(ptr+y*templ_img->widthStep+x)!=0) {
    				coords.push_back(make_pair(x,y));
    			}
    		}
    	}
    }

    /////////////////////////////////////////////////
    // Analyze the disparity image that values should not be too far off from one another
    // Id  -- 8 bit, 1 channel disparity image
    // R   -- rectangular region of interest
    // vertical -- This is a return that tells whether something is on a wall (descending disparities) or not.
    // minDisparity -- disregard disparities less than this
    //
    double disparitySTD(IplImage *Id, const CvRect & R, double & meanDisparity, double minDisparity = 0.5)
    {
        int ws = Id->widthStep;
        unsigned char *p = (unsigned char*)(Id->imageData);
        int rx = R.x;
        int ry = R.y;
        int rw = R.width;
        int rh = R.height;
        int nchan = Id->nChannels;
        p += ws * ry + rx * nchan; //Put at start of box
        double mean = 0.0, var = 0.0;
        double val;
        int cnt = 0;
        //For vertical objects, Disparities should decrease from top to bottom, measure that
        for(int Y = 0;Y < rh;++Y){
            for(int X = 0;X < rw;X++, p += nchan){
                val = (double)*p;
                if(val < minDisparity)
                    continue;

                mean += val;
                var += val * val;
                cnt++;
            }
            p += ws - (rw * nchan);
        }

        if(cnt == 0){
            return 10000000.0;
        }
        //DO THE VARIANCE MATH
        mean = mean / (double)cnt;
        var = (var / (double)cnt) - mean * mean;
        meanDisparity = mean;
        return (sqrt(var));
    }

    /**
     * \brief Transforms a disparity image pixel to real-world point
     *
     * @param cam_model Camera model
     * @param x coordinate in the disparity image
     * @param y coordinate in the disparity image
     * @param d disparity pixel value
     * @return point in 3D space
     */
    robot_msgs::Point disparityTo3D(CvStereoCamModel & cam_model, int x, int y, double d)
    {
        CvMat *uvd = cvCreateMat(1, 3, CV_32FC1);
        cvmSet(uvd, 0, 0, x);
        cvmSet(uvd, 0, 1, y);
        cvmSet(uvd, 0, 2, d);
        CvMat *xyz = cvCreateMat(1, 3, CV_32FC1);
        cam_model.dispToCart(uvd, xyz);
        robot_msgs::Point result;
        result.x = cvmGet(xyz, 0, 0);
        result.y = cvmGet(xyz, 0, 1);
        result.z = cvmGet(xyz, 0, 2);
        return result;
    }

    /**
	 * \brief Computes distance between two 3D points
	 *
	 * @param a
	 * @param b
	 * @return
	 */
    double distance3D(robot_msgs::Point a, robot_msgs::Point b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }

    /**
     * \brief Computes size and center of ROI in real-world
     *
     * Given a disparity images and a ROI in the image this function computes the approximate real-world size
     * and center of the ROI.
     *
     * This function is just an approximation, it uses the mean value of the disparity in the ROI and assumes
     * the ROI is flat region perpendicular on the camera z axis. It could be improved by finding a dominant plane
     * in the and using only those disparity values.
     *
     * @param R
     * @param meanDisparity
     * @param dx
     * @param dy
     * @param center
     */
    void getROIDimensions(const CvRect& r, double & dx, double & dy, robot_msgs::Point & center)
    {
        // initialize stereo camera model
        double Fx = rcinfo.P[0];
        double Fy = rcinfo.P[5];
        double Clx = rcinfo.P[2];
        double Crx = Clx;
        double Cy = rcinfo.P[6];
        double Tx = -rcinfo.P[3] / Fx;
        CvStereoCamModel cam_model(Fx, Fy, Tx, Clx, Crx, Cy, 4.0 / (double)dispinfo.dpp);

        double mean = 0;
        disparitySTD(disp, r, mean);

        robot_msgs::Point p1 = disparityTo3D(cam_model, r.x, r.y, mean);
        robot_msgs::Point p2 = disparityTo3D(cam_model, r.x + r.width, r.y, mean);
        robot_msgs::Point p3 = disparityTo3D(cam_model, r.x, r.y + r.height, mean);
        center = disparityTo3D(cam_model, r.x + r.width / 2, r.y + r.height / 2, mean);
        dx = distance3D(p1, p2);
        dy = distance3D(p1, p3);
    }



    float localChamferDistance(IplImage* dist_img, const vector<int>& templ_addr, CvPoint offset)
    {
    	int x = offset.x;
    	int y = offset.y;
    	float sum = 0;

    	float* ptr = (float*) dist_img->imageData;
    	ptr += (y*dist_img->width+x);
    	for (size_t i=0;i<templ_addr.size();++i) {
    		sum += *(ptr+templ_addr[i]);
    	}
    	return sum/templ_addr.size();

//    	IndexedIplImage<float> dist(dist_img);
//    	for (size_t i=0;i<templ_coords.size();++i) {
//    		int px = x+templ_coords[i].first;
//    		int py = y+templ_coords[i].second;
//    		if (px<dist_img->width && py<dist_img->height)
//    			sum += dist.at(px,py);
//    	}
//    	return sum/templ_coords.size();
    }

    void matchTemplate(IplImage* dist_img, const template_coords_t& coords, CvSize template_size, CvPoint& offset, float& dist)
    {
    	int width = dist_img->width;
    	vector<int> templ_addr;
    	templ_addr.clear();
    	for (size_t i= 0; i<coords.size();++i) {
    		templ_addr.push_back(coords[i].second*width+coords[i].first);
    	}

    	// sliding window
    	for (int y=0;y<dist_img->height - template_size.height; y+=2) {
    		for (int x=0;x<dist_img->width - template_size.width; x+=2) {
				CvPoint test_offset;
				test_offset.x = x;
				test_offset.y = y;
				float test_dist = localChamferDistance(dist_img, templ_addr, test_offset);

				if (test_dist<dist) {
					dist = test_dist;
					offset = test_offset;
				}
    		}
    	}
    }


    void matchTemplateScale(IplImage* dist_img, CvPoint& offset, float& dist, int& scale)
    {
    	for(int i = 0; i < count_scale; i++) {
    		CvPoint test_offset;
            test_offset.x = 0;
            test_offset.y = 0;
    		float test_dist = 1e10;

    		matchTemplate(dist_img, template_coords[i], template_sizes[i], test_offset, test_dist);
			if (test_dist<dist) {
				dist = test_dist;
				offset = test_offset;
				scale = i;
			}
    	}
    }


    void showMatch(IplImage* img, CvPoint& offset, int scale)
    {
    	unsigned char* ptr = (unsigned char*) img->imageData;
    	template_coords_t& templ_coords = template_coords[scale];
    	for (size_t i=0;i<templ_coords.size();++i) {
    		int x = offset.x + templ_coords[i].first;
    		int y = offset.y + templ_coords[i].second;
    		(ptr+y*img->widthStep+x*img->nChannels)[1] = 255;
    	}
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


//    	Mat2D<int> dt(left->width, left->height);
        IplImage* dist_img = cvCreateImage(cvSize(left->width, left->height), IPL_DEPTH_32F, 1);

        computeDistanceTransform2(gray,dist_img, -1);
//    	computeDistanceTransform(gray,dt,20);

        CvPoint offset;
        offset.x = 0;
        offset.y = 0;
        float dist = 1e10;
        int scale = 0;
        matchTemplateScale(dist_img, offset, dist, scale);
//        matchTemplate(dist_img, offset, dist);



        IplImage* left_clone = cvCloneImage(left);
        printf("Scale: %d\n", scale);
        showMatch(left,offset, scale);


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
     *
     * @param edges_img
     * @param dist_img - IPL_DEPTH_32F image
     */
    void computeDistanceTransform2(IplImage* edges_img, IplImage* dist_img, float truncate)
    {
    	cvNot(edges_img, edges_img);
    	cvDistTransform(edges_img, dist_img);
    	cvNot(edges_img, edges_img);

    	if (truncate>0) {
    		cvMinS(dist_img, truncate, dist_img);
    	}
    }


    void computeDistanceTransform(IplImage* edges_img, Mat2D<int>& dt, int truncate)
    {
    	int d[][2] = { {-1,-1},{ 0,-1},{ 1,-1},
					  {-1,0},          { 1,0},
					  {-1,1}, { 0,1},  { 1,1} };

    	ROS_INFO("Computing distance transform");

    	CvSize s = cvGetSize(edges_img);
    	int w = s.width;
    	int h = s.height;
    	for (int i=0;i<h;++i) {
    		for (int j=0;j<w;++j) {
    			dt[i][j] = -1;
    		}
    	}

    	queue<pair<int,int> > q;
    	// initialize queue
    	IndexedIplImage<unsigned char> edges(edges_img);
    	for (int y=0;y<h;++y) {
    		for (int x=0;x<w;++x) {
    			if (edges.at(x,y)!=0) {
    				q.push(make_pair(x,y));
    				dt[y][x] = 0;
    			}
    		}
    	}

    	pair<int,int> crt;
    	while (!q.empty()) {
    		crt = q.front();
    		q.pop();

    		int x = crt.first;
    		int y = crt.second;
    		int dist = dt[y][x]+1;
    		for (size_t i=0;i<sizeof(d)/sizeof(d[0]);++i) {
    			int nx = x + d[i][0];
    			int ny = y + d[i][1];

    			if (nx<0 || ny<0 || nx>w || ny>h) continue;

    			if (dt[ny][nx]==-1 || dt[ny][nx]>dist) {
    				dt[ny][nx] = dist;
    				q.push(make_pair(nx,ny));
    			}
    		}
    	}

    	// truncate dt
    	if (truncate>0) {
    		for (int i=0;i<h;++i) {
    			for (int j=0;j<w;++j) {
    				dt[i][j] = min( dt[i][j],truncate);
    			}
    		}
    	}


//    	int f = 1;
//    	if (truncate>0) f = 255/truncate;
//    	// display image
//    	IplImage *dt_image = cvCreateImage(s, IPL_DEPTH_8U, 1);
//		unsigned char* dt_p = (unsigned char*)dt_image->imageData;
//
//		for (int i=0;i<w*h;++i) {
//			dt_p[i] = f*dt.data_[i];
//    	}
//
//    	cvNamedWindow("dt",1);
//    	cvShowImage("dt",dt_image);
//    	cvReleaseImage(&dt_image);

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

