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


#include "image_msgs/CvBridge.h"

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

#include <string>

// transform library
#include <tf/transform_listener.h>

#include "topic_synchronizer.h"

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

class HandleDetector : public ros::Node
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

	robot_msgs::PointCloud cloud;
	robot_msgs::Door door;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

	CvScalar door_plane;
	bool have_door_plane;
	bool pause;

	string destination_frame;

	TopicSynchronizer<HandleDetector> sync;

	boost::mutex cv_mutex;

	tf::TransformListener *tf_;


	CvHaarClassifierCascade* cascade;
	CvMemStorage* storage;

	HandleDetector() : ros::Node("stereo_view"),
		left(NULL), right(NULL), disp(NULL), disp_clone(NULL), pause(false),
		sync(this, &HandleDetector::image_cb_all, ros::Duration().fromSec(0.05), &HandleDetector::image_cb_timeout)

	{
		tf_ = new tf::TransformListener(*this);

		cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
		//cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
//		cvNamedWindow("contours", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("disparity_original", CV_WINDOW_AUTOSIZE);


		std::list<std::string> left_list;
		left_list.push_back(std::string("stereo/left/image_rect_color"));
		left_list.push_back(std::string("stereo/left/image_rect"));

//		std::list<std::string> right_list;
//		right_list.push_back(std::string("stereo/right/image_rect_color"));
//		right_list.push_back(std::string("stereo/right/image_rect"));

		sync.subscribe(left_list,  limage, 1);
//		sync.subscribe(right_list, rimage, 1);

		sync.subscribe("stereo/disparity", dimage, 1);
		sync.subscribe("stereo/stereo_info", stinfo, 1);
		sync.subscribe("stereo/disparity_info", dispinfo, 1);
		sync.subscribe("stereo/right/cam_info", rcinfo, 1);

		sync.subscribe("stereo/cloud", cloud, 1);
		sync.ready();

		have_door_plane = false;

		subscribe("/door_detector/door_msg", door, &HandleDetector::door_detected,1);

		string handle_features_file;
        string background_features_file;
        stringstream ss;
        ss << getenv("ROS_ROOT") << "/../ros-pkg/vision/door_handle_detect/data/";
        string path = ss.str();

        string cascade_classifier;
        param<string>("cascade_classifier", cascade_classifier, path+"handles_data.xml");
        ROS_INFO("Loading cascade classifier\n");
        cascade = (CvHaarClassifierCascade*)cvLoad( cascade_classifier.c_str(), 0, 0, 0 );

        if (!cascade) {
        	ROS_ERROR("Cannot load cascade classifier\n");
        }
        storage = cvCreateMemStorage(0);


        param<string>("destination_frame", destination_frame, "base_link");
        advertise<robot_msgs::PointStamped>("handle_detector/handle_location",1);
        advertise<robot_msgs::VisualizationMarker>("visualizationMarker",1);
	}

	~HandleDetector()
	{
		if (left) {
			cvReleaseImage(&left);
		}
		if (right) {
			cvReleaseImage(&right);
		}
		if (disp) {
			cvReleaseImage(&disp);
		}
		if (storage) {
			cvReleaseMemStorage(&storage);
		}
	}

private:
	/////////////////////////////////////////////////
	// Analyze the disparity image that values should not be too far off from one another
	// Id  -- 8 bit, 1 channel disparity image
	// R   -- rectangular region of interest
	// vertical -- This is a return that tells whether something is on a wall (descending disparities) or not.
	// minDisparity -- disregard disparities less than this
	//
	double disparitySTD(IplImage *Id, CvRect &R, double& meanDisparity, double minDisparity = 0.5 )
	{
		int ws = Id->widthStep;
		unsigned char *p = (unsigned char *)(Id->imageData);
		int rx = R.x;
		int ry = R.y;
		int rw = R.width;
		int rh = R.height;
		int nchan = Id->nChannels;
		p += ws*ry + rx*nchan; //Put at start of box
		double mean = 0.0,var = 0.0;
		double val;
		int cnt = 0;
		//For vertical objects, Disparities should decrease from top to bottom, measure that
		for(int Y=0; Y<rh; ++Y)
		{
			for(int X=0; X<rw; X++, p+=nchan)
			{
				val = (double)*p;
				if(val < minDisparity)
					continue;
				mean += val;
				var += val*val;
				cnt++;
			}
			p+=ws-(rw*nchan);
		}
		if(cnt == 0) //Error condition, no disparities, return impossible variance
		{
			return 10000000.0;
		}
		//DO THE VARIANCE MATH
		mean = mean/(double)cnt;
		var = (var/(double)cnt) - mean*mean;
		meanDisparity = mean;
		return(sqrt(var));
	}


	robot_msgs::Point disparityTo3D(CvStereoCamModel& cam_model, int x, int y, double d)
	{
		CvMat* uvd = cvCreateMat(1,3,CV_32FC1);
		cvmSet(uvd,0,0,x);
		cvmSet(uvd,0,1,y);
		cvmSet(uvd,0,2,d);
		CvMat* xyz = cvCreateMat(1,3,CV_32FC1);
		cam_model.dispToCart(uvd,xyz);
		robot_msgs::Point result;
		result.x = cvmGet(xyz,0,0);
		result.y = cvmGet(xyz,0,1);
		result.z = cvmGet(xyz,0,2);
		return result;
	}


	/**
	 *
	 * @param a
	 * @param b
	 * @return
	 */
	double distance3D(robot_msgs::Point a, robot_msgs::Point b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
	}

	void get_bb_dimensions(IplImage *Id, CvRect &R, double meanDisparity, double& dx, double& dy, robot_msgs::Point& center)
	{
		// initialize stereo camera model
		double Fx = rcinfo.P[0];  double Fy = rcinfo.P[5];
		double Clx = rcinfo.P[2]; double Crx = Clx;
		double Cy = rcinfo.P[6];
		double Tx = -rcinfo.P[3]/Fx;
		CvStereoCamModel cam_model(Fx,Fy,Tx,Clx,Crx,Cy,4.0/(double)dispinfo.dpp);


		int ws = Id->widthStep;
		unsigned char *p = (unsigned char *)(Id->imageData);
		int rx = R.x;
		int ry = R.y;
		int rw = R.width;
		int rh = R.height;
		int nchan = Id->nChannels;
		p += ws*ry + rx*nchan; //Put at start of box

		robot_msgs::Point p1 = disparityTo3D(cam_model, rx, ry, meanDisparity);
		robot_msgs::Point p2 = disparityTo3D(cam_model, rx+rw, ry, meanDisparity);
		robot_msgs::Point p3 = disparityTo3D(cam_model, rx, ry+rh, meanDisparity);

		center = disparityTo3D(cam_model, rx+rw/2, ry+rh/2, meanDisparity);

		dx = distance3D(p1,p2);
		dy = distance3D(p1,p3);

	}


	bool handlePossibleHere(CvRect& r)
	{
		const float nz_fraction = 0.1;

		cvSetImageROI(disp,r);
		cvSetImageCOI(disp,	1);
		int cnt = cvCountNonZero(disp);
		cvResetImageROI(disp);
		cvSetImageCOI(disp,0);

		double mean;
		disparitySTD(disp, r, mean);

		if (cnt<nz_fraction*r.width*r.height) {
//			ROS_INFO("Too few pixels in the disparity, discarding");
			return false;
		}

		robot_msgs::PointCloud pc = filterPointCloud(r);
		CvScalar plane = estimatePlaneLS(pc);
		cnt = 0;
		double sum = 0;
		double max_dist = 0;
		for (size_t i = 0; i<pc.pts.size();++i) {
			robot_msgs::Point32 p = pc.pts[i];
			double dist = fabs(plane.val[0]*p.x+plane.val[1]*p.y+plane.val[2]*p.z+plane.val[3]);
			max_dist = max(max_dist,dist);
			sum += dist;
			cnt++;
		}
		sum /= cnt;

		printf("Average distance to plane: %f, max: %f\n", sum, max_dist);

		if (max_dist>0.1 || sum<0.005) {
//			cvRectangle(left, cvPoint(r.x,r.y), cvPoint(r.x+r.width, r.y+r.height), CV_RGB(0,0,255));
			return false;
		}


		double dx, dy;
		robot_msgs::Point p;
		get_bb_dimensions(disp, r, mean, dx, dy,p);

		if (dx>0.25 || dy>0.15) {
			ROS_INFO("Too big, discarding");
			return false;
		}

		robot_msgs::PointStamped pin, pout;
		pin.header.frame_id = cloud.header.frame_id;
		pin.header.stamp = cloud.header.stamp;
		pin.point.x = p.z;
		pin.point.y = -p.x;
		pin.point.z = -p.y;


		tf_->transformPoint("base_link", pin, pout);

//		printf("r: (%d, %d, %d, %d), sdv: %f, dx: %f, dy: %f, x: %f, y: %f, z: %f\n", r.x, r.y, r.width, r.height, sdv, dx, dy,
//					pout.point.x, pout.point.y, pout.point.z);

		if (pout.point.z>0.9 || pout.point.z<0.7) {
			ROS_INFO("Too high, discarding");
			return false;

		}


		// publish handle location
		tf_->transformPoint(destination_frame, pin, pout);
		publish("handle_detector/handle_location", pout);


		robot_msgs::VisualizationMarker marker;
        marker.header.frame_id = destination_frame;
        marker.header.stamp = ros::Time((uint64_t)0ULL);
        marker.id = 0;
        marker.type = robot_msgs::VisualizationMarker::SPHERE;
        marker.action = robot_msgs::VisualizationMarker::ADD;
        marker.x = pout.point.x;
        marker.y = pout.point.y;
        marker.z = pout.point.z;
        marker.yaw = 0.0;
        marker.pitch = 0.0;
        marker.roll = 0.0;
        marker.xScale = 0.1;
        marker.yScale = 0.1;
        marker.zScale = 0.1;
        marker.alpha = 255;
        marker.r = 0;
        marker.g = 255;
        marker.b = 0;

        publish( "visualizationMarker", marker );



		return true;
	}


	void findHandleCascade( )
	{

	    IplImage *gray = cvCreateImage( cvSize(left->width,left->height), 8, 1 );

	    cvCvtColor( left, gray, CV_BGR2GRAY );
	    cvEqualizeHist( gray, gray );
	    cvClearMemStorage( storage );

	    if( cascade )
	    {
	        CvSeq* handles = cvHaarDetectObjects( gray, cascade, storage,
	                                            1.1, 2, 0
	                                            //|CV_HAAR_FIND_BIGGEST_OBJECT
	                                            //|CV_HAAR_DO_ROUGH_SEARCH
	                                            //|CV_HAAR_DO_CANNY_PRUNING
	                                            //|CV_HAAR_SCALE_IMAGE
	                                            ,
	                                            cvSize(10, 10) );
	        for(int i = 0; i < (handles ? handles->total : 0); i++ )
	        {
	            CvRect* r = (CvRect*)cvGetSeqElem( handles, i );
	            if (handlePossibleHere(*r)) {
	            	cvRectangle(left, cvPoint(r->x,r->y), cvPoint(r->x+r->width, r->y+r->height), CV_RGB(0,255,0));
	            }
	            else {
//	            	cvRectangle(left, cvPoint(r->x,r->y), cvPoint(r->x+r->width, r->y+r->height), CV_RGB(255,0,0));
	            }
	        }
	    }

	    cvReleaseImage( &gray );
	}


	void image_cb_all(ros::Time t)
	{
		if (pause)  return;
		cv_mutex.lock();

		if (lbridge.fromImage(limage, "bgr"))
		{
			if(left != NULL)
				cvReleaseImage(&left);
			left = cvCloneImage(lbridge.toIpl());
		}

		if (rbridge.fromImage(rimage, "bgr"))
		{
			if(right != NULL)
				cvReleaseImage(&right);

			right = cvCloneImage(rbridge.toIpl());
		}

		if (dbridge.fromImage(dimage))
		{
			if(disp != NULL)
				cvReleaseImage(&disp);

			disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
			cvCvtScale(dbridge.toIpl(), disp, 4.0/dispinfo.dpp);
		}

		cvShowImage("left", left);
		//cvShowImage("right", right);
//		cvShowImage("disparity", disp_clone);

		cvShowImage("disparity_original", disp);


		applyPositionPrior(disp);
		cvShowImage("disparity", disp);
//		findEdges(left);
		findHandleCascade();


//		cvShowImage("disparity", disp);
		cvShowImage("left", left);

		if (disp_clone!=NULL)
			cvReleaseImage(&disp_clone);

		cv_mutex.unlock();

//		pause = true;
	}



	void door_detected()
	{
		door_plane.val[0] = door.normal.x;
		door_plane.val[1] = door.normal.y;
		door_plane.val[2] = door.normal.z;
		door_plane.val[3] = -(door.normal.x*door.door_p1.x+door.normal.y*door.door_p1.y+door.normal.z*door.door_p1.z);

		printf("Door plane: %f, %f, %f, %f\n", door_plane.val[0],door_plane.val[1],door_plane.val[2],door_plane.val[3]);

		have_door_plane = true;
	}

	void findEdges(IplImage* img)
	{
		IplImage* gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvCvtColor(img, gray, CV_RGB2GRAY);
		cvCanny(gray,gray,20,40);

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* lines = 0;
		lines = cvHoughLines2( gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/360, 100, 200, 100 );
		for( int i = 0; i < lines->total; i++ )
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			CvPoint p1 = line[0];
			CvPoint p2 = line[1];


			if (abs(p1.x-p2.x)>0) {
				float min_angle = 80;
				float slope = float(abs(p1.y-p2.y))/abs(p1.x-p2.x);
				float min_slope = tan(CV_PI/2-min_angle*CV_PI/180);
				if (slope<min_slope) continue;
				printf("slope: %f, min_slope: %f\n",slope, min_slope);
			}

			cvLine( left, p1, p2, CV_RGB(255,0,0), 2, CV_AA, 0 );
		}


		cvShowImage("contours", gray);

		cvReleaseImage(&gray);
	}


	void image_cb_timeout(ros::Time t)
	{
		if (limage.header.stamp != t)
			printf("Timed out waiting for left image\n");

//		if (rimage.header.stamp != t)
//			printf("Timed out waiting for right image\n");

		if (dimage.header.stamp != t)
			printf("Timed out waiting for disparity image\n");

		if (stinfo.header.stamp != t)
			printf("Timed out waiting for stereo info\n");
//
		if (cloud.header.stamp != t)
			printf("Timed out waiting for point cloud\n");

	}


	robot_msgs::PointCloud filterPointCloud(const CvRect& rect)
	{
		robot_msgs::PointCloud result;

		int xchan = -1;
		int ychan = -1;

		for (size_t i=0;i<cloud.chan.size();++i) {
			if (cloud.chan[i].name == "x") {
				xchan = i;
			}
			if (cloud.chan[i].name == "y") {
				ychan = i;
			}
		}

		if (xchan!=-1 && ychan!=-1) {
			for (size_t i=0;i<cloud.pts.size();++i) {
				int x = (int)cloud.chan[xchan].vals[i];
				int y = (int)cloud.chan[ychan].vals[i];
				if (x>=rect.x && x<rect.x+rect.width && y>=rect.y && y<rect.y+rect.height) {
					result.pts.push_back(cloud.pts[i]);
				}
			}
		}

		return result;
	}

//
//	// returns the plane in Hessian normal form
	CvScalar estimatePlaneLS(robot_msgs::PointCloud points)
	{
		int cnt = points.pts.size();
		CvMat* A = cvCreateMat(cnt, 3, CV_32FC1);

		for (int i=0;i<cnt;++i) {
			robot_msgs::Point32 p = points.pts[i];
			cvmSet(A,i,0,p.x);
			cvmSet(A,i,1,p.y);
			cvmSet(A,i,2,p.z);
		}

		vector<float> ones(cnt,1);
		CvMat B;
		cvInitMatHeader(&B,cnt,1,CV_32FC1,&ones[0]);
		CvMat* X = cvCreateMat(3,1,CV_32FC1);

		int ok = cvSolve( A, &B, X,CV_SVD );

		CvScalar plane;

		if (ok) {
			float* xp = X->data.fl;

			float d = sqrt(xp[0]*xp[0]+xp[1]*xp[1]+xp[2]*xp[2]);
			plane.val[0] = xp[0]/d;
			plane.val[1] = xp[1]/d;
			plane.val[2] = xp[2]/d;
			plane.val[3] = -1/d;
		}
		else {
			plane.val[0] = plane.val[1] = plane.val[2] = plane.val[3] = -1;
		}

		cvReleaseMat(&A);

		return plane;
	}

	void applyPositionPrior(IplImage* disp)
	{
		robot_msgs::PointCloud base_cloud;

		try {
			tf_->transformPointCloud("base_link", cloud, base_cloud);
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_WARN("TF exception: %s", ex.what());
	    }


		int xchan = -1;
		int ychan = -1;

		for (size_t i=0;i<base_cloud.chan.size();++i) {
			if (base_cloud.chan[i].name == "x") {
				xchan = i;
			}
			if (base_cloud.chan[i].name == "y") {
				ychan = i;
			}
		}

		if (xchan!=-1 && ychan!=-1) {

			unsigned char *pd = (unsigned char *)(disp->imageData);
			int ws = disp->widthStep;
			for (size_t i=0;i<base_cloud.get_pts_size();++i) {
				robot_msgs::Point32 crt_point = base_cloud.pts[i];

				int x = (int)base_cloud.chan[xchan].vals[i];
				int y = (int)base_cloud.chan[ychan].vals[i];

				// pointer to the current pixel
				unsigned char* crt_pd = pd+y*ws+x;
				if (crt_point.z>1.00 || crt_point.z<0.70) {
					*crt_pd = 0;
				}

				if (*crt_pd>0 && have_door_plane) {
					// distance from the door plane
					double distance = crt_point.x*door_plane.val[0]+crt_point.y*door_plane.val[1]+crt_point.z*door_plane.val[2]+door_plane.val[3];

					if (distance<0.01 || distance>0.1) {
						*crt_pd = 0;
					}
				}

			}
		}
		else {
			ROS_WARN("I can't find image coordinates in the point cloud, no filtering done.");
		}
	}

public:
	bool spin()
	{
		while (ok())
		{
			cv_mutex.lock();
			int key = cvWaitKey(3)&0x00FF;
			if(key == 27) //ESC
				break;

			if (key=='p')
				pause = false;

			cv_mutex.unlock();
			usleep(10000);
		}

		return true;
	}
};

int main(int argc, char **argv)
{
	for(int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;

	ros::init(argc, argv);
	HandleDetector view;
	view.spin();

	return 0;
}

