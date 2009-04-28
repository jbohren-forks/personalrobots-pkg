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
#include "door_handle_detector/DoorsDetector.h"
#include "std_srvs/Empty.h"

#include <string>

// transform library
#include <tf/transform_listener.h>

#include <topic_synchronizer/topic_synchronizer.h>

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
		return at(r.x+r.width,r.y+r.height)-at(r.x+r.width,r.y)-at(r.x,r.y+r.height)+at(r.x,r.y);
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

	robot_msgs::PointCloud cloud_fetch;
	robot_msgs::PointCloud cloud;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

	TopicSynchronizer<HandleDetector> sync;

	boost::mutex cv_mutex;
	boost::condition_variable images_ready;

	tf::TransformListener *tf_;


	// minimum height to look at (in base_footprint frame)
	double min_height;
	// maximum height to look at (in base_footprint frame)
	double max_height;
	// no. of frames to detect handle in
	int frames_no;
	// display stereo images ?
	bool display;

	bool preempted_;
	bool got_images_;


	double image_timeout_;
	ros::Time start_image_wait_;


	CvHaarClassifierCascade* cascade;
	CvMemStorage* storage;

    HandleDetector()
    :ros::Node("stereo_view"), left(NULL), right(NULL), disp(NULL), disp_clone(NULL), sync(this, &HandleDetector::image_cb_all, ros::Duration().fromSec(0.1), &HandleDetector::image_cb_timeout)
    {
        tf_ = new tf::TransformListener(*this);
        // define node parameters


        param("~min_height", min_height, 0.7);
        param("~max_height", max_height, 1.0);
        param("~frames_no", frames_no, 10);
        param("~timeout", image_timeout_, 3.0);		// timeout (in seconds) until an image must be received, otherwise abort


        param("~display", display, false);
        stringstream ss;
        ss << getenv("ROS_ROOT") << "/../ros-pkg/mapping/door_handle_detector/data/";
        string path = ss.str();
        string cascade_classifier;
        param<string>("~cascade_classifier", cascade_classifier, path + "handles_data.xml");

        if(display){
            cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
            //cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("disparity_original", CV_WINDOW_AUTOSIZE);
        }

//        subscribeStereoData();

        // load a cascade classifier
        loadClassifier(cascade_classifier);
        // invalid location until we get a detection

//        advertise<robot_msgs::PointStamped>("handle_detector/handle_location", 1);
        advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 1);
        advertiseService("door_handle_vision_detector", &HandleDetector::detectHandleSrv, this);
        advertiseService("door_handle_vision_preempt", &HandleDetector::preempt, this);
    }

    ~HandleDetector()
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
        unadvertise("visualizationMarker");
        unadvertiseService("door_handle_vision_detector");
        unadvertiseService("door_handle_vision_preempt");
    }

private:
    void loadClassifier(string cascade_classifier)
    {
        //		subscribe("/door_detector/door_msg", door, &HandleDetector::door_detected,1);
        ROS_INFO("Loading cascade classifier: %s", cascade_classifier.c_str());
        cascade = (CvHaarClassifierCascade*)(cvLoad(cascade_classifier.c_str(), 0, 0, 0));
        if(!cascade){
            ROS_ERROR("Cannot load cascade classifier\n");
        }
        storage = cvCreateMemStorage(0);
    }

    void subscribeStereoData()
    {

    	sync.reset();
        std::list<std::string> left_list;
        left_list.push_back(std::string("stereo/left/image_rect_color"));
        left_list.push_back(std::string("stereo/left/image_rect"));
        sync.subscribe(left_list, limage, 1);

        //		std::list<std::string> right_list;
        //		right_list.push_back(std::string("stereo/right/image_rect_color"));
        //		right_list.push_back(std::string("stereo/right/image_rect"));
        //		sync.subscribe(right_list, rimage, 1);

        sync.subscribe("stereo/disparity", dimage, 1);
        sync.subscribe("stereo/stereo_info", stinfo, 1);
        sync.subscribe("stereo/disparity_info", dispinfo, 1);
        sync.subscribe("stereo/right/cam_info", rcinfo, 1);
        sync.subscribe("stereo/cloud", cloud_fetch, 1);
        sync.ready();
//        sleep(1);
    }

    void unsubscribeStereoData()
    {
        unsubscribe("stereo/left/image_rect_color");
        unsubscribe("stereo/left/image_rect");
        unsubscribe("stereo/disparity");
        unsubscribe("stereo/stereo_info");
        unsubscribe("stereo/disparity_info");
        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");
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


    /**
     * \brief Publishes a visualization marker for a point.
     * @param p
     */
    void showHandleMarker(robot_msgs::PointStamped p)
    {
        robot_msgs::VisualizationMarker marker;
        marker.header.frame_id = p.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)(0ULL));
        marker.id = 0;
        marker.type = robot_msgs::VisualizationMarker::SPHERE;
        marker.action = robot_msgs::VisualizationMarker::ADD;
        marker.x = p.point.x;
        marker.y = p.point.y;
        marker.z = p.point.z;
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
        publish("visualizationMarker", marker);
    }



//    void showPlaneMarker(CvScalar plane, robot_msgs::PointCloud pc)
//    {
//
//    	double min_d = 10000;
//    	int min_i = -1;
//    	for (size_t i=0;i<pc.pts.size();++i) {
//			float dist = fabs(plane.val[0]*pc.pts[i].x+plane.val[1]*pc.pts[i].y+plane.val[2]*pc.pts[i].z+plane.val[3]);
//			if (dist<min_d) {
//				min_d = dist;
//				min_i = i;
//			}
//    	}
//
//
//        robot_msgs::VisualizationMarker marker;
//        marker.header.frame_id = pc.header.frame_id;
//        marker.header.stamp = pc.header.stamp;
//        marker.id = 1211;
//        marker.type = robot_msgs::VisualizationMarker::SPHERE;
//        marker.action = robot_msgs::VisualizationMarker::ADD;
//        marker.x = pc.pts[min_i].x;
//        marker.y = pc.pts[min_i].y;
//        marker.z = pc.pts[min_i].z;
//        marker.yaw = 0.0;
//        marker.pitch = 0.0;
//        marker.roll = 0.0;
//        marker.xScale = 0.1;
//        marker.yScale = 0.1;
//        marker.zScale = 0.1;
//        marker.alpha = 255;
//        marker.r = 255;
//        marker.g = 0;
//        marker.b = 0;
//        publish("visualizationMarker", marker);
//
//        printf("Show marker at: (%f,%f,%f)", marker.x, marker.y, marker.z);
//    }



    CvPoint getDisparityCenter(CvRect& r)
    {
		CvMoments moments;
		double M00, M01, M10;

        cvSetImageROI(disp, r);
        cvSetImageCOI(disp, 1);
    	cvMoments(disp,&moments,1);
    	M00 = cvGetSpatialMoment(&moments,0,0);
    	M10 = cvGetSpatialMoment(&moments,1,0);
    	M01 = cvGetSpatialMoment(&moments,0,1);
        cvResetImageROI(disp);
        cvSetImageCOI(disp, 0);

        CvPoint center;
        center.x = r.x+(int)(M10/M00);
        center.y = r.y+(int)(M01/M00);

        return center;
    }


    void tryShrinkROI(CvRect& r)
    {
        cvSetImageROI(disp, r);
    	IplImage* integral_patch = cvCreateImage(cvSize(r.width+1, r.height+1), IPL_DEPTH_32S, 1);
    	cvIntegral(disp, integral_patch);
    	IndexedIplImage<int> ipatch(integral_patch);

    	CvRect r2 = r;
    	CvRect p;
    	p.x = 0;
    	p.y = 0;
    	p.height = r.height;
    	p.width = 1;
    	while (ipatch.integral_sum(p)==0 && p.x<r.width) {
    		p.x++;
    	}
    	r2.x = r.x+p.x;

    	p.x = r.width-1;
    	while (ipatch.integral_sum(p)==0 && p.x>0) {
    		p.x--;
    	}
    	r2.width = r.x-r2.x+p.x;

    	p.x = 0;
    	p.y = 0;
    	p.height = 1;
    	p.width = r.width;
    	while (ipatch.integral_sum(p)==0 && p.y<r.height) {
    		p.y++;
    	}
    	r2.y = r.y+p.y;

    	p.y = r.height-1;
    	while (ipatch.integral_sum(p)==0 && p.y>0) {
    		p.y--;
    	}
    	r2.height = r.y-r2.y+p.y;

    	r = r2;
        cvResetImageROI(disp);
        cvReleaseImage(&integral_patch);

    }

    /**
     * \brief Determine if it's possible for handle to be in a specific ROI
     *
     * It looks at things like, height, approximate size of ROI, how flat it is.
     *
     * @param r
     * @return
     */
    bool handlePossibleHere(CvRect &r)
    {

        tryShrinkROI(r);

        if (r.width<10 || r.height<10) {
        	return false;
        }

        cvSetImageROI(disp, r);
        cvSetImageCOI(disp, 1);
        int cnt;
        const float nz_fraction = 0.1;
        cnt = cvCountNonZero(disp);
        if (cnt < nz_fraction * r.width * r.height){
        		cvResetImageROI(disp);
        		cvSetImageCOI(disp, 0);
        		return false;
        }
        cvResetImageROI(disp);
        cvSetImageCOI(disp, 0);


        // compute least-squares handle plane
        robot_msgs::PointCloud pc = filterPointCloud(r);
        CvScalar plane = estimatePlaneLS(pc);

        cnt = 0;
        double sum = 0;
        double max_dist = 0;
        for(size_t i = 0;i < pc.pts.size();++i){
            robot_msgs::Point32 p = pc.pts[i];
            double dist = fabs(plane.val[0] * p.x + plane.val[1] * p.y + plane.val[2] * p.z + plane.val[3]);
            max_dist = max(max_dist, dist);
            sum += dist;
            cnt++;
        }
        sum /= cnt;
        if(max_dist > 0.1 || sum < 0.002){
        	ROS_INFO("Not enough depth variation for handle candidate: %f, %f\n", max_dist, sum);
            return false;
        }

        double dx, dy;
        robot_msgs::Point p;
        getROIDimensions(r, dx, dy, p);
        if(dx > 0.25 || dy > 0.15){
            ROS_INFO("Too big, discarding");
            return false;
        }

        robot_msgs::PointStamped pin, pout;
        pin.header.frame_id = cloud.header.frame_id;
        pin.header.stamp = cloud.header.stamp;
        pin.point.x = p.x;
        pin.point.y = p.y;
        pin.point.z = p.z;
        try {
            tf_->transformPoint("base_footprint", pin, pout);
        }
        catch(tf::LookupException & ex){
            ROS_ERROR("Lookup exception: %s\n", ex.what());
        }
        catch(tf::ExtrapolationException & ex){
            ROS_DEBUG("Extrapolation exception: %s\n", ex.what());
        }
        catch(tf::ConnectivityException & ex){
            ROS_ERROR("Connectivity exception: %s\n", ex.what());
        }

        if(pout.point.z > max_height || pout.point.z < min_height){
        	printf("Height not within admissable range: %f\n", pout.point.z);
            return false;
        }

        printf("Handle at: (%d,%d,%d,%d)\n", r.x,r.y,r.width, r.height);


        return true;
    }

    /**
     * \brief Start handle detection
     */
    void findHandleCascade(vector<CvRect> & handle_rect)
    {
        IplImage *gray = cvCreateImage(cvSize(left->width, left->height), 8, 1);
        cvCvtColor(left, gray, CV_BGR2GRAY);
        cvEqualizeHist(gray, gray);
        cvClearMemStorage(storage);
        if(cascade){
            CvSeq *handles = cvHaarDetectObjects(gray, cascade, storage, 1.1, 2, 0, //|CV_HAAR_FIND_BIGGEST_OBJECT
            //|CV_HAAR_DO_ROUGH_SEARCH
            //|CV_HAAR_DO_CANNY_PRUNING
            //|CV_HAAR_SCALE_IMAGE
            cvSize(10, 10));

            for(int i = 0;i < (handles ? handles->total : 0);i++){
                CvRect *r = (CvRect*)cvGetSeqElem(handles, i);

                if(handlePossibleHere(*r)){
                    handle_rect.push_back(*r);
                    if(display){
                        cvRectangle(left, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(0, 255, 0));
                        cvRectangle(disp, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 255, 255));
                    }
                }
                else{
                	if (display) {
                		cvRectangle(left, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 0, 0));
                	}
                }
            }

        }
        else {
        	ROS_ERROR("Cannot look for handle, no detector loaded");
        }

        cvReleaseImage(&gray);
    }

    /**
     * \brief Checks if a point should belong to a cluster
     *
     * @param center
     * @param p
     * @return
     */
    bool belongsToCluster(pair<float,float> center, pair<float,float> p)
    {
    	return fabs(center.first-p.first)<3 && fabs(center.second-p.second)<3;
    }


    /**
     * Helper function.
     *
     * @param r
     * @return
     */
    pair<float,float> rectCenter(const CvRect& r)
    {
    	return make_pair(r.x+(float)r.width/2,r.y+(float)r.height/2);
    }

    /**
     * \brief Decide the handle position from several detections across multiple frames
     *
     * This method does some simple clustering of all the bounding boxes and picks the cluster with
     * the most elements.
     *
     * @param handle_rect The handle bounding boxes
     * @param handle Point indicating the real-world handle position
     * @return
     */
    bool decideHandlePosition(vector<CvRect>& handle_rect, robot_msgs::PointStamped & handle)
    {
    	if (handle_rect.size()==0) {
    		return false;
    	}

    	vector<int> cluster_size;
    	vector<pair<float,float> > centers;
    	vector<pair<float,float> > sizes;

    	for (size_t i=0;i<handle_rect.size();++i) {
    		CvRect r = handle_rect[i];
    		pair<float,float> crt = rectCenter(r);
    		bool found_cluster = false;
    		for (size_t j=0;j<centers.size();++j) {
    			if (belongsToCluster(centers[j],crt)) {
    				int n = cluster_size[j];
    				centers[j].first = (centers[j].first*n+crt.first)/(n+1);
    				centers[j].second = (centers[j].second*n+crt.second)/(n+1);
    				sizes[j].first = (sizes[j].first*n+r.width)/(n+1);
    				sizes[j].second = (sizes[j].second*n+r.height)/(n+1);
    				cluster_size[j]++;
    				found_cluster = true;
    				break;
    			}
    		}
    		if (!found_cluster) {
    			centers.push_back(crt);
    			sizes.push_back(make_pair(r.width,r.height));
    			cluster_size.push_back(1);
    		}
    	}

    	int max_ind = 0;
    	int max = cluster_size[0];

    	for (size_t i=0;i<cluster_size.size();++i) {
    		if (cluster_size[i]>max) {
    			max = cluster_size[i];
    			max_ind = i;
    		}
    	}

    	CvRect bbox;
    	bbox.x = (int) centers[max_ind].first-(int)(sizes[max_ind].first/2);
    	bbox.y = (int) centers[max_ind].second-(int)(sizes[max_ind].second/2);
    	bbox.width = (int) sizes[max_ind].first;
    	bbox.height = (int) sizes[max_ind].second;

        double dx, dy;
        robot_msgs::Point p;
        getROIDimensions(bbox, dx, dy, p);

        robot_msgs::PointStamped handle_stereo;

        handle_stereo.header.frame_id = cloud.header.frame_id;
        handle_stereo.header.stamp = cloud.header.stamp;
        handle_stereo.point.x = p.z;
        handle_stereo.point.y = -p.x;
        handle_stereo.point.z = -p.y;

        try {
        	tf_->transformPoint(handle.header.frame_id, handle_stereo, handle);
        }
        catch(tf::TransformException & ex){
        	ROS_ERROR("Lookup exception: %s\n", ex.what());
        }


        printf("Clustered Handle at: (%d,%d,%d,%d)\n", bbox.x,bbox.y,bbox.width, bbox.height);


        if(display){
        	cvRectangle(left, cvPoint(bbox.x, bbox.y), cvPoint(bbox.x + bbox.width, bbox.y + bbox.height), CV_RGB(0, 255, 0));
        	cvShowImage("left", left);
        }
        showHandleMarker(handle_stereo);

		return true;
    }


    /**
     * \brief Runs the handle detector
     *
     * @param handle Position of detected handle
     * @return True if handle was found, false otherwise.
     */
    bool runHandleDetector(robot_msgs::PointStamped & handle)
    {
    	vector<CvRect> handle_rect;

        // acquire cv_mutex lock
        boost::unique_lock<boost::mutex> images_lock(cv_mutex);

        for (int i=0;i<frames_no;++i) {

//        	printf("Waiting for images\n");
        	// block until images are available to process
        	got_images_ = false;
        	preempted_ = false;
        	start_image_wait_ = ros::Time::now();
        	while (!got_images_ && !preempted_) {
        		images_ready.wait(images_lock);
        	}
        	if (preempted_) break;

//        	printf("Woke up, processing images\n");

        	if(display){
        		// show original disparity
        		cvShowImage("disparity_original", disp);
        	}
        	// eliminate from disparity locations that cannot contain a handle
        	applyPositionPrior();
        	// run cascade classifier
        	findHandleCascade(handle_rect);
        	if(display){
        		// show filtered disparity
        		cvShowImage("disparity", disp);
        		// show left image
        		cvShowImage("left", left);
        	}
        }

        bool found = decideHandlePosition(handle_rect, handle);
        return found;
    }

    /**
     * \brief Service call to detect doors
     */
    bool detectHandleSrv(door_handle_detector::DoorsDetector::Request & req, door_handle_detector::DoorsDetector::Response & resp)
    {

        robot_msgs::PointStamped handle;
        handle.header.frame_id = req.door.header.frame_id;   // want handle in the same frame as the door
    	subscribeStereoData();
        bool found = runHandleDetector(handle);
    	unsubscribeStereoData();

        if(!found){
            return false;
        }
        robot_msgs::PointStamped handle_transformed;
        // transform the point in the expected frame
        try {
            tf_->transformPoint(req.door.header.frame_id, handle, handle_transformed);
        }
        catch(tf::LookupException & ex){
            ROS_ERROR("Lookup exception: %s\n", ex.what());
        }
        catch(tf::ExtrapolationException & ex){
            ROS_DEBUG("Extrapolation exception: %s\n", ex.what());
        }
        catch(tf::ConnectivityException & ex){
            ROS_ERROR("Connectivity exception: %s\n", ex.what());
        }
        if(found){
            resp.doors.resize(1);
            resp.doors[0] = req.door;
            resp.doors[0].header.stamp = handle.header.stamp; // set time stamp
            resp.doors[0].handle.x = handle.point.x;
            resp.doors[0].handle.y = handle.point.y;
            resp.doors[0].handle.z = handle.point.z;
            resp.doors[0].weight = 1;
        }else{
            resp.doors[0] = req.door;
            resp.doors[0].weight = -1;
        }
        return true;
    }


    bool preempt(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
    {
    	ROS_INFO("Preempting.");
        boost::lock_guard<boost::mutex> lock(cv_mutex);
    	preempted_ = true;
    	images_ready.notify_all();

    	return true;
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
     * \brief Computes a least-squares estimate of a plane.
     *
     * @param points The point cloud
     * @return Plane in Hessian normal form
     */
    CvScalar estimatePlaneLS(robot_msgs::PointCloud points)
    {
        CvScalar plane;
        int cnt = points.pts.size();
        if (cnt==0) {
            plane.val[0] = plane.val[1] = plane.val[2] = plane.val[3] = -1;
            return plane;
        }
        CvMat *A = cvCreateMat(cnt, 3, CV_32FC1);
        for(int i = 0;i < cnt;++i){
            robot_msgs::Point32 p = points.pts[i];
            cvmSet(A, i, 0, p.x);
            cvmSet(A, i, 1, p.y);
            cvmSet(A, i, 2, p.z);
        }
        vector<float> ones(cnt, 1);
        CvMat B;
        cvInitMatHeader(&B, cnt, 1, CV_32FC1, &ones[0]);
        CvMat *X = cvCreateMat(3, 1, CV_32FC1);
        int ok = cvSolve(A, &B, X, CV_SVD);
        if(ok){
            float *xp = X->data.fl;
            float d = sqrt(xp[0] * xp[0] + xp[1] * xp[1] + xp[2] * xp[2]);
            plane.val[0] = xp[0] / d;
            plane.val[1] = xp[1] / d;
            plane.val[2] = xp[2] / d;
            plane.val[3] = -1 / d;
        }else{
            plane.val[0] = plane.val[1] = plane.val[2] = plane.val[3] = -1;
        }
        cvReleaseMat(&A);
        return plane;
    }

    /**
     * \brief Filters cloud point, retains only regions that could contain a handle
     */
    void applyPositionPrior()
    {
        robot_msgs::PointCloud base_cloud;
        tf_->setExtrapolationLimit(ros::Duration(2.0));
        try {
            tf_->transformPointCloud("base_footprint", cloud, base_cloud);
        }
        catch(tf::ExtrapolationException & ex){
            tf_->clear();
            ROS_WARN("TF exception: %s", ex.what());
        }
        catch(tf::LookupException & ex){
            ROS_ERROR("Lookup exception: %s\n", ex.what());
        }
        catch(tf::ConnectivityException & ex){
            ROS_ERROR("Connectivity exception: %s\n", ex.what());
        }
        int xchan = -1;
        int ychan = -1;
        for(size_t i = 0;i < base_cloud.chan.size();++i){
            if(base_cloud.chan[i].name == "x"){
                xchan = i;
            }
            if(base_cloud.chan[i].name == "y"){
                ychan = i;
            }
        }

        if(xchan != -1 && ychan != -1){
            unsigned char *pd = (unsigned char*)(disp->imageData);
            int ws = disp->widthStep;
            for(size_t i = 0;i < base_cloud.get_pts_size();++i){
                robot_msgs::Point32 crt_point = base_cloud.pts[i];
                int x = (int)(base_cloud.chan[xchan].vals[i]);
                int y = (int)(base_cloud.chan[ychan].vals[i]);

				// pointer to the current pixel
				unsigned char* crt_pd = pd+y*ws+x;
				if (crt_point.z>max_height || crt_point.z<min_height) {
					*crt_pd = 0;
				}
			}
		}
		else {
			ROS_WARN("I can't find image coordinates in the point cloud, no filtering done.");
		}
	}


    /**
     * Callback from topic synchronizer, timeout
     * @param t
     */
    void image_cb_timeout(ros::Time t)
    {
        boost::lock_guard<boost::mutex> lock(cv_mutex);
        if(limage.header.stamp != t) {
//            printf("Timed out waiting for left image\n");
        }

        if(dimage.header.stamp != t) {
//            printf("Timed out waiting for disparity image\n");
        }

        if(stinfo.header.stamp != t) {
//            printf("Timed out waiting for stereo info\n");
        }

        if(cloud_fetch.header.stamp != t) {
//        	printf("Timed out waiting for point cloud\n");
        }
        if ((ros::Time::now()-start_image_wait_) > ros::Duration(image_timeout_)) {
        	ROS_INFO("No images for %f seconds, timing out...", image_timeout_);
        	preempted_ = true;
        	images_ready.notify_all();
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

            disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
            cvCvtScale(dbridge.toIpl(), disp, 4.0 / dispinfo.dpp);
        }

        cloud = cloud_fetch;

        got_images_ = true;
        images_ready.notify_all();
    }



public:
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

