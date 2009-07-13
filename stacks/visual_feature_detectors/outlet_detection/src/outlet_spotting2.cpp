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
#include <map>
#include <list>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <limits>


#include "outlet_detection/outlet_util.h"

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
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "outlet_detection/OutletDetection.h"



#include <point_cloud_mapping/geometry/point.h>
#include "tf/transform_listener.h"
#include "topic_synchronizer/topic_synchronizer.h"

using namespace std;
using namespace robot_msgs;


#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)


static const double scan_speed  = 0.1; // [m/sec]
static const double scan_height = 0.4; //[m]

class OutletSpotting : public ros::Node
{
public:

	sensor_msgs::Image limage;
//	sensor_msgs::Image rimage;
	sensor_msgs::Image dimage;
	sensor_msgs::StereoInfo stinfo;
	sensor_msgs::DisparityInfo dispinfo;
	sensor_msgs::CamInfo lcinfo;
	sensor_msgs::CamInfo rcinfo;

	sensor_msgs::CvBridge lbridge;
//	sensor_msgs::CvBridge rbridge;
	sensor_msgs::CvBridge dbridge;

	robot_msgs::PointCloud cloud;
	robot_msgs::PointCloud cloud_fetch;

	robot_msgs::PointCloud base_cloud_;
	robot_msgs::PointCloud base_cloud_fetch_;

//	robot_msgs::PoseStamped outlet_pose;

	IplImage* left;
//	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

	bool display;
	bool save_patches;

	TopicSynchronizer<OutletSpotting> sync;


//	boost::mutex clound_point_mutex;
//	boost::condition_variable cloud_point_cv;

	boost::mutex data_lock_;
	boost::condition_variable data_cv_;
	bool have_cloud_point_;
	bool have_images_;
	bool preempt_;

	double timeout_;
	ros::Time start_image_wait_;

	tf::TransformListener* tf_;

	int frames_number_;
	string target_frame_;
	string base_scan_topic_;
	string head_controller_;

	double min_outlet_height_;
	double max_outlet_height_;

	string patch_path;

	double ppmm;		// pixels per millimeter


	typedef list<IplImage*> image_list_t;
	typedef map<string,image_list_t> template_dict_t;

	template_dict_t templates;

	vector<IplImage*> positive_templates;
	vector<IplImage*> negative_templates;

	OutletSpotting() : ros::Node("outlet_spotting"),left(NULL), disp(NULL), disp_clone(NULL),
		sync(this, &OutletSpotting::image_cb_all, ros::Duration().fromSec(0.1), &OutletSpotting::image_cb_timeout), have_cloud_point_(true), have_images_(true), ppmm(0.5)

	{
        tf_ = new tf::TransformListener(*this);

        // TODO: remove this after finished testing
        tf_->setExtrapolationLimit(ros::Duration(10.0));

		param ("~display", display, false);
		param ("~save_patches", save_patches, false);
		param ("~frames_number", frames_number_, 5);
		param<string> ("~target_frame", target_frame_, "odom_combined");
		param<string> ("~base_scan_topic", base_scan_topic_, "base_scan_marking");
		param<string>("~head_controller", head_controller_, "head_controller");
		param ("~min_outlet_height", min_outlet_height_, 0.1);
		param ("~max_outlet_height", max_outlet_height_, 0.7);
		param<string>("~patch_path", patch_path, "data");
		param("~timeout", timeout_, 3.0);
//		string template_path;
//        param<string>("~template_path", template_path,"templates");

		if (display) {
			ROS_INFO("Displaying images\n");
			cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
			//cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("contours", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
		}

        advertise<visualization_msgs::Marker>("visualization_marker", 1);
    	advertise<robot_msgs::PointStamped>(head_controller_ + "/head_track_point",10);
        advertiseService("~coarse_outlet_detect", &OutletSpotting::outletSpottingService, this);

//        loadTemplates(template_path);

	}

	~OutletSpotting()
	{
		if (left)
			cvReleaseImage(&left);
//		if (right)
//			cvReleaseImage(&right);
		if (disp)
			cvReleaseImage(&disp);

		unadvertise("visualization_marker");
		unadvertiseService("~coarse_outlet_detect");
	}

private:

    void trimSpaces( string& str)
    {
    	size_t startpos = str.find_first_not_of(" \t\n"); // Find the first character position after excluding leading blank spaces
    	size_t endpos = str.find_last_not_of(" \t\n"); // Find the first character position from reverse af

    	// if all spaces or empty return an empty string
    	if(( string::npos == startpos ) || ( string::npos == endpos)) {
    		str = "";
    	}
    	else {
    		str = str.substr( startpos, endpos-startpos+1 );
    	}
    }

    void loadTemplates(string path)
    {

    	string template_file = path + "/templates.txt";
    	FILE* f = fopen( template_file.c_str(), "r" );

    	if (!f) {
    		ROS_ERROR("Cannot open templates file: %s", template_file.c_str());
    	}

    	char buffer[1024];

    	while (fgets(buffer,1024,f)) {
    		string line = buffer;
    		trimSpaces(line);
    		if (line.empty()) continue;
    		int pos = line.find_first_of(" ");
    		string filename = line.substr(0,pos);
    		string category = line.substr(pos+1);
    		ROS_INFO("Loading template %s for category %s", filename.c_str(), category.c_str());
    		IplImage* outlet_template = cvLoadImage((path + "/" + filename).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    		if (!outlet_template) {
    			ROS_ERROR("Cannot load template from file: %s", (path + "/" + filename).c_str());
    		}
    		templates[category].push_back(outlet_template);
    	}

    	fclose(f);
	}


    void subscribeToData()
    {

    	sync.reset();
		std::list<std::string> left_list;
		left_list.push_back(std::string("stereo/left/image_rect_color"));
		left_list.push_back(std::string("stereo/left/image_rect"));
		sync.subscribe(left_list,  limage, 1);

//		std::list<std::string> right_list;
//		right_list.push_back(std::string("stereo/right/image_rect_color"));
//		right_list.push_back(std::string("stereo/right/image_rect"));
//		sync.subscribe(right_list, rimage, 1);

		sync.subscribe("stereo/disparity", dimage, 1);
		sync.subscribe("stereo/stereo_info", stinfo, 1);
		sync.subscribe("stereo/disparity_info", dispinfo, 1);
		sync.subscribe("stereo/left/cam_info", lcinfo, 1);
		sync.subscribe("stereo/right/cam_info", rcinfo, 1);

		sync.subscribe("stereo/cloud", cloud_fetch, 1);

		sync.ready();

		subscribe(base_scan_topic_, base_cloud_fetch_, &OutletSpotting::laser_callback, 1);
    }

    void unsubscribeFromData()
    {
        unsubscribe("stereo/left/image_rect_color");
        unsubscribe("stereo/left/image_rect");
        unsubscribe("stereo/disparity");
        unsubscribe("stereo/stereo_info");
        unsubscribe("stereo/disparity_info");
        unsubscribe("stereo/left/cam_info");
        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");
    }


#define CVCONTOUR_APPROX_LEVEL 2   // Approx.threshold - the bigger it is, the simpler is the boundary
#define CV_CVX_WHITE  CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK CV_RGB(0x00,0x00,0x00)
    /**
     * A function that finds connected components (blobs) in an image.
     *
     * @param mask				Is a grayscale (8 bit depth) image where the contours are found. The image will be altered in the process.
     * @param poly1_hull0		If set, approximate connected component by (DEFAULT) polygon, or else convex hull (0)
     * @param areaTooSmall		Kill contours whose bounding box area is less than this
     * @param areaTooLarge		Kill contours whose bounding box area is more than this
     * @param aspectLimit		Kill contours with aspect ratio smaller than aspectLimit.
     * @param num				Maximum number of rectangles and/or centers to return, on return, will contain number filled (DEFAULT: NULL)
     * @param bbs				Pointer to bounding box rectangle vector of length num.  (DEFAULT SETTING: NULL)
     * @param centers			Pointer to contour centers vectore of length num (DEFULT: NULL)
     */
	void findConnectedComponents(IplImage *mask, int poly1_hull0, float areaTooSmall, float areaTooLarge, float aspectLimit, int *num = NULL, CvRect *bbs = NULL, CvPoint *centers = NULL)
	{
		static CvMemStorage*	mem_storage	= NULL;
		static CvSeq*			contours	= NULL;

		// FIND CONTOURS AROUND ONLY BIGGER REGIONS
		if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
		else cvClearMemStorage(mem_storage);

		CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		CvSeq* c;
		int numCont = 0;
		CvRect bb;
		// filter loop
		while( (c = cvFindNextContour( scanner )) != NULL ) {
			bb = cvBoundingRect(c);
			float wd = (float)(bb.width);
			float ht = (float)(bb.height);
			float carea = (float)(wd*ht);
			float aspect;
			if(wd > ht) {
				aspect = ht/wd;
			} else {
				aspect = wd/ht;
			}
			//Get rid of blob if it's too small or too large
			if( carea <  areaTooSmall || carea > areaTooLarge || (aspect < aspectLimit)) {
				cvSubstituteContour( scanner, NULL );
			}
			else {
				// Smooth it's edges if it's large enough
				CvSeq* c_new;
				if(poly1_hull0) //Polygonal approximation of the segmentation
					c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
				else //Convex Hull of the segmentation
					c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
//				cvSubstituteContour( scanner, c_new );
				numCont++;
			}
		}
		contours = cvEndFindContours( &scanner );

		// paint the found contours back into the image
		cvZero( mask );
		IplImage *maskTemp = cvCloneImage(mask);
		// find centers of mass and bounding boxes
		int numFilled = 0;
		int i;
		for(i=0, c=contours; c != NULL; c = c->h_next,i++ ) {
			// Only process up to *num of them
			if(num != NULL && i < *num) {
				cvZero(maskTemp);
				cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
				//Find the center of each contour
				if(centers != NULL) {
					CvMoments moments;
					cvMoments(maskTemp,&moments,1);
					double M00 = cvGetSpatialMoment(&moments,0,0);
					double M10 = cvGetSpatialMoment(&moments,1,0);
					double M01 = cvGetSpatialMoment(&moments,0,1);
					centers[i].x = (int)(M10/M00);
					centers[i].y = (int)(M01/M00);
				}
				// Bounding rectangles around blobs
				if(bbs != NULL) {
					bbs[i] = cvBoundingRect(c);
				}
				numFilled++;
			}
			//Draw filled contours into mask
			cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
#if 0
			cvNamedWindow("temp",1);
			cvShowImage("temp", mask);
			cvWaitKey(0);
#endif
		}
		cvReleaseImage( &maskTemp);

		if (num != NULL) {
			*num = numFilled;
		}
	}


	void savePatch(IplImage* img, CvRect& R, const char* prefix)
	{
		static int patch_cnt = 1;

		char name[100];
		sprintf(name,"%s/%s_%.3d.png", patch_path.c_str() , prefix, patch_cnt++);
		printf("Saving patch %s\n",name);
		cvSetImageROI(img,R);
		cvSaveImage(name, img);
		cvResetImageROI(img);
	}


    /**
     * \brief Publishes a visualization marker for a point.
     * @param p
     */
    void showMarkers(robot_msgs::PoseStamped pose)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = pose.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)(0ULL));
        marker.ns = "outlet_spotting";
        marker.id = 101;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = pose.pose;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.g = 1.0;

        publish("visualization_marker", marker);


        tf::Pose tf_pose;

        tf::poseMsgToTF(pose.pose,tf_pose);
        tf::Point point(-1,0,0);
        tf::Point normal = tf_pose*point;


        marker.header.frame_id = pose.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)(0ULL));
        marker.id = 102;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = normal.x();
        marker.pose.position.y = normal.y();
        marker.pose.position.z = normal.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        publish("visualization_marker", marker);
    }


    void showPointMarker(robot_msgs::PointStamped point, int id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = point.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)(0ULL));
        marker.ns = "outlet_spotting_points";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.g = 1.0;

        publish("visualization_marker", marker);
    }



//    bool getLaserScan(PointStamped outlet_location, PointCloud& laser_scan)
//    {
//    	ROS_INFO("OutletSpotter: Getting laser scan...");
//    	tf_->transformPoint("base_footprint", outlet_location, outlet_location);
//
//    	// get laser height
//    	tf::Stamped<tf::Transform> tilt_stage;
//    	tf_->lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
//    	double laser_height = tilt_stage.getOrigin()[2];
//    	double outlet_height = outlet_location.point.z;
//    	double dist = outlet_location.point.x;
//    	double outlet_bottom = outlet_height-(scan_height/2.0);
//    	double outlet_top = outlet_height+(scan_height/2.0);
//
//    	point_cloud_assembler::BuildCloudAngle::Request req_pointcloud;
//    	point_cloud_assembler::BuildCloudAngle::Response res_pointcloud;
//    	req_pointcloud.angle_begin = -atan2(outlet_top - laser_height, dist);
//    	req_pointcloud.angle_end = atan2(laser_height - outlet_bottom, dist);
//    	req_pointcloud.duration = scan_height/scan_speed;
//    	if (!ros::service::call("point_cloud_srv/single_sweep_cloud", req_pointcloud, res_pointcloud)){
//    		return false;
//    	}
//
//    	laser_scan = res_pointcloud.cloud;
//    	return true;
//    }


    /**
     * Shows a line segment in the visualizer
     *
     * @param line_segment
     */
    void showLineMarker(const vector<Point32>& line_segment)
    {
    	visualization_msgs::Marker marker;
    	marker.header.frame_id = base_cloud_.header.frame_id;
    	marker.header.stamp = ros::Time((uint64_t)0ULL);
    	marker.ns = "outlet_spotting";
    	marker.id = 102;
    	marker.type = visualization_msgs::Marker::LINE_STRIP;
    	marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.orientation.w = 1.0;
    	marker.scale.x = 0.01;
    	marker.color.a = 1.0;
    	marker.color.g = 1.0;
    	marker.set_points_size(2);

    	marker.points[0].x = line_segment[0].x;
    	marker.points[0].y = line_segment[0].y;
    	marker.points[0].z = line_segment[0].z;

    	marker.points[1].x = line_segment[1].x;
    	marker.points[1].y = line_segment[1].y;
    	marker.points[1].z = line_segment[1].z;

    	publish( "visualization_marker", marker );

    }

	void morphologicSmoothing(IplImage* img)
	{
		int Nerode = 1;
		int Ndialate = 7;

		cvErode(img, img, NULL, Nerode);
		cvDilate(img, img, NULL,Ndialate);
		cvErode(img, img, NULL, Ndialate);
	}

	struct Stats {

		Stats() : mean(0), stdev(0), min(0), max(0) {};

		float mean;
		float stdev;
		float min;
		float max;
	};


	void pointCloudStatistics(const PointCloud& pc, Stats& x_stats, Stats& y_stats, Stats& z_stats)
	{
		uint32_t size = pc.get_pts_size();
		if (size==0) {
			return;
		}
		x_stats.mean = 0;
		x_stats.stdev = 0;
		y_stats.mean = 0;
		y_stats.stdev = 0;
		z_stats.mean = 0;
		z_stats.stdev = 0;

		x_stats.min = pc.pts[0].x;
		x_stats.max = pc.pts[0].x;
		y_stats.min = pc.pts[0].y;
		y_stats.max = pc.pts[0].y;
		z_stats.min = pc.pts[0].z;
		z_stats.max = pc.pts[0].z;

		for (uint32_t i=0;i<size;++i) {
			x_stats.mean += pc.pts[i].x;
			y_stats.mean += pc.pts[i].y;
			z_stats.mean += pc.pts[i].z;
			x_stats.stdev += (pc.pts[i].x*pc.pts[i].x);
			y_stats.stdev += (pc.pts[i].y*pc.pts[i].y);
			z_stats.stdev += (pc.pts[i].z*pc.pts[i].z);

			if (x_stats.min>pc.pts[i].x) x_stats.min = pc.pts[i].x;
			if (x_stats.max<pc.pts[i].x) x_stats.max = pc.pts[i].x;
			if (y_stats.min>pc.pts[i].y) y_stats.min = pc.pts[i].y;
			if (y_stats.max<pc.pts[i].y) y_stats.max = pc.pts[i].y;
			if (z_stats.min>pc.pts[i].z) z_stats.min = pc.pts[i].z;
			if (z_stats.max<pc.pts[i].z) z_stats.max = pc.pts[i].z;
		}

		x_stats.mean /= size;
		y_stats.mean /= size;
		z_stats.mean /= size;

		x_stats.stdev = x_stats.stdev/size - x_stats.mean*x_stats.mean;
		y_stats.stdev = y_stats.stdev/size - y_stats.mean*y_stats.mean;
		z_stats.stdev = z_stats.stdev/size - z_stats.mean*z_stats.mean;
	}


	/**
	 * Projects 3D point into image plane
	 * @param cam_info Camera info containing camera projection matrix
	 * @param point The 3D point
	 * @return Projected point
	 */
	Point project3DPointIntoImage(const sensor_msgs::CamInfo& cam_info, PointStamped point)
	{
		PointStamped image_point;
		tf_->transformPoint(cam_info.header.frame_id, point, image_point);
		Point pp; // projected point

		pp.x = cam_info.P[0]*image_point.point.x+
				cam_info.P[1]*image_point.point.y+
				cam_info.P[2]*image_point.point.z+
				cam_info.P[3];
		pp.y = cam_info.P[4]*image_point.point.x+
				cam_info.P[5]*image_point.point.y+
				cam_info.P[6]*image_point.point.z+
				cam_info.P[7];
		pp.z = cam_info.P[8]*image_point.point.x+
				cam_info.P[9]*image_point.point.y+
				cam_info.P[10]*image_point.point.z+
				cam_info.P[11];

		pp.x /= pp.z;
		pp.y /= pp.z;
		pp.z = 1;

		return pp;
	}


	bool getRectifiedPatch(const PoseStamped& pose, IplImage* patch, double extent, CvMat* homography)
	{
		PointStamped base_p1, base_p2, base_p3, base_p4;

		tf::Pose tf_pose;
		tf::Point tf_point;
		tf::poseMsgToTF(pose.pose,tf_pose);

		tf_point = tf::Point(0,extent,extent);
		tf_point = tf_pose*tf_point;
		base_p1.header.stamp = base_cloud_.header.stamp;
		base_p1.header.frame_id = base_cloud_.header.frame_id;
		tf::pointTFToMsg(tf_point,base_p1.point);

		tf_point = tf::Point(0,-extent,extent);
		tf_point = tf_pose*tf_point;
		base_p2.header.stamp = base_cloud_.header.stamp;
		base_p2.header.frame_id = base_cloud_.header.frame_id;
		tf::pointTFToMsg(tf_point,base_p2.point);

		tf_point = tf::Point(0,extent,-extent);
		tf_point = tf_pose*tf_point;
		base_p3.header.stamp = base_cloud_.header.stamp;
		base_p3.header.frame_id = base_cloud_.header.frame_id;
		tf::pointTFToMsg(tf_point,base_p3.point);

		tf_point = tf::Point(0,-extent,-extent);
		tf_point = tf_pose*tf_point;
		base_p4.header.stamp = base_cloud_.header.stamp;
		base_p4.header.frame_id = base_cloud_.header.frame_id;
		tf::pointTFToMsg(tf_point,base_p4.point);

		Point p1 = project3DPointIntoImage(lcinfo, base_p1);
		Point p2 = project3DPointIntoImage(lcinfo, base_p2);
		Point p3 = project3DPointIntoImage(lcinfo, base_p3);
		Point p4 = project3DPointIntoImage(lcinfo, base_p4);

		int width = patch->width;
		int height = patch->height;

		CvPoint2D32f objPts[4], imgPts[4];
		objPts[0].x = 0; objPts[0].y = 0;
		objPts[1].x = width; objPts[1].y = 0;
		objPts[2].x = 0; objPts[2].y = height;
		objPts[3].x = width; objPts[3].y = height;

		imgPts[0].x = p1.x; imgPts[0].y = p1.y;
		imgPts[1].x = p2.x; imgPts[1].y = p2.y;
		imgPts[2].x = p3.x; imgPts[2].y = p3.y;
		imgPts[3].x = p4.x; imgPts[3].y = p4.y;

		// compute perspective transformation
//		CvMat *H = cvCreateMat(3,3,CV_32F);
		cvGetPerspectiveTransform(objPts,imgPts,homography);
		cvWarpPerspective(left,patch, homography, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS );

//		cvReleaseMat(&H);

		return true;
	}





    /**
     * \brief Filters cloud point, retains only regions that could contain a handle
     */
    void filterByHeight(PointCloud stereo_cloud, IplImage* disparity, double min_height, double max_height)
    {
        robot_msgs::PointCloud base_cloud;
        tf_->transformPointCloud("base_footprint", stereo_cloud, base_cloud);

        // find original image coordinates channels
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
            for(size_t i = 0;i < base_cloud.get_pts_size();++i){
                robot_msgs::Point32 crt_point = base_cloud.pts[i];
                int x = (int)(base_cloud.chan[xchan].vals[i]);
                int y = (int)(base_cloud.chan[ychan].vals[i]);

				// pointer to the current pixel
				unsigned char* crt_pd = CV_PIXEL(unsigned char, disparity, x,y);
				if (crt_point.z>max_height || crt_point.z<min_height) {
					*crt_pd = 0;
				}
			}
		}
		else {
			ROS_WARN("I can't find image coordinates in the point cloud, no filtering done.");
		}
	}




//    void fitOutletPlane(PointCloud &points, double z_min, double z_max, double support, double min_area, int n_max,
//    		vector<vector<int> > &indices, vector<vector<double> > &models)
//    {
//    	// This should be given as a parameter as well, or set global, etc
//    	double sac_distance_threshold_ = 0.007;        // 2cm distance threshold for inliers (point-to-plane distance)
//
//    	vector<int> indices_in_bounds;
//    	// Get the point indices within z_min <-> z_max
//    	getPointIndicesInZBounds (points, z_min, z_max, indices_in_bounds);
//
//    	// We need to know the viewpoint where the data was acquired
//    	// For simplicity, assuming 0,0,0 for stereo data in the stereo frame - however if this is not true, use TF to get
//    	//the point in a different frame !
//    	Point32 viewpoint;
//    	viewpoint.x = viewpoint.y = viewpoint.z = 0;
//
//    	// Use the entire data to estimate the plane equation.
//    	// NOTE: if this is slow, we can downsample first, then fit (check mapping/point_cloud_mapping/src/planar_fit.cpp)
//    	//   vector<vector<int> > inliers;
//    	indices.clear(); //Points that are in plane
//    	models.clear();  //Plane equations
//    	//    vector<vector<double> > models;
//    	fitSACPlanes (&points, indices_in_bounds, indices, models, viewpoint, sac_distance_threshold_, n_max);
//
//    	// Check the list of planar areas found against the minimally imposed area
//    	for (unsigned int i = 0; i < models.size (); i++)
//    	{
//    		// Compute the convex hull of the area
//    		// NOTE: this is faster than computing the concave (alpha) hull, so let's see how this works out
//    		Polygon3D polygon;
//    		cloud_geometry::areas::convexHull2D (points, indices[i], models[i], polygon);
//
//    		// Compute the area of the polygon
//    		double area = cloud_geometry::areas::compute2DPolygonalArea (polygon, models[i]);
//
//    		// If the area is smaller, reset this planar model
//    		if (area < min_area)
//    		{
//    			models[i].resize (0);
//    			indices[i].resize (0);
//    			continue;
//    		}
//    	}
//
//    	//    // Copy all the planar models inliers to indices
//    	//    for (unsigned int i = 0; i < inliers.size (); i++)
//    	//    {
//    	//      if (inliers[i].size () == 0) continue;
//    	//
//    	//      int old_indices_size = indices.size ();
//    	//      indices.resize (old_indices_size + inliers[i].size ());
//    	//      for (unsigned int j = 0; j < inliers[i].size (); j++)
//    	//        indices[old_indices_size + j] = inliers[i][j];
//    	//    }
//    }



    bool fitOutletPlane(const PointCloud& outlet_cloud, PointCloud& outlet_plane_cloud, double distance, double eps_angle, int min_points, vector<double>& coeff)
    {

    	ROS_INFO("Fit outlet plane using %d points.", outlet_cloud.get_pts_size());
    	// initialize indices vector
    	vector<int> indices(outlet_cloud.get_pts_size());
    	for (size_t i=0;i<indices.size();++i) {
    		indices[i] = i;
    	}
    	// orientation axis (in wall frame)
    	Point32 axis;
    	axis.x = 1;
    	axis.y = 0;
    	axis.z = 0;

    	vector<int> inliers;

    	// fit plane
    	bool fit_ok = fitSACOrientedPlane(outlet_cloud, indices,inliers, coeff, axis, distance, eps_angle, min_points);
    	if (!fit_ok) {
    		return false;
    	}
    	// select inliers
    	cloud_geometry::getPointCloud (outlet_cloud, inliers, outlet_plane_cloud);
    	return true;
    }


    double matchTemplateScore(IplImage* patch, IplImage* templ, CvPoint* position = NULL)
    {
    	CvSize patch_size = cvGetSize(patch);
    	CvSize templ_size = cvGetSize(templ);
    	CvSize result_size = cvSize(patch->width-templ->width+1,patch->height-templ->height+1 );
    	IplImage* result = cvCreateImage(result_size, IPL_DEPTH_32F, 1);

    	cvMatchTemplate(patch, templ, result, CV_TM_SQDIFF_NORMED);
		double min_val;
		double max_val;
		cvMinMaxLoc(result, &min_val, &max_val, position, NULL);

		cvReleaseImage(&result);

		return min_val;
    }


    template<typename T, typename U>
    double pointSquaredDistance2D(T a, U b)
    {
    	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
    }

    int find_dir(const CvPoint* dir, int xsign, int ysign)
    {
    	for(int i = 0; i < 4; i++)
    	{
    		if(dir[i].x*xsign > 0 && dir[i].y*ysign > 0)
    		{
    			return i;
    		}
    	}
    	return -1;
    }

    void order_tuple(vector<CvPoint>& centers)
    {
    	CvPoint ordered[4];
    	int idx[4];

    	CvPoint center = cvPoint(0.0f, 0.0f);
    	for(int i = 0; i < 4; i++)
    	{
    		center.x += centers[i].x;
    		center.y += centers[i].y;
    	}
    	center.x *= 0.25f;
    	center.y *= 0.25f;

    	CvPoint dir[4];
    	for(int i = 0; i < 4; i++)
    	{
    		dir[i].x = centers[i].x - center.x;
    		dir[i].y = centers[i].y - center.y;
    	}

    	idx[0] = find_dir(dir, -1, -1);
    	idx[1] = find_dir(dir, 1, -1);
    	idx[2] = find_dir(dir, 1, 1);
    	idx[3] = find_dir(dir, -1, 1);

    	for(int i = 0; i < 4; i++)
    	{
    		ordered[i] = centers[idx[i]];
    	}

    	for(int i = 0; i < 4; i++)
    	{
    		centers[i] = ordered[i];
    	}

    }

    bool detectOutletInImage(IplImage *outlet_image, CvRect& rect, string& category)
    {
    	IplImage* gray_image = cvCreateImage(cvGetSize(outlet_image), outlet_image->depth, 1);
    	if (outlet_image->nChannels>1) {
    		cvCvtColor(outlet_image,gray_image,CV_RGB2GRAY);
    	}
    	else {
    		cvCopy(outlet_image,gray_image);
    	}
    	IplImage* binary_image = cvCreateImage(cvGetSize(outlet_image), outlet_image->depth, 1);

    	cvCanny(gray_image, binary_image, 30,60);
    	if (display) {
    		cvNamedWindow("templ",1);
    		//cvShowImage("templ", outlet_image);
    		cvShowImage("templ", binary_image);
    	}


    	CvMemStorage* storage = cvCreateMemStorage();
    	CvSeq* first = 0;
    	cvFindContours(binary_image, storage, &first, sizeof(CvContour), CV_RETR_LIST);
    	vector<CvPoint > candidates;
    	for(CvSeq* seq = first; seq != NULL; seq = seq->h_next)
    	{

    		CvRect rect = cvBoundingRect(seq);

    		if (abs(20-rect.width)>4 || abs(15-rect.height)>4)
    		{
    			continue;
    		}

    		CvPoint center = cvPoint(rect.x+rect.width/2, rect.y+rect.height/2);

			bool add_candidate = true;
			for (size_t i = 0; i<candidates.size();++i) {
				if (pointSquaredDistance2D(candidates[i],center)<25) {
					add_candidate = false;
				}
			}
			if (add_candidate) {
				candidates.push_back(center);
			}
    	}


    	cvReleaseMemStorage(&storage);
    	cvReleaseImage(&binary_image);
    	cvReleaseImage(&gray_image);



    	if (candidates.size()!=4) {
                ROS_INFO("in detectOutletInImage did not find 4 blocks\n\n\n");
    		return false;
    	}

    	order_tuple(candidates);

    	if (abs(candidates[0].y-candidates[1].y)>5 || abs(candidates[2].y-candidates[3].y)>5) {
                ROS_INFO("in detectOutletInImage vertical alignment bad \n\n\n");
    		return false;
    	}
    	if (abs(candidates[0].x-candidates[3].x)>5 || abs(candidates[1].x-candidates[2].x)>5) {
                ROS_INFO("in detectOutletInImage horizontal alignment bad \n\n\n");
    		return false;
    	}

    	if (abs(abs(candidates[0].x-candidates[1].x)-45*ppmm)>5 || abs(abs(candidates[2].x-candidates[3].x)-45*ppmm)>5) {
                ROS_INFO("in detectOutletInImage horizontal size wrong \n\n\n");
    		return false;
    	}

    	if (abs(abs(candidates[0].y-candidates[3].y)-39*ppmm)>5 || abs(abs(candidates[1].y-candidates[2].y)-39*ppmm)>5) {
                ROS_INFO("in detectOutletInImage vertical size wrong \n\n\n");
    		return false;
    	}


    	rect.x = candidates[0].x-20;
    	rect.y = candidates[0].y-20;
    	rect.width = candidates[1].x-candidates[0].x+40;
    	rect.height = candidates[2].y-candidates[0].y+40;


    	return true;

    }

#if 0

    bool detectOutletInImage2(IplImage *outlet_image, CvRect& rect, string& category)
    {
    	IplImage* gray_image = cvCreateImage(cvGetSize(outlet_image), outlet_image->depth, 1);
    	if (outlet_image->nChannels>1) {
    		cvCvtColor(outlet_image,gray_image,CV_RGB2GRAY);
    	}
    	else {
    		cvCopy(outlet_image,gray_image);
    	}


    	double best_score = -1;
    	string best_category;
    	CvPoint best_position = cvPoint(0,0);
    	IplImage* best_template = NULL;
    	for (template_dict_t::const_iterator it = templates.begin(); it!=templates.end(); ++it) {
    		image_list_t& outlet_templates = templates[it->first];
    		double score = -1;
        	CvPoint position = cvPoint(0,0);
        	IplImage* outlet_template = NULL;
    		for (image_list_t::const_iterator img_it = outlet_templates.begin(); img_it!=outlet_templates.end(); ++img_it) {
    			IplImage* crt_template = *img_it;
    			CvPoint crt_position;
    			double crt_score = matchTemplateScore(gray_image, crt_template, &crt_position);
    			if (score==-1 || score>crt_score) {
    				score = crt_score;
    				position = crt_position;
    				outlet_template = crt_template;
    			}
    		}
    		ROS_INFO("Category: %s, score: %f, at: (%d,%d)", it->first.c_str(), score, position.x,position.y);
    		if (best_score==-1 || best_score>score) {
    			best_score = score;
    			best_category = it->first;
    			best_position =  position;
    			best_template = outlet_template;
    		}
    	}
		cvReleaseImage(&gray_image);

    	category = best_category;

    	CvSize s = cvGetSize(best_template);
    	rect.x = best_position.x;
    	rect.y = best_position.y;
    	rect.width = s.width;
    	rect.height = s.height;

    	ROS_INFO("Outlet_type: %s", best_category.c_str());


    	if (best_score>0.01) {
    		return false;
    	}

    	if (best_category=="comm") {
    		return false;
    	}

    	if (display) {
    		CvPoint p = cvPoint(rect.x + rect.width, rect.y + rect.height);
    		cvRectangle(outlet_image,best_position,p, CV_RGB(0,255,0));
    		cvNamedWindow("templ", 1);
    		cvShowImage("templ", outlet_image);
    	}

    	return true;
    }
#endif

    Point nearestPoint(const PointCloud& pc, const Point& point)
    {
    	if (pc.get_pts_size()==0) {
    		return point;
    	}
    	Point32 nearest_point = pc.pts[0];
    	double dist = squaredPointDistance(point, nearest_point);
    	for (size_t i=1;i<pc.get_pts_size();++i) {
    		double crt_dist = squaredPointDistance(point, pc.pts[i]);
    		if (crt_dist<dist) {
    			dist = crt_dist;
    			nearest_point = pc.pts[i];
    		}
    	}
    	Point result;
    	result.x = nearest_point.x;
    	result.y = nearest_point.y;
    	result.z = nearest_point.z;

    	return result;
    }


	void perspectiveTransformOutletBBox(CvRect outlet_rect, CvMat* homography, CvPoint* outlet_bbox)
	{
		CvMat* src = cvCreateMat(1,4,CV_32FC2);
		float* ptr = src->data.fl;
		*ptr++ = outlet_rect.x;
		*ptr++ = outlet_rect.y;
		*ptr++ = outlet_rect.x+outlet_rect.width;
		*ptr++ = outlet_rect.y;
		*ptr++ = outlet_rect.x+outlet_rect.width;
		*ptr++ = outlet_rect.y+outlet_rect.height;
		*ptr++ = outlet_rect.x;
		*ptr++ = outlet_rect.y+outlet_rect.height;

		cvPerspectiveTransform(src,src,homography);

		ptr = src->data.fl;
		outlet_bbox[0].x = *ptr++;	outlet_bbox[0].y = *ptr++;
		outlet_bbox[1].x = *ptr++;	outlet_bbox[1].y = *ptr++;
		outlet_bbox[2].x = *ptr++;	outlet_bbox[2].y = *ptr++;
		outlet_bbox[3].x = *ptr++;	outlet_bbox[3].y = *ptr++;

		cvReleaseMat(&src);
	}


	bool detectOutlet(PoseStamped& pose)
	{
		int areaTooSmall = 20; //bounding box area
		int areaTooLarge = 150;
		int aspectLimit = 45;
		int num = 20;
		CvRect bbs[num];
		CvPoint centers[num];

		disp_clone = cvCloneImage(disp);

		// smooth out the blobs in the disparity image
		// TODO: commented out due to bug in OpenCV
//		morphologicSmoothing(disp);

		// find connected components (blobs) in the disparity image
		findConnectedComponents(disp, 1, areaTooSmall*areaTooSmall,
				areaTooLarge*areaTooLarge,(float)(aspectLimit)/100.0, &num, bbs, centers);

		Stats x_stats;
		Stats y_stats;
		Stats z_stats;

		// filtering loop
		for(int i=0; i<num; ++i) {

			// get outlet candidate point cloud
			PointCloud blob_cloud = filterPointCloud(cloud, bbs[i]);

			pointCloudStatistics(blob_cloud, x_stats, y_stats, z_stats);
			// center of outlet candidate cloud
			PointStamped center_in_stereo_frame;
			center_in_stereo_frame.header.frame_id = blob_cloud.header.frame_id;
			center_in_stereo_frame.header.stamp = blob_cloud.header.stamp;
			center_in_stereo_frame.point.x = x_stats.mean;
			center_in_stereo_frame.point.y = y_stats.mean;
			center_in_stereo_frame.point.z = z_stats.mean;

			// check the center of the blob is at a reasonable height in "base_footprint" frame
			PointStamped center_in_base_footprint;
			tf_->transformPoint("base_footprint", center_in_stereo_frame, center_in_base_footprint);
			if (center_in_base_footprint.point.z<min_outlet_height_ || center_in_base_footprint.point.z>max_outlet_height_) {
				continue;
			}

			PointStamped center_in_base_laser_frame;
			tf_->transformPoint(base_cloud_.header.frame_id, center_in_stereo_frame, center_in_base_laser_frame);
			PoseStamped wall_pose;
			ROS_INFO("Getting wall pose");
			if (!getWallPoseFromBaseLaser(base_cloud_, center_in_base_laser_frame, 0.4, wall_pose)) {
				ROS_INFO("OutletSpotter: Cannot compute wal pose, skipping outlet candidate");
				continue;
			}

			// add wall_frame to tf
			tf::Pose tf_wall_pose;
			tf::poseMsgToTF(wall_pose.pose, tf_wall_pose);
			tf::Stamped<tf::Pose> wall_pose_frame(tf_wall_pose, wall_pose.header.stamp, "wall_frame", wall_pose.header.frame_id);
			tf_->setTransform(wall_pose_frame);

//			tf::TransformBroadcaster broadcaster(*this);
//			broadcaster.sendTransform(wall_pose_frame);

			PointCloud blob_in_wall_frame;
			tf_->transformPointCloud("wall_frame", blob_cloud, blob_in_wall_frame);

			PointCloud outlet_plane;
			vector<double> plane_coefficients_wall_frame;
			// fit outlet plane, min 300 inliers, max 20 deg deviation from correct orientation
			if (!fitOutletPlane(blob_in_wall_frame, outlet_plane, 0.02, 20*M_PI/180, 300, plane_coefficients_wall_frame)) {
				ROS_INFO("Cannot fit outlet plane, skipping outlet candidate");
				continue;
			}

			pointCloudStatistics(outlet_plane, x_stats, y_stats, z_stats);
			if ((y_stats.max-y_stats.min)>0.25 && (y_stats.max-y_stats.min)>0.25 ) {
				ROS_INFO("OutletSpotter: Size too big for an outlet (%f,%f), ignoring blob", x_stats.max-x_stats.min,y_stats.max-y_stats.min);
			}

			// adjust the wall pose origin
			PointStamped origin_in_wall_frame;
			origin_in_wall_frame.header.frame_id = blob_in_wall_frame.header.frame_id;
			origin_in_wall_frame.header.stamp = blob_in_wall_frame.header.stamp;
			origin_in_wall_frame.point.x = x_stats.mean;
			origin_in_wall_frame.point.y = y_stats.mean;
			origin_in_wall_frame.point.z = z_stats.mean;

			PointStamped origin_in_base_laser_frame;
			tf_->transformPoint(base_cloud_.header.frame_id, origin_in_wall_frame, origin_in_base_laser_frame);
			// find the nearest base_laser_point
			wall_pose.pose.position = nearestPoint(base_cloud_, origin_in_base_laser_frame.point);

//			tf::poseMsgToTF(wall_pose.pose, tf_wall_pose);
//			tf::Stamped<tf::Pose> wall_pose_frame2(tf_wall_pose, wall_pose.header.stamp, "wall_frame", wall_pose.header.frame_id);
//			tf_->setTransform(wall_pose_frame2);

			double wall_region_size = 0.3; // 30 cm
			double ppm = 0.5;  // pixels per millimeter
			int patch_size = int(wall_region_size*1000*ppm);

			ROS_INFO("Computing rectified patch");
			IplImage* patch = cvCreateImage(cvSize(patch_size,patch_size),left->depth, left->nChannels);
			CvMat *homography = cvCreateMat(3,3,CV_32F);
			getRectifiedPatch(wall_pose, patch, wall_region_size/2, homography);

			CvRect outlet_rect;
			string outlet_type;
			if (!detectOutletInImage(patch,outlet_rect, outlet_type)) {
				ROS_INFO("OutletSpotter: The image patch doesn't contain an outlet, skipping");
				continue;
			}

			// transform detected outlet bounding box to the left image plane
			CvPoint outlet_bbox[4];
			perspectiveTransformOutletBBox(outlet_rect, homography, outlet_bbox);

			// compute outlet center and project onto wall plane
			CvPoint outlet_center;
			outlet_center.x = (outlet_bbox[0].x + outlet_bbox[2].x)/2;
			outlet_center.y = (outlet_bbox[0].y + outlet_bbox[2].y)/2;


			// need to compute wall plane equation in stereo frame
			// we already have it in the wall frame
//			tf::Stamped<tf::Pose> tf_wall_to_stereo;
//			tf_->lookupTransform(cloud.header.frame_id, "wall_frame", cloud.header.stamp, tf_wall_to_stereo);
//			btScalar transform[16];
//			tf_wall_to_stereo.inverse().getOpenGLMatrix(transform);
//

//			Point outlet_center_in_wall;
//			projectToWallPlane(outlet_center, )

			for (int k = 0;k<4;++k) {
				if (outlet_bbox[k].x<0 || outlet_bbox[k].x>left->width || outlet_bbox[k].y<0 || outlet_bbox[k].y>left->height) {
					continue;
				}
			}


			// update wall pose origin to the outlet center

			if (display) {
				// draw bounding box
				cvLine(left,outlet_bbox[0],outlet_bbox[1],CV_RGB(0,255,0));
				cvLine(left,outlet_bbox[1],outlet_bbox[2],CV_RGB(0,255,0));
				cvLine(left,outlet_bbox[2],outlet_bbox[3],CV_RGB(0,255,0));
				cvLine(left,outlet_bbox[3],outlet_bbox[0],CV_RGB(0,255,0));

				cvCircle(left, outlet_center, 5, CV_RGB(0,255,0));
			}

			cvReleaseMat(&homography);


			if (save_patches) savePatch(left,bbs[i],"outlet_match");
			cvReleaseImage(&patch);

			// if we got here, we found an outlet

			tf_->transformPose(target_frame_, wall_pose, pose);
			return true;
		}

		ROS_INFO("OutletSpotter: outlet not found");
		return false;
	}


	robot_msgs::PointCloud filterPointCloud(const PointCloud pc, const CvRect& rect)
	{
		robot_msgs::PointCloud result;
		result.header.frame_id = pc.header.frame_id;
		result.header.stamp = pc.header.stamp;

		int xchan = -1;
		int ychan = -1;

		for (size_t i=0;i<pc.chan.size();++i) {
			if (pc.chan[i].name == "x") {
				xchan = i;
			}
			if (pc.chan[i].name == "y") {
				ychan = i;
			}
		}

		int chan_size = pc.get_chan_size();
		result.chan.resize(chan_size);
		for (int j=0;j<chan_size;++j) {
			result.chan[j].name = pc.chan[j].name;
		}

		if (xchan!=-1 && ychan!=-1) {
			for (size_t i=0;i<pc.pts.size();++i) {
				int x = (int)pc.chan[xchan].vals[i];
				int y = (int)pc.chan[ychan].vals[i];
				if (x>=rect.x && x<rect.x+rect.width && y>=rect.y && y<rect.y+rect.height) {
					result.pts.push_back(pc.pts[i]);
					for (int j=0;j<chan_size;++j) {
						result.chan[j].vals.push_back(pc.chan[j].vals[i]);
					}
				}
			}
		}

		return result;
	}

    /**
     * \brief Service call to spot outlets
     */
    bool outletSpottingService(outlet_detection::OutletDetection::Request & req, outlet_detection::OutletDetection::Response & resp)
    {
    	ROS_INFO("OutletSpotter: service called");

    	bool found = runOutletSpotter(req.point,resp.pose);

    	return found;
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

		if (lbridge.fromImage(limage, "bgr"))
		{
			if(left != NULL)
				cvReleaseImage(&left);
			left = cvCloneImage(lbridge.toIpl());
		}

//		if (rbridge.fromImage(rimage, "bgr"))
//		{
//			if(right != NULL)
//				cvReleaseImage(&right);
//
//			right = cvCloneImage(rbridge.toIpl());
//		}

		if (dbridge.fromImage(dimage))
		{
			if(disp != NULL)
				cvReleaseImage(&disp);

			disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
			cvCvtScale(dbridge.toIpl(), disp, 4.0/dispinfo.dpp);
		}

		cloud = cloud_fetch;

		if (disp_clone!=NULL)
			cvReleaseImage(&disp_clone);

		have_images_ = true;
		data_cv_.notify_all();
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

		if (cloud_fetch.header.stamp != t)
			printf("Timed out waiting for point cloud\n");

	}


	void turnHead(float horizontalAngle, float verticalAngle)
	{

	//	if (horizontalAngle==0 && verticalAngle==0) return;
		PointStamped point;

		point.header.stamp = ros::Time::now();
		point.header.frame_id = "torso_lift_link";

		point.point.x = 1;
		point.point.y = tan(M_PI*horizontalAngle/180);
		point.point.z = tan(M_PI*verticalAngle/180);

		publish(head_controller_ + "/head_track_point", point);
		ros::Duration(1.0).sleep();
	}


	bool runDetectLoop(robot_msgs::PoseStamped& pose)
	{
		// lock data
		boost::unique_lock<boost::mutex> images_lock(data_lock_);

		for (int i=0;i<frames_number_;++i) {
			ROS_INFO("OutletSpotter: waiting for images and base laser data");
        	start_image_wait_ = ros::Time::now();
        	preempt_ = false;
			// want new data for next detection
			have_images_ = false;
			have_cloud_point_ = false;
			while (!(have_images_ && have_cloud_point_) && !preempt_) {
				data_cv_.wait(images_lock);
			}
			if (preempt_) {
				ROS_INFO("OutletSpotter: detect loop preempted");
				return false;
			}
			ROS_INFO("OutletSpotter: received images and base laser data, performing detection");

			bool found = detectOutlet(pose);

			showMarkers(pose);

			if (display) {
				cvShowImage("left", left);
				//cvShowImage("right", right);
				cvShowImage("contours", disp);
				cvShowImage("disparity", disp_clone);
			}

			if (found) {
				ROS_INFO("OutletSpotter: Hooray! I've found an outlet. I can taste that sweet electricity already... let's go there!\n");
				return true;;
			}
			else {
				if (i<frames_number_-1) {
					ROS_INFO("OutletSpotter: I didn't find any outlets, I'll try again... fingers crossed... I mean grippers crossed... oh, never mind.\n");
				}

			}
		}

		ROS_INFO("OutletSpotter: I didn't find any outlets :(. I this goes on, I'll starve to death... I mean to complete discharge.\n");
		return false;
	}

public:

	bool spin()
	{
		while (ok())
		{
			data_lock_.lock();

			if (!have_images_ && !have_cloud_point_) {
				if ((ros::Time::now()-start_image_wait_) > ros::Duration(timeout_)) {
					preempt_ = true;
					data_cv_.notify_all();
				}
			}

			int key = cvWaitKey(3)&0x00FF;
			data_lock_.unlock();
			if(key == 27) //ESC
				break;

			if (key=='s')
				save_patches ^= true;

			usleep(10000);
		}

		return true;
	}



	bool runOutletSpotter(const robot_msgs::PointStamped& request, robot_msgs::PoseStamped& pose)
	{
		bool found = false;
		subscribeToData();

		//		float directions[][2] = { {0,0}, {-15,-10}, {0,-10}, {15,-10}, {15,0}, {0,0}, {-15,0,},{-15,10}, {0,10},{15,10} };
		float directions[][2] = {{0, 0},{0, -10},{0, 10},{0, -20},{0, 20},{-10, 0},{-10, -10},
					 {-10, 10},{-10, -20},{-10, 20},{10, 0},{10, -10},{10, 10},{10, -20},
					 {10, 20},{-20, 0},{-20, -10},{-20, 10},{-20, -20},{-20, 20},{20, 0},
					 {20, -10},{20, 10},{20, -20},{20, 20}};

//



		for (size_t k=0; k<sizeof(directions)/sizeof(directions[0]); ++k)
		{
			turnHead(directions[k][0],directions[k][1]);
			found = runDetectLoop(pose);

			if (found) break;
		}

		unsubscribeFromData();

//		showMarkers(pose);

		return found;
	}



};




int main(int argc, char **argv)
{
	ros::init(argc, argv);
	OutletSpotting spotter;


	spotter.spin();

	return 0;
}

