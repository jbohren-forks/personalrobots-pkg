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

#include "opencv_latest/CvBridge.h"

// opencv
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
#include "robot_msgs/PoseStamped.h"
#include "visualization_msgs/VisualizationMarker.h"


#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include "topic_synchronizer/topic_synchronizer.h"
#include <tf/transform_listener.h>

#include "CvStereoCamModel.h"
#include "outlet_detection/outlet_tuple_coarse.h"

#include "outlet_detection/OutletDetection.h"
#include "robot_actions/action.h"
#include "robot_actions/action_runner.h"
#include "pr2_robot_actions/DetectOutletState.h"

#include <point_cloud_assembler/BuildCloudAngle.h>

#include <boost/thread.hpp>

using namespace std;
using namespace robot_msgs;


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


static const double scan_speed  = 0.1; // [m/sec]
static const double scan_height = 0.4; //[m]

class OutletSpotting : public ros::Node
{
public:

	image_msgs::Image limage;
//	image_msgs::Image rimage;
	image_msgs::Image dimage;
	image_msgs::StereoInfo stinfo;
	image_msgs::DisparityInfo dispinfo;
	image_msgs::CamInfo rcinfo;

	image_msgs::CvBridge lbridge;
//	image_msgs::CvBridge rbridge;
	image_msgs::CvBridge dbridge;

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


	boost::mutex clound_point_mutex;
	boost::condition_variable cloud_point_cv;

	boost::mutex cv_mutex;
	boost::condition_variable images_cv;
	bool have_cloud_point_;


	tf::TransformListener *tf_;

	int frames_number_;
	string target_frame_;
	string base_scan_topic_;

	OutletSpotting() : ros::Node("stereo_view"),left(NULL), disp(NULL), disp_clone(NULL),
		sync(this, &OutletSpotting::image_cb_all, ros::Duration().fromSec(0.1), &OutletSpotting::image_cb_timeout), have_cloud_point_(false)

	{
        tf_ = new tf::TransformListener(*this);

		param ("~display", display, false);
		param ("~save_patches", save_patches, false);
		param ("~frames_number", frames_number_, 7);
		param<string> ("~target_frame", target_frame_, "odom_combined");
		param<string> ("~base_scan_topic", base_scan_topic_, "base_scan_filtered");

		if (display) {
			ROS_INFO("Displaying images\n");
			cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
			//cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("contours", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
		}

        advertise<visualization_msgs::VisualizationMarker>("visualizationMarker", 1);
        advertiseService("~coarse_outlet_detect", &OutletSpotting::outletSpottingService, this);
	}

	~OutletSpotting()
	{
		if (left)
			cvReleaseImage(&left);
//		if (right)
//			cvReleaseImage(&right);
		if (disp)
			cvReleaseImage(&disp);

		unadvertise("visualizationMarker");
		unadvertiseService("~coarse_outlet_detect");
	}

private:

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
        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");
//        unsubscribe("full_cloud");
    }





	/////WORKING FUNCTIONS////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	// JUST A MODIFIED CONNECTED COMPONENTS ROUTINE
	//void cvconnectedDisparityComponents(IplImage *mask, int poly1_hull0, float perimScaleTooSmall, int *num, CvRect *bbs, CvPoint *centers)
	// This will find the blobs in the image.  I
	//
	// mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
	//
	// OPTIONAL PARAMETERS:
	// poly1_hull0				If set, approximate connected component by (DEFAULT) polygon, or else convex hull (0)
	// areaTooSmall			Kill contours whose bounding box area is less than this
	// areaTooLarg				Kill contours whose bounding box area is more than this
	// aspectLimit				Enforce width > height. Then aspect = height/width.  Kill aspects < aspectLimit.
	// num						Maximum number of rectangles and/or centers to return, on return, will contain number filled (DEFAULT: NULL)
	// bbs						Pointer to bounding box rectangle vector of length num.  (DEFAULT SETTING: NULL)
	// centers					Pointer to contour centers vectore of length num (DEFULT: NULL)
	//
	// KNOWN PROBLEM!!!  If a rejected contour compltely surrounds another contour, the whole area is deleted
	//
	#define CVCONTOUR_APPROX_LEVEL  2   // Approx.threshold - the bigger it is, the simpler is the boundary
	#define CV_CVX_WHITE	CV_RGB(0xff,0xff,0xff)
	#define CV_CVX_BLACK	CV_RGB(0x00,0x00,0x00)
	void cvconnectedDisparityComponents(IplImage *mask, int poly1_hull0, float areaTooSmall, float areaTooLarge, float aspectLimit, float fillFactor, int *num, CvRect *bbs, CvPoint *centers)
	{
		static CvMemStorage*	mem_storage	= NULL;
		static CvSeq*			contours	= NULL;

		//FIND CONTOURS AROUND ONLY BIGGER REGIONS
		if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
		else cvClearMemStorage(mem_storage);

		CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
		CvSeq* c;
		int numCont = 0;
		CvRect bb;
		//FILTER LOOP:
		while( (c = cvFindNextContour( scanner )) != NULL )
		{
//			printf("CONTOR %d:\n",ccnt++);
			bb = cvBoundingRect(c);
			float wd = (float)(bb.width);
			float ht = (float)(bb.height);
			float carea = (float)(wd*ht);
//			printf("  Area = %f vs too small(%f) and too large (%f)\n",carea,areaTooSmall,areaTooLarge);
			float aspect;
			if(wd > ht) //Keep this number (0,1]
				aspect = ht/wd;
			else
				aspect = wd/ht;
//			printf("  ... aspectLimit(%f) vs aspect(%f)\n",aspectLimit,aspect);
			if( carea <  areaTooSmall || carea > areaTooLarge || (aspect < aspectLimit)) //Get rid of blob if it's too small or too large
			{
	//			printf("  DELETED\n"); //If bad contour surrounds a good one, both are delteted here :-(
				cvSubstituteContour( scanner, NULL );
			}
			else //Smooth it's edges if it's large enough
			{
				CvSeq* c_new;
				if(poly1_hull0) //Polygonal approximation of the segmentation
					c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
				else //Convex Hull of the segmentation
					c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
				//cvSubstituteContour( scanner, c_new );
				numCont++;
			}
		}
		contours = cvEndFindContours( &scanner );

		//COMPUTE LOOP:
		// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
		cvZero( mask );
		IplImage *maskTemp = cvCloneImage(mask);
		//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
		if(num != NULL)
		{
			int N = *num, numFilled = 0, i=0;
			//maskTemp = cvCloneImage(mask);
			for(i=0, c=contours; c != NULL; c = c->h_next,i++ )
			{
				if(i < N) //Only process up to *num of them
				{
					double M00, M01, M10;
					cvZero(maskTemp);
					cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
					//Find the center of each contour
					if(centers != NULL)
					{
						CvMoments moments;
						cvMoments(maskTemp,&moments,1);
						M00 = cvGetSpatialMoment(&moments,0,0);
						M10 = cvGetSpatialMoment(&moments,1,0);
						M01 = cvGetSpatialMoment(&moments,0,1);
						centers[i].x = (int)(M10/M00);
						centers[i].y = (int)(M01/M00);
					}
					//Bounding rectangles around blobs
					if(bbs != NULL)
					{
						bbs[i] = cvBoundingRect(c);
					}

//					int area = bbs[i].width*bbs[i].height;
//					if (M00<fillFactor*area) {
//						--i;
//						printf("Contour shape too distorted... DELETED\n");
//						continue;
//					}


					numFilled++;
				}
				//Draw filled contours into mask
				cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
			} //end looping over contours
			*num = numFilled;
			cvReleaseImage( &maskTemp);
		}
		//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
		else
		{
			for( c=contours; c != NULL; c = c->h_next )
			{
				cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
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
	double disparitySTD(IplImage *Id, CvRect &R, bool &vertical, double& meanDisparity, double minDisparity = 0.5 )
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
		double AvgTopDisp = 0.0;
		double AvgBotDisp = 0.0;
		int top_cnt = 0;
		int bot_cnt = 0;
		bool top = true;
		int halfwayY = rh/2;
		for(int Y=0; Y<rh; ++Y)
		{
			for(int X=0; X<rw; X++, p+=nchan)
			{
				val = (double)*p;
				if(val < minDisparity)
					continue;
				if(top){
					AvgTopDisp += val;
					top_cnt++;
				}
				else {
					AvgBotDisp += val;
					bot_cnt++;
				}
				mean += val;
				var += val*val;
				cnt++;
			}
			p+=ws-(rw*nchan);
			if(Y >= halfwayY)
				top = false;
		}
		if(cnt == 0) //Error condition, no disparities, return impossible variance
		{
			vertical = false;
			return 10000000.0;
		}
		//FIND OUT IF THE OBJECT IS VERTICAL (Descending disparities) OR NOT:
		if(top_cnt == 0) top_cnt = 1;
		if(bot_cnt == 0) bot_cnt = 1;
		AvgTopDisp = AvgTopDisp/(double)top_cnt;
		AvgBotDisp = AvgBotDisp/(double)bot_cnt;
		if(AvgTopDisp >= AvgBotDisp)
			vertical = true;
		else
			vertical = false;
		//DO THE VARIANCE MATH
		mean = mean/(double)cnt;
		var = (var/(double)cnt) - mean*mean;
		meanDisparity = mean;
		return(sqrt(var));
	}


	CvPoint3D32f disp_to_3D_point(CvStereoCamModel& cam_model, int x, int y, double d)
	{
		CvMat* uvd = cvCreateMat(1,3,CV_32FC1);
		cvmSet(uvd,0,0,x);
		cvmSet(uvd,0,1,y);
		cvmSet(uvd,0,2,d);
		CvMat* xyz = cvCreateMat(1,3,CV_32FC1);
		cam_model.dispToCart(uvd,xyz);
		CvPoint3D32f result;
		result.x = cvmGet(xyz,0,0);
		result.y = cvmGet(xyz,0,1);
		result.z = cvmGet(xyz,0,2);
		return result;
	}


	double cvDist(CvPoint3D32f a, CvPoint3D32f b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
	}

	void getBBoxDimensions(IplImage *Id, CvRect &R, double meanDisparity, double& dx, double& dy)
	{
		// initialize stereo camera model
		double Fx = rcinfo.P[0];  double Fy = rcinfo.P[5];
		double Clx = rcinfo.P[2]; double Crx = Clx;
		double Cy = rcinfo.P[6];
		double Tx = -rcinfo.P[3]/Fx;
		CvStereoCamModel cam_model(Fx,Fy,Tx,Clx,Crx,Cy,4.0/(double)dispinfo.dpp);

		int rx = R.x;
		int ry = R.y;
		int rw = R.width;
		int rh = R.height;

		CvPoint3D32f p1 = disp_to_3D_point(cam_model, rx, ry, meanDisparity);
		CvPoint3D32f p2 = disp_to_3D_point(cam_model, rx+rw, ry, meanDisparity);
		CvPoint3D32f p3 = disp_to_3D_point(cam_model, rx, ry+rh, meanDisparity);

		dx = cvDist(p1,p2);
		dy = cvDist(p1,p3);
	}


	void savePatch(IplImage* img, CvRect& R, const char* prefix)
	{
		static int patch_cnt = 1;

		char name[100];
		sprintf(name,"%s/../ros-pkg/vision/stereo_capture/data/%s_%.3d.png", getenv("ROS_ROOT"), prefix, patch_cnt++);
		printf("Saving patch %s\n",name);
		cvSetImageROI(img,R);
		cvSaveImage(name, img);
		cvResetImageROI(img);
	}


	/**
	 * \brief Finds the nearest point with non-zero disparity
	 * @param r
	 * @return
	 */
	bool findCenterPoint(IplImage* disp_image, const CvRect& r, CvPoint& p)
	{
		int dir[][2] = { {1,0}, {0,1}, {-1,0}, {0,-1}};

		int d = 0;
		int cnt = 1;
		int i=0;
		IndexedIplImage<unsigned char> img(disp_image);
		while (r.x<=p.x && r.y<=p.y && r.x+r.width>p.x && r.y+r.height>p.y) {

			if (img.at(p.x,p.y)!=0) {
				return true;
			}
			p.x += dir[d][0];
			p.y += dir[d][1];
			i++;
			if (i==cnt) {
				i=0;
				d = (d+1) %4;
				if (d%2==0) {
					cnt++;
				}
			}
		}

		return false;
	}


	bool find3DPoint(const robot_msgs::PointCloud& pc, const CvPoint& p, robot_msgs::Point32& center_point)
	{
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

		if (xchan!=-1 && ychan!=-1) {
			for (size_t i=0;i<pc.pts.size();++i) {
				int x = (int)pc.chan[xchan].vals[i];
				int y = (int)pc.chan[ychan].vals[i];
				if (x==p.x && y==p.y) {
					center_point = pc.pts[i];
					return true;
				}
			}
		}
		return false;
	}



    /**
     * \brief Publishes a visualization marker for a point.
     * @param p
     */
    void showMarkers(robot_msgs::PoseStamped pose)
    {
        visualization_msgs::VisualizationMarker marker;
        marker.header.frame_id = pose.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)(0ULL));
        marker.id = 101;
        marker.type = visualization_msgs::VisualizationMarker::SPHERE;
        marker.action = visualization_msgs::VisualizationMarker::ADD;

        marker.x = pose.pose.position.x;
        marker.y = pose.pose.position.y;
        marker.z = pose.pose.position.z;
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


        tf::Pose tf_pose;

        tf::PoseMsgToTF(pose.pose,tf_pose);
        tf::Point point(-1,0,0);
        tf::Point normal = tf_pose*point;


        marker.header.frame_id = pose.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)(0ULL));
        marker.id = 102;
        marker.type = visualization_msgs::VisualizationMarker::SPHERE;
        marker.action = visualization_msgs::VisualizationMarker::ADD;

        marker.x = normal.x();
        marker.y = normal.y();
        marker.z = normal.z();
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


    double squaredPointDistance(Point32 p1, Point p2)
    {
    	return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
    }


    PointCloud outletVecinity(PointCloud laser_cloud, PointStamped ps_cloud, double distance)
    {
    	PointCloud result;
    	result.header.frame_id = laser_cloud.header.frame_id;
    	result.header.stamp = laser_cloud.header.stamp;

    	double d = distance*distance;
    	for (size_t i=0; i<laser_cloud.get_pts_size(); ++i) {
    		if (squaredPointDistance(laser_cloud.pts[i],ps_cloud.point)<d) {
    			result.pts.push_back(laser_cloud.pts[i]);
    		}
    	}

    	return result;
    }

//
//    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    /** \brief Find a plane model in a point cloud given via a set of point indices with SAmple Consensus methods
//      * \param points the point cloud message
//      * \param indices a pointer to a set of point cloud indices to test
//      * \param inliers the resultant planar inliers
//      * \param coeff the resultant plane coefficients
//      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
//      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
//      * \param min_pts the minimum number of points allowed as inliers for a plane model
//      */
//    bool
//      fitSACPlane (PointCloud &points, vector<int> indices, vector<int> &inliers, vector<double> &coeff,
//                   const robot_msgs::PointStamped &viewpoint_cloud, double dist_thresh, int min_pts)
//    {
//      if ((int)indices.size () < min_pts)
//      {
//        inliers.resize (0);
//        coeff.resize (0);
//        return (false);
//      }
//
//      // Create and initialize the SAC model
//      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
//      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
//      //sample_consensus::SAC *sac             = new sample_consensus::LMedS (model, dist_thresh);
//      sac->setMaxIterations (500);
//      model->setDataSet (&points, indices);
//
//      // Search for the best plane
//      if (sac->computeModel ())
//      {
//        // Obtain the inliers and the planar model coefficients
//        if ((int)sac->getInliers ().size () < min_pts)
//        {
//          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
//          inliers.resize (0);
//          coeff.resize (0);
//          return (false);
//        }
//        sac->computeCoefficients (coeff);          // Compute the model coefficients
//        sac->refineCoefficients (coeff);           // Refine them using least-squares
//        model->selectWithinDistance (coeff, dist_thresh, inliers);
//        //inliers = sac->getInliers ();
//
//        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.pts.at (inliers[0]), viewpoint_cloud);
//
//        //ROS_DEBUG ("Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
//        //           coeff[0], coeff[1], coeff[2], coeff[3]);
//
//        // Project the inliers onto the model
//        model->projectPointsInPlace (inliers, coeff);
//      }
//      else
//      {
//        ROS_ERROR ("Could not compute a plane model.");
//        return (false);
//      }
//      sort (inliers.begin (), inliers.end ());
//
//      delete sac;
//      delete model;
//      return (true);
//    }




    bool fitSACLine(PointCloud& points, vector<int> indices, vector<double> &coeff, double dist_thresh, int min_pts, vector<Point32> &line_segment)
    {
    	Point32 minP, maxP;

    	// Create and initialize the SAC model
    	sample_consensus::SACModelLine *model = new sample_consensus::SACModelLine ();
    	sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
    	sac->setMaxIterations (100);
    	model->setDataSet (&points, indices);

    	if(sac->computeModel())
    	{
    		if((int) sac->getInliers().size() < min_pts) {
    			coeff.resize (0);
    			return (false);
    		}
    		sac->computeCoefficients (coeff);

    		Point32 minP, maxP;
    		cloud_geometry::statistics::getLargestDiagonalPoints(points, sac->getInliers(), minP, maxP);
    		line_segment.push_back(minP);
    		line_segment.push_back(maxP);


    		fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", (int)sac->getInliers ().size (), coeff[0], coeff[1], coeff[2], coeff[3]);
    	}

    	delete sac;
    	delete model;
    	return true;
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


    void showLineMarker(const vector<Point32>& line_segment)
    {
    	visualization_msgs::VisualizationMarker marker;
    	marker.header.frame_id = base_cloud_.header.frame_id;
    	marker.header.stamp = ros::Time((uint64_t)0ULL);
    	marker.id = 102;
    	marker.type = visualization_msgs::VisualizationMarker::LINE_STRIP;
    	marker.action = visualization_msgs::VisualizationMarker::ADD;
    	marker.x = 0.0;
    	marker.y = 0.0;
    	marker.z = 0.0;
    	marker.yaw = 0.0;
    	marker.pitch = 0.0;
    	marker.roll = 0.0;
    	marker.xScale = 0.01;
    	marker.yScale = 0.1;
    	marker.zScale = 0.1;
    	marker.alpha = 255;
    	marker.r = 0;
    	marker.g = 255;
    	marker.b = 0;
    	marker.set_points_size(2);

    	marker.points[0].x = line_segment[0].x;
    	marker.points[0].y = line_segment[0].y;
    	marker.points[0].z = line_segment[0].z;

    	marker.points[1].x = line_segment[1].x;
    	marker.points[1].y = line_segment[1].y;
    	marker.points[1].z = line_segment[1].z;

    	publish( "visualizationMarker", marker );

    }

	bool getPoseStamped(const CvRect& r, const CvPoint& p, PoseStamped& pose)
	{
		CvPoint cp = p;
		bool found = findCenterPoint(disp, r, cp);
		if (!found) {
			return false;
		}

		robot_msgs::PointCloud outlet_cloud = filterPointCloud(r);
		robot_msgs::Point32 center_point;
		found = find3DPoint(outlet_cloud, cp, center_point);
		if (!found) {
			return false;
		}

		PointStamped ps_stereo;
		ps_stereo.header.frame_id = cloud.header.frame_id;
		ps_stereo.header.stamp = cloud.header.stamp;
		ps_stereo.point.x = center_point.x;
		ps_stereo.point.y = center_point.y;
		ps_stereo.point.z = center_point.z;

		ROS_INFO("OutletSpotter: Found outlet bbox, I'm waiting for cloud point.");

		boost::unique_lock<boost::mutex> lock(clound_point_mutex);

		// waiting for the cloud point
		while (!have_cloud_point_) {
			cloud_point_cv.wait(lock);
		}


//		PointCloud laser_scan;
//		bool laser_found;
//		try {
//			laser_found = getLaserScan(ps_stereo, laser_scan);
//		}
//		catch(tf::TransformException & ex){
//			ROS_ERROR("Transform exception: %s\n", ex.what());
//			return false;
//		}

//		if (!laser_found) {
//			ROS_ERROR("OutletSpotter: Cannot get laser scan, aborting detection");
//		}

		ROS_INFO("OutletSpotter: fit line in base point cloud.");

		// got a cloud point
		// transform outlet location to cloud frame
		PointStamped ps_cloud;
        try {
            tf_->transformPoint(base_cloud_.header.frame_id, ps_stereo, ps_cloud);
        }
        catch(tf::TransformException & ex){
            ROS_ERROR("Transform exception: %s\n", ex.what());
        	return false;
        }
        PointCloud outlet_vecinity = outletVecinity(base_cloud_, ps_cloud, 0.4);

        // fit a line in the outlet cloud
        vector<int> indices(outlet_vecinity.pts.size());
        for (size_t i=0;i<outlet_vecinity.get_pts_size();++i) {
        	indices[i] = i;
        }
        vector<double> coeff(4);	// plane coefficients

        double dist_thresh = 0.02;
        int min_pts = 10;

        vector<Point32> line_segment;
		ROS_INFO("OutletSpotter: finding wall orientation");
        if ( !fitSACLine(outlet_vecinity, indices, coeff, dist_thresh, min_pts, line_segment) ) {
        	ROS_ERROR ("Cannot find line in laser scan, aborting...");
        	return false;
        }

        showLineMarker(line_segment);

        PoseStamped temp_pose;

        // fill the outlet pose
        temp_pose.header.frame_id = base_cloud_.header.frame_id;
        temp_pose.header.stamp = base_cloud_.header.stamp;

        btVector3 position(ps_cloud.point.x,ps_cloud.point.y,ps_cloud.point.z);
        btVector3 up(0,0,1);
        btVector3 left(line_segment[1].x-line_segment[0].x,line_segment[1].y-line_segment[0].y,line_segment[1].z-line_segment[0].z);
        btVector3 normal = left.cross(up).normalized();


        btMatrix3x3 rotation;
        rotation[0] = normal; // x
        rotation[1] = left; // y
        rotation[2] = up;     // z
        rotation = rotation.transpose();
        btQuaternion orientation;
        rotation.getRotation(orientation);
        tf::Transform outlet_pose(orientation, position);
        tf::PoseTFToMsg(outlet_pose, temp_pose.pose);



        try {
        	tf_->transformPose(target_frame_, temp_pose, pose);
        }
        catch(tf::TransformException & ex){
        	ROS_ERROR("Transform exception: %s\n", ex.what());
        	return false;
        }

        ROS_INFO("OutletSpotter: finished computing outlet pose");

		return true;
	}

	bool detectOutlet(PoseStamped& pose)
	{
		int Nerode = 1;
		int Ndialate = 7;
		int areaTooSmall = 25; //bounding box area
		int areaTooLarge = 200;
		int aspectLimit = 45;
		int fillFactor = 0.5;
		int num = 20;
		CvRect bbs[num];
		CvPoint centers[num];
		bool vertical = false;

		disp_clone = cvCloneImage(disp);
//
//		cvErode(disp,disp,NULL, Nerode); //Probably 1
//		cvDilate(disp,disp,NULL,Ndialate); //Probably 9
//		cvErode(disp,disp,NULL, Ndialate);

		cvconnectedDisparityComponents(disp, 1, areaTooSmall*areaTooSmall,
				areaTooLarge*areaTooLarge,(float)(aspectLimit)/100.0, fillFactor,
				&num, bbs, centers);

		for(int t=0; t<num; ++t)
		{
			CvPoint pt1 = cvPoint(bbs[t].x,bbs[t].y);
			int wi = bbs[t].width;
			int hi = bbs[t].height;
//			printf("Box %d: w=%d, h=%d\n",t,wi,hi);
			double meanDisparity;
			double Disp = disparitySTD(disp_clone, bbs[t],vertical,meanDisparity);
//			printf("** STD = %lf, vertical=%d\n",Disp,vertical);
			if(Disp >= 4.8) {
				if (save_patches) savePatch(left,bbs[t],"outlet_high_std");
				continue;
			}
//			if(!vertical) {
//				if (save_patches) savePatch(left,bbs[t],"outlet_not_vertical");
//				continue;
//			}


			// check bonding box real world dimensions
			double dx;
			double dy;
			getBBoxDimensions(disp_clone, bbs[t], meanDisparity, dx, dy);
			if (dx<0.05 || dx>0.26) {
				if (save_patches) savePatch(left,bbs[t],"outlet_wrong_dimensions");
				continue;
			}
			if (dy<0.05 || dy>0.26) {
				if (save_patches) savePatch(left,bbs[t],"outlet_wrong_dimensions");
				continue;
			}

			// if we made it this far, check the patch to see if it's an outlet
			IplImage* patch = cvCreateImage(cvSize(bbs[t].width, bbs[t].height), IPL_DEPTH_8U, 1);
			cvCopyPatch(left,patch, bbs[t]);
			CvPoint2D32f centroids[4];
			bool found = find_outlet_centroids(patch, centroids, 1);
			cvReleaseImage(&patch);

			if (!found) {
				if (save_patches) savePatch(left,bbs[t],"outlet_failed_vision");
				continue;
			}
			if (save_patches) savePatch(left,bbs[t],"outlet_match");


			if (display) {
				// draw bounding box
				CvPoint pt2 = cvPoint(bbs[t].x+wi,bbs[t].y+hi);
				cvRectangle(left,pt1,pt2,CV_RGB(0,255,0));
			}
			found = getPoseStamped(bbs[t], centers[t], pose);

			// one outlet is enough
			if (found) {
				return true;
			}
		}

		ROS_INFO("OutletSpotter: outlet not found");
		return false;
	}


	// TODO: replace this with version using opencv functions
	void cvCopyPatch(IplImage* src, IplImage* patch, CvRect &R)
	{
		int ws = src->widthStep;
		unsigned char *p = (unsigned char *)(src->imageData);
		int rx = R.x;
		int ry = R.y;
		int rw = R.width;
		int rh = R.height;
		int nchan = src->nChannels;
		p += ws*ry + rx*nchan; //Put at start of box
		unsigned char *d = (unsigned char *) patch->imageData;
		int dchan = patch->nChannels;
		int dws = patch->widthStep;
		for(int Y=0; Y<rh; ++Y)
		{
			for(int X=0; X<rw; X++, p+=nchan, d+=dchan)
			{
				*d = *p;
			}
			p+=ws-(rw*nchan);
			d+=dws-(rw*dchan);
		}
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

		int chan_size = cloud.get_chan_size();
		result.chan.resize(chan_size);
		for (int j=0;j<chan_size;++j) {
			result.chan[j].name = cloud.chan[j].name;
		}

		if (xchan!=-1 && ychan!=-1) {
			for (size_t i=0;i<cloud.pts.size();++i) {
				int x = (int)cloud.chan[xchan].vals[i];
				int y = (int)cloud.chan[ychan].vals[i];
				if (x>=rect.x && x<rect.x+rect.width && y>=rect.y && y<rect.y+rect.height) {
					result.pts.push_back(cloud.pts[i]);
					for (int j=0;j<chan_size;++j) {
						result.chan[j].vals.push_back(cloud.chan[j].vals[i]);
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
		boost::lock_guard<boost::mutex> lock(clound_point_mutex);
		have_cloud_point_ = true;
		base_cloud_ = base_cloud_fetch_;

		cloud_point_cv.notify_all();
	}


	void image_cb_all(ros::Time t)
	{
        boost::lock_guard<boost::mutex> lock(cv_mutex);

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


		images_cv.notify_all();
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

public:

	bool spin()
	{
		while (ok())
		{
			int key = cvWaitKey(3)&0x00FF;
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

        {
        	boost::unique_lock<boost::mutex> images_lock(cv_mutex);
        	for (int i=0;i<frames_number_;++i) {
        		ROS_INFO("OutletSpotter: waiting for images");
        		images_cv.wait(images_lock);

        		ROS_INFO("OutletSpotter: performing detection");
        		found = detectOutlet(pose);
        		if (found) break;

        		if (display) {
        			cvShowImage("left", left);
        			//cvShowImage("right", right);
        			cvShowImage("contours", disp);
        			cvShowImage("disparity", disp_clone);
        		}
        	}
        }

        unsubscribeFromData();

        showMarkers(pose);

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

