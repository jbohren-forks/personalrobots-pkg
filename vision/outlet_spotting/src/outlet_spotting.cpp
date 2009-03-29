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

#include "image_msgs/CvBridge.h"

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
#include "robot_msgs/BoundingBoxStamped.h"
#include "std_msgs/UInt8.h" //for projector status


#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include "topic_synchronizer.h"

#include "CvStereoCamModel.h"
#include "outlet_tuple.h"

#include <boost/thread.hpp>

using namespace std;
using namespace robot_msgs;

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

	robot_msgs::BoundingBoxStamped outlet_bbox;

	IplImage* left;
//	IplImage* right;''
	IplImage* disp;
	IplImage* disp_clone;

	bool display;
	bool save_patches;

	TopicSynchronizer<OutletSpotting> sync;


	boost::mutex cv_mutex;


	OutletSpotting() : ros::Node("stereo_view"),left(NULL), disp(NULL), disp_clone(NULL),
		sync(this, &OutletSpotting::image_cb_all, ros::Duration().fromSec(0.05), &OutletSpotting::image_cb_timeout)

	{

		param ("~display", display, false);  // 100 points at high resolution
		param ("~save_patches", save_patches, false);  // 100 points at high resolution


		if (display) {
			ROS_INFO("Displaying images\n");
			cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
			//cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("contours", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
		}


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

		sync.subscribe("stereo/cloud", cloud, 1);

		sync.ready();

		advertise<BoundingBoxStamped>("stereo/outlet_bbox",1);
	}

	~OutletSpotting()
	{
		if (left)
			cvReleaseImage(&left);
//		if (right)
//			cvReleaseImage(&right);
		if (disp)
			cvReleaseImage(&disp);
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
			CvMoments moments;
			double M00, M01, M10;
			//maskTemp = cvCloneImage(mask);
			for(i=0, c=contours; c != NULL; c = c->h_next,i++ )
			{
				if(i < N) //Only process up to *num of them
				{
					cvZero(maskTemp);
					cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
					//Find the center of each contour
					if(centers != NULL)
					{
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

					int area = bbs[i].width*bbs[i].height;
					if (M00<fillFactor*area) {
						--i;
						printf("Contour shape too distorted... DELETED\n");
						continue;
					}


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

	void getBBoxDimensions(IplImage *Id, CvRect &R, double meanDisparity, double& dx, double& dy, double& z)
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

		z = p1.z;
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


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Find a plane model in a point cloud given via a set of point indices with SAmple Consensus methods
	  * \param points the point cloud message
	  * \param indices a pointer to a set of point cloud indices to test
	  * \param inliers the resultant planar inliers
	  * \param coeff the resultant plane coefficients
	  * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
	  * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
	  * \param min_pts the minimum number of points allowed as inliers for a plane model
	  */
	bool
	  fitSACPlane (PointCloud &points, vector<int> indices, vector<int> &inliers, vector<double> &coeff,
	               const robot_msgs::Point32 &viewpoint, double dist_thresh, int min_pts)
	{
	  if ((int)indices.size () < min_pts)
	  {
	    inliers.resize (0);
	    coeff.resize (0);
	    return (false);
	  }

	  // Create and initialize the SAC model
	  sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
	 //  sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
	  sample_consensus::SAC *sac             = new sample_consensus::LMedS (model, dist_thresh);
	  sac->setMaxIterations (500);
	  model->setDataSet (&points, indices);

	  // Search for the best plane
	  if (sac->computeModel ())
	  {
	    // Obtain the inliers and the planar model coefficients
	    if ((int)sac->getInliers ().size () < min_pts)
	    {
//	      ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), min_pts);
	      inliers.resize (0);
	      coeff.resize (0);
	      return (false);
	    }
	    sac->computeCoefficients ();          // Compute the model coefficients
	    coeff   = sac->refineCoefficients (); // Refine them using least-squares
	    model->selectWithinDistance (coeff, dist_thresh, inliers);

	    // Flip plane normal according to the viewpoint information
	    Point32 vp_m;
	    vp_m.x = viewpoint.x - points.pts.at (inliers[0]).x;
	    vp_m.y = viewpoint.y - points.pts.at (inliers[0]).y;
	    vp_m.z = viewpoint.z - points.pts.at (inliers[0]).z;

	    // Dot product between the (viewpoint - point) and the plane normal
	    double cos_theta = (vp_m.x * coeff[0] + vp_m.y * coeff[1] + vp_m.z * coeff[2]);

	    // Flip the plane normal
	    if (cos_theta < 0)
	    {
	      for (int d = 0; d < 3; d++)
	        coeff[d] *= -1;
	      // Hessian form (D = nc . p_plane (centroid here) + p)
	      coeff[3] = -1 * (coeff[0] * points.pts.at (inliers[0]).x + coeff[1] * points.pts.at (inliers[0]).y + coeff[2] * points.pts.at (inliers[0]).z);
	    }
	    //ROS_DEBUG ("Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
	    //           coeff[0], coeff[1], coeff[2], coeff[3]);
	  }
	  else
	  {
	    ROS_ERROR ("Could not compute a plane model.");
	    return (false);
	  }
	  delete sac;
	  delete model;
	  return (true);
	}


	bool getOutletBBox(const CvRect& r, Point32 &p1, Point32 &p2)
	{
		vector<int> indices;

		Point32 bb_min;
		Point32 bb_max;
		bool init_bb = false;


		// see which are the image x and y channels in the point cloud
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
			vector<float>& xcoords = cloud.chan[xchan].vals;
			vector<float>& ycoords = cloud.chan[ychan].vals;
			for (uint32_t i=0;i<cloud.get_pts_size();++i) {
				if (xcoords[i]>r.x && xcoords[i]<r.x+r.width && ycoords[i]>r.y && ycoords[i]<r.y+r.height) {

					if (init_bb) {
						bb_min.x = min(bb_min.x,cloud.pts[i].x);
						bb_min.y = min(bb_min.y,cloud.pts[i].y);
						bb_min.z = min(bb_min.z,cloud.pts[i].z);
						bb_max.x = max(bb_max.x,cloud.pts[i].x);
						bb_max.y = max(bb_max.y,cloud.pts[i].y);
						bb_max.z = max(bb_max.z,cloud.pts[i].z);
					}
					else {
						bb_min = cloud.pts[i];
						bb_max = cloud.pts[i];
						init_bb = true;
					}
					// point inside the interest region
//					indices.push_back(i);
				}
			}
		}

		// fit a plane
//		vector<int> inliers;
//		vector<double> coeff;
//		Point32 viewpoint;
//		bool plane_found = fitSACPlane(cloud, indices, inliers, coeff, viewpoint, 0.1, 1000);

//		if (!plane_found) {
//			return false;
//		}


		//publish bounding_box
		outlet_bbox.header.frame_id = cloud.header.frame_id;
		outlet_bbox.header.stamp = cloud.header.stamp;

		outlet_bbox.center.x = (bb_min.x+bb_max.x)/2;
		outlet_bbox.center.y = (bb_min.y+bb_max.y)/2;
		outlet_bbox.center.z = (bb_min.z+bb_max.z)/2;

		outlet_bbox.extents.x = (bb_max.x-bb_min.x)/2;
		outlet_bbox.extents.y = (bb_max.y-bb_min.y)/2;
		outlet_bbox.extents.z = (bb_max.z-bb_min.z)/2;

		publish("stereo/outlet_bbox", outlet_bbox);

		return true;
	}

	void detectOutlet()
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

		cvErode(disp,disp,NULL, Nerode); //Probably 1
		cvDilate(disp,disp,NULL,Ndialate); //Probably 9
		cvErode(disp,disp,NULL, Ndialate);

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
			if(!vertical) {
				if (save_patches) savePatch(left,bbs[t],"outlet_not_vertical");
				continue;
			}


			// check bonding box real world dimensions
			double dx;
			double dy;
			double z;
			getBBoxDimensions(disp_clone, bbs[t], meanDisparity, dx, dy, z);
			if (dx<0.05 || dx>0.16) {
				if (save_patches) savePatch(left,bbs[t],"outlet_wrong_dimensions");
				continue;
			}
			if (dy<0.05 || dy>0.16) {
				if (save_patches) savePatch(left,bbs[t],"outlet_wrong_dimensions");
				continue;
			}

			// if we made it this far, check the patch to see if it's an outlet
			IplImage* patch = cvCreateImage(cvSize(bbs[t].width, bbs[t].height), IPL_DEPTH_8U, 1);
			cvCopyPatch(left,patch, bbs[t]);
			CvPoint2D32f centers[4];
			bool found = find_outlet_centroids(patch, centers, 1);
			cvReleaseImage(&patch);

			if (!found) {
				if (save_patches) savePatch(left,bbs[t],"outlet_failed_vision");
				continue;
			}
			if (save_patches) savePatch(left,bbs[t],"outlet_match");


			Point32 p1;
			Point32 p2;
			getOutletBBox(bbs[t], p1, p2);

			// draw bounding box for now
			CvPoint pt2 = cvPoint(bbs[t].x+wi,bbs[t].y+hi);
			cvRectangle(left,pt1,pt2,CV_RGB(0,255,0));

		}

		// wire detection attempt
//		areaTooSmall = 160;
//		areaTooLarge = 400;
//		aspectLimit = 1;
//		fillFactor = 0;
//
//		cvCopy(disp_clone,disp);
//		cvconnectedDisparityComponents(disp, 1, areaTooSmall*areaTooSmall,
//				areaTooLarge*areaTooLarge,(float)(aspectLimit)/100.0, fillFactor,
//				&num, bbs, centers);
//
//		for(int t=0; t<num; ++t)
//		{
//			CvPoint pt1 = cvPoint(bbs[t].x,bbs[t].y);
//			int wi = bbs[t].width;
//			int hi = bbs[t].height;
//			//			printf("Box %d: w=%d, h=%d\n",t,wi,hi);
//			double meanDisparity;
//			double Disp = disparitySTD(disp_clone, bbs[t],vertical,meanDisparity);
//			//			printf("** STD = %lf, vertical=%d\n",Disp,vertical);
//			if(Disp < 10) {
//				continue;
//			}
//
//			if (vertical) {
//				continue;
//			}
//			CvPoint pt2 = cvPoint(bbs[t].x+wi,bbs[t].y+hi);
//			cvRectangle(left,pt1,pt2,CV_RGB(255,0,0));
//		}

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


	void image_cb_all(ros::Time t)
	{
		cv_mutex.lock();

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

		detectOutlet();

		if (display) {
			cvShowImage("left", left);
			//cvShowImage("right", right);
			cvShowImage("contours", disp);
			cvShowImage("disparity", disp_clone);
		}

		if (disp_clone!=NULL)
			cvReleaseImage(&disp_clone);


		cv_mutex.unlock();

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

		if (cloud.header.stamp != t)
			printf("Timed out waiting for point cloud\n");

		//Proceed to show images anyways
//		image_cb_all(t);
	}

	bool spin()
	{
		while (ok())
		{
			cv_mutex.lock();
			int key = cvWaitKey(3)&0x00FF;
			if(key == 27) //ESC
				break;

			if (key=='s')
				save_patches ^= true;


			cv_mutex.unlock();
			usleep(10000);
		}

		return true;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv);

	OutletSpotting view;
	view.spin();

	return 0;
}

