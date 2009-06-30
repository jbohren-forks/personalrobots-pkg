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

// Author: Marius Muja, Gary Bradski

#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <cmath>

#include "opencv_latest/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cv.hpp"

#include "ros/node.h"
#include "sensor_msgs/StereoInfo.h"
#include "sensor_msgs/DisparityInfo.h"
#include "sensor_msgs/CamInfo.h"
#include "sensor_msgs/Image.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/Vector3.h"
#include "robot_msgs/PointStamped.h"
#include <robot_msgs/Polygon3D.h>
#include "door_msgs/Door.h"
//#include "robot_msgs/VisualizationMarker.h"

// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>


#include <recognition_lambertian/visualization.h>

#include <string>

#include <limits>

// transform library
#include <tf/transform_listener.h>

#include "topic_synchronizer/topic_synchronizer.h"

#include "CvStereoCamModel.h"

#include <boost/thread.hpp>

using namespace std;
using namespace robot_msgs;
static const double pi = 3.141592653589793238462643383279502884197; // Archimedes constant pi

typedef struct _triangle_offsets
{
    int  v0;
    int  v1;
    int  v2;
}
triangle_offsets;

typedef cv::Vec_<float, 16> Mat4x4f;

static inline cv::Vec4f transform4x4(const Mat4x4f& M, const cv::Vec4f& v)
{
    return cv::Vec4f(M[0]*v[0] + M[1]*v[1] + M[2]*v[2] + M[3]*v[3],
                     M[4]*v[0] + M[5]*v[1] + M[6]*v[2] + M[7]*v[3],
                     M[8]*v[0] + M[9]*v[1] + M[10]*v[2] + M[11]*v[3],
                     M[12]*v[0] + M[13]*v[1] + M[14]*v[2] + M[15]*v[3]);
}


////////////////////PLANE FINDING/////////////////////////////////////////////////////

// Slightly modifed plane finding class by RADU.
// !!!!NOTE, grid should just find planes from points in the grid, rather than stuffing them into point clouds to use these functions as done now.!!!!
class PlanarFit
{

  void getPointIndicesInZBounds (const PointCloud &points, double z_min, double z_max, vector<int> &indices);
  bool fitSACPlanes (PointCloud *points, vector<int> &indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff,
                     const robot_msgs::Point32 &viewpoint_cloud, double dist_thresh, int n_max, int min_points_per_model = 100);

  /**
      * Input
            o list of X,Y,Z,x,y points
            o Z min, Z max (only process points within min to max range
            o Support  where "Support" is a vertical distance above the plane where an object not on that plane will be considered to be supported by that plane  (OK, if this takes time, don't do it right now)
            o A where A is the minimal area a plane should have (again, don't bother with for now unless its already there)
            o N max number of planes to find in order of size
      * Output
            o list of indices in order of input list. Each indices is the tuple x,y,A where x,y is the pixel and A is "computed attribute"
                  + 0 is illegal range or othersize "no data"
                  + 1-N where this numbers the planes in terms of area. 1 is largest plane, 2 is next largest ...
                  + -1, -2, -3 means supported object by cluster (or for now for speed, all points that are not a 0 or not a plane are -1).


\NOTE: not sure if I understand the support parameter :(

   **/

public:

	/**
	 * \brief Segment planes from point clouds (this is the function I call in Radu's code for planes)
	 * @param points 			The 3D points
	 * @param indices_in_bounds	Index to the good points in "points" above
	 * @param num_pts			Number of 3D points belonging to the found required to accept that plane
	 * @param n_max				Maximum number of planes to look for
	 * @param indices			2D list of planes and indices to points belonging to that plane (if indices[i].size() = 0, plane was invalid)
	 * @param models			Equations of planes that were found.  Again, if models[i].size() = 0, then the plane wsa invalid
	 */
  void
     segmentPlanes (PointCloud &points, vector<int> &indices_in_bounds, int num_pts, int n_max,
                    vector<vector<int> > &indices, vector<vector<double> > &models)
   {
		// This should be given as a parameter as well, or set global, etc
		double sac_distance_threshold_ = 0.007; // 2cm distance threshold for inliers (point-to-plane distance)

		// We need to know the viewpoint where the data was acquired
		// For simplicity, assuming 0,0,0 for stereo data in the stereo frame - however if this is not true, use TF to get
		//the point in a different frame !
		Point32 viewpoint;
		viewpoint.x = viewpoint.y = viewpoint.z = 0;

		// Use the entire data to estimate the plane equation.
		// NOTE: if this is slow, we can downsample first, then fit (check mapping/point_cloud_mapping/src/planar_fit.cpp)
		//   vector<vector<int> > inliers;
		indices.clear(); //Points that are in plane
		models.clear(); //Plane equations
		//    vector<vector<double> > models;
//		printf("#indices_in_bounds = %d\n", (int) indices_in_bounds.size());
		fitSACPlanes(&points, indices_in_bounds, indices, models, viewpoint,
				sac_distance_threshold_, n_max);
//		printf("Num models found = %d\n", (int) models.size());
		// Check the list of planar areas found against the minimally imposed area
		for (unsigned int i = 0; i < models.size(); i++)
		{
			if ((int) indices[i].size() < num_pts){
//
//				// If the area is smaller, reset this planar model
//				if (area < min_area) {
					models[i].resize(0);
					indices[i].resize(0);
					continue;
				}
		}
	}

  void
    segmentPlanes (PointCloud &points, double z_min, double z_max, double min_area, int n_max,
                   vector<vector<int> > &indices, vector<vector<double> > &models)
  {
    // This should be given as a parameter as well, or set global, etc
    double sac_distance_threshold_ = 0.007;        // 2cm distance threshold for inliers (point-to-plane distance)

    vector<int> indices_in_bounds;
    // Get the point indices within z_min <-> z_max
    getPointIndicesInZBounds (points, z_min, z_max, indices_in_bounds);

    // We need to know the viewpoint where the data was acquired
    // For simplicity, assuming 0,0,0 for stereo data in the stereo frame - however if this is not true, use TF to get
    //the point in a different frame !
    Point32 viewpoint;
    viewpoint.x = viewpoint.y = viewpoint.z = 0;

    // Use the entire data to estimate the plane equation.
    // NOTE: if this is slow, we can downsample first, then fit (check mapping/point_cloud_mapping/src/planar_fit.cpp)
 //   vector<vector<int> > inliers;
    indices.clear(); //Points that are in plane
    models.clear();  //Plane equations
//    vector<vector<double> > models;
    fitSACPlanes (&points, indices_in_bounds, indices, models, viewpoint, sac_distance_threshold_, n_max);

    // Check the list of planar areas found against the minimally imposed area
    for (unsigned int i = 0; i < models.size (); i++)
    {
      // Compute the convex hull of the area
      // NOTE: this is faster than computing the concave (alpha) hull, so let's see how this works out
      Polygon3D polygon;
      cloud_geometry::areas::convexHull2D (points, indices[i], models[i], polygon);

      // Compute the area of the polygon
      double area = cloud_geometry::areas::compute2DPolygonalArea (polygon, models[i]);

      // If the area is smaller, reset this planar model
      if (area < min_area)
      {
        models[i].resize (0);
        indices[i].resize (0);
        continue;
      }
    }

//    // Copy all the planar models inliers to indices
//    for (unsigned int i = 0; i < inliers.size (); i++)
//    {
//      if (inliers[i].size () == 0) continue;
//
//      int old_indices_size = indices.size ();
//      indices.resize (old_indices_size + inliers[i].size ());
//      for (unsigned int j = 0; j < inliers[i].size (); j++)
//        indices[old_indices_size + j] = inliers[i][j];
//    }
  }

  protected:


  public:

    // ROS messages
//    PointCloud cloud_, cloud_plane_, cloud_outliers_;
//
//    double z_min_, z_max_, support_, min_area_;
//    int n_max_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*  PlanarFit (ros::Node& anode) : node_ (anode)
    {
      node_.param ("~z_min", z_min_, 0.5);
      node_.param ("~z_max", z_max_, 1.5);
      node_.param ("~support", support_, 0.1);
      node_.param ("~min_area", min_area_, 0.2);
      node_.param ("~n_max", n_max_, 1);

      string cloud_topic ("/stereo/cloud");

      vector<pair<string, string> > t_list;
      node_.getPublishedTopics (&t_list);
      bool topic_found = false;
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (node_.mapName (cloud_topic)) != string::npos)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", node_.mapName (cloud_topic).c_str ());

      node_.subscribe (cloud_topic, cloud_, &PlanarFit::cloud_cb, this, 1);
      node_.advertise<PointCloud> ("~plane", 1);
      node_.advertise<PointCloud> ("~outliers", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_.pts.size (), cloud_.header.frame_id.c_str (),
                (int)cloud_.chan.size (), cloud_geometry::getAvailableChannels (cloud_).c_str ());
      if (cloud_.pts.size () == 0)
      {
        ROS_ERROR ("No data points found. Exiting...");
        return;
      }

      ros::Time ts = ros::Time::now ();

      vector<vector<int> > indices;
      vector<vector<double> > models;
      segmentPlanes (cloud_, z_min_, z_max_, support_, min_area_, n_max_, indices, models);

      if((int)indices.size() > 0){
      cloud_geometry::getPointCloud (cloud_, indices[0], cloud_plane_);
      cloud_geometry::getPointCloudOutside (cloud_, indices[0], cloud_outliers_);
      ROS_INFO ("Planar model found with %d / %d inliers in %g seconds.\n", (int)indices[0].size (), (int)cloud_.pts.size (), (ros::Time::now () - ts).toSec ());
      }

      node_.publish ("~plane", cloud_plane_);
      node_.publish ("~outliers", cloud_outliers_);
    }
    */
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Obtain a subset of the point cloud indices between two given Z (min, max) values
  * \param points the point cloud message
  * \param z_min the minimum Z value
  * \param z_max the maximum Z value
  */
void
  PlanarFit::getPointIndicesInZBounds (const PointCloud &points, double z_min, double z_max, vector<int> &indices)
{
  indices.resize (points.pts.size ());
  int nr_p = 0;
  for (unsigned int i = 0; i < points.pts.size (); i++)
  {
    if ((points.pts[i].z >= z_min && points.pts[i].z <= z_max))
    {
      indices[nr_p] = i;
      nr_p++;
    }
  }
  indices.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Find a list of plane models in a point cloud with SAmple Consensus methods
  * \param points the point cloud message
  * \param indices a subset of point indices to use
  * \param inliers the resultant planar model inliers
  * \param coeff the resultant plane model coefficients
  * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
  * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
  * \param n_max maximum number of planar models to search for
  * \param min_points_per_model the minimum number of points allowed for a planar model (default: 100)
  */
bool
  PlanarFit::fitSACPlanes (PointCloud *points, vector<int> &indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff,
                           const robot_msgs::Point32 &viewpoint_cloud, double dist_thresh, int n_max, int min_points_per_model)
{
  // Create and initialize the SAC model
  sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
  sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
  sac->setMaxIterations (100);
  model->setDataSet (points, indices);

  int nr_models = 0, nr_points_left = indices.size ();
  while (nr_models < n_max && nr_points_left > min_points_per_model)
  {
    // Search for the best plane
    if (sac->computeModel ())
    {
      vector<double> model_coeff;
      sac->computeCoefficients (model_coeff);                              // Compute the model coefficients
      sac->refineCoefficients (model_coeff);                               // Refine them using least-squares
      coeff.push_back (model_coeff);

      // Get the list of inliers
      vector<int> model_inliers;
      model->selectWithinDistance (model_coeff, dist_thresh, model_inliers);
      inliers.push_back (model_inliers);

      // Flip the plane normal towards the viewpoint
      cloud_geometry::angles::flipNormalTowardsViewpoint (model_coeff, points->pts.at (model_inliers[0]), viewpoint_cloud);

      ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)model_inliers.size (),
                model_coeff[0], model_coeff[1], model_coeff[2], model_coeff[3]);

      // Remove the current inliers from the global list of points
      nr_points_left = sac->removeInliers ();
      nr_models++;
    }
    else
    {
      ROS_ERROR ("Could not compute a planar model for %d points.", nr_points_left);
      break;
    }
  }

  delete sac;
  delete model;
  return (true);
}





////////////////////STEREO POINT CLOUD FINDING////////////////////////////////////////
class StereoPointCloudProcessing{

public:

	CvRNG rng_state;
	StereoPointCloudProcessing()
	{
		rng_state = cvRNG();
	}

	//na -- for testing
	void makePlaneData(CvMat *xyzd, float A, float B, float C, float Xa, float Ya, float Za)
	{
		float X,Y,Z;
		if(!xyzd) return;
		if(C == 0.0) return;
		int rows = xyzd->rows;
		int cols = xyzd->cols;
		for(int y=0; y<rows; ++y){
			for(int x= 0; x<cols; ++x){
				X = (float)x;
				Y = (float)y;
				Z = Za - ((A*(X-Xa)+B*(Y-Ya))/C); //Fill in depth that satisfies the equation of the plane
 				cvSet2D( xyzd, y, x, cvScalar(X,Y,Z,1.0) ); //Stick it into the array
			}
		}
	}


//	void rotateHomogeniousPointsInROI_1(const CvMat *xyzd,
//	                                    CvMat *_rot_trans, CvRect roi,
//	                                    CvMat* output)
//	{
//	    CV_Assert(roi.x >= 0 && roi.width >= 0 && roi.x + roi.width <= xyzd->cols &&
//	        roi.y >= 0 && roi.height >= 0 && roi.y + roi.height <= xyzd->rows);
//	    CV_Assert(output->cols == roi.width && output->rows == roi.height &&
//	        CV_ARE_TYPES_EQ(output, xyzd) && CV_MAT_TYPE(output->type) == CV_32FC4);
//	    CV_Assert(_rot_trans->cols == CV_MAT_CN(output->type) &&
//	        _rot_trans->rows == _rot_trans->cols &&
//	        (CV_MAT_TYPE(_rot_trans->type) == CV_32F ||
//	        CV_MAT_TYPE(_rot_trans->type) == CV_64F));
//	    Mat4x4f M;
//	    CvMat _M = cvMat(4, 4, CV_32F, &M[0]);
//	    cvConvert(_rot_trans, &_M);
//
//	    for(int i = 0; i < roi.height; i++)
//	    {
//	        const cv::Vec4f* src = (const cv::Vec4f*)(xyzd->data.ptr + xyzd->step*(roi.y + i)) + roi.x;
//	        cv::Vec4f* dst = (cv::Vec4f*)(output->data.ptr + output->step*i);
//	        for(int j = 0; j < roi.width; j++)
//	            dst[j] = transform4x4(M, src[j]);
//	    }
//	}

	/**
	 * \brief Using the XYZA structure rotate points using _rot_trans within roi and return in output
	 * @param xyzd          2D, 4 Channel float X,Y,Z,A grid of 3D points
	 * @param _rot_trans    4x4 float rotation and translation matrix
	 * @param roi           rectangular region of interest
	 * @param output        2D, 4 Channel float X,Y,Z,A grid, size of roi rotated, translated output
	 */
	void rotateHomogeniousPointsInROI(const CvMat *xyzd,
	                                    CvMat *_rot_trans, CvRect roi,
	                                    CvMat* output)
	{
	    CV_Assert(roi.x >= 0 && roi.width >= 0 && roi.x + roi.width <= xyzd->cols &&
	        roi.y >= 0 && roi.height >= 0 && roi.y + roi.height <= xyzd->rows);
	    CV_Assert(output->cols == roi.width && output->rows == roi.height &&
	        CV_ARE_TYPES_EQ(output, xyzd) && CV_MAT_DEPTH(output->type) == CV_32F);
	    CV_Assert(_rot_trans->cols == CV_MAT_CN(output->type) && _rot_trans->rows == _rot_trans->cols);
	    CvMat xyzd_roi;
	    cvGetSubRect(xyzd, &xyzd_roi, roi);

	    cvTransform(&xyzd_roi, output, _rot_trans );
	}



	/**
	 * \breif Print out a (hopefully small) float matrix with with lable string C
	 * @param M   2D float matrix with 1,2,3 or 4 channels
	 * @param C   char string label to print first
	 */
	void prnMatF(CvMat *M, char * C)
	{
		int d = M->cols;
		int chans = 1;
		int type = M->type;
		if(type == CV_32FC2) chans = 2;
		if(type == CV_32FC3) chans = 3;
		if(type == CV_32FC4) chans = 4;
		printf("\n%s:\n",C);
		float *f = M->data.fl;
		for(int r = 0; r<d; ++r){
			for(int c = 0; c<d; ++c){
				printf("[");
				for(int ch = 0; ch<chans; ++ch){
					printf("%f, ",*f++);
				}
				printf("], ");
			}
			printf("\n");
		}
	}

	/**
	 * \brief This function will be in OpenCV, returns rotation of norman n to Z axis
	 * @param n  Normal vector in A,B,C form
	 * @return   Rotation Mat
	 */
	cv::Mat rotationFromNormal(cv::Vec3f n)
	{
	    cv::Mat _out(3,3,CV_32F);
	    float* out = (float*)_out.data;
	    double pitch = atan2(n[0], n[2]);
	    double pmat[] = { cos(pitch), 0, -sin(pitch) ,
	                        0      , 1,      0      ,
	                     sin(pitch), 0,  cos(pitch) };

	    double roll = atan2((double)n[1], n[0] * pmat[3*2+0] + n[2] * pmat[3*2+2]);

	    double rmat[] = { 1,     0,         0,
	                     0, cos(roll), -sin(roll) ,
	                     0, sin(roll),  cos(roll) };

	    for(int i = 0; i < 3; ++i)
	        for(int j = 0; j < 3; ++j)
	            out[3*i+j] = (float)(rmat[3*i+0]*pmat[3*0+j] +
	                rmat[3*i+1]*pmat[3*1+j] + rmat[3*i+2]*pmat[3*2+j]);
	    return _out;
	}

//	CvMat foo;
//	Mat bar = &foo;
//
//	of course, no & needed if foo is already a pointer
//	CvMat* foo = cvCreateMat(3, 3, CV_32F);
//	Mat bar = foo;
//
//	Mat bar;
//	. . . data ...
//	CvMat foo = bar;

	/**
	 * \brief Compute RT matrix from a normal n and point pt3d to the Z axis and camera origin
	 * @param pt3d  The 3D point location of type CvPoint3D32f
	 * @param n     The normal at that point of type CvPoint3D32f
	 * @param RT    4x4 float matrix pointer will be filled with Rotation and Translation
	 * @param R     4x4 float matrix pointer will be filled with just the rotation matrix
	 * @return      0=OK, -3=>RT NULL or of wrong format, -4=>R NULL or of wrong format
	 */
	int computeRTtoCameraFromNormal(CvPoint3D32f pt3d, CvPoint3D32f n, CvMat *RT, CvMat *R )
	{
		//CHECKS
		if((RT->rows) != 4 || (RT->cols) != 4) return -3;
		int mtype =  cvGetElemType(RT);
		if(mtype != CV_32FC1) return -3;
		cvSetZero(RT);
		if((R->rows) != 4 || (R->cols) != 4) return -4;
		mtype =  cvGetElemType(R);
		if(mtype != CV_32FC1) return -4;
		cvSetZero(R);
		cvSet2D(R, 3, 3, cvScalar(1.0));

		//SET TRANSLATION PART (cvSet uses the order (y,x)
		//debug
//		pt3d.x = 1.0;pt3d.y = 2.0; pt3d.z = 3.0;
//		n.x = 1.0; n.y = 1.732050808; n.z = 1.0;
//		n.x = 1.0; n.y = 1.0; n.z = 1.0;
		cvSet2D(RT, 0, 3, cvScalar(-pt3d.x));
		cvSet2D(RT, 1, 3, cvScalar(-pt3d.y));
		cvSet2D(RT, 2, 3, cvScalar(-pt3d.z));
		cvSet2D(RT, 3, 3, cvScalar(1.0));

		cv::Vec3f N;
		N[0] = n.x;
		N[1] = n.y;
		N[2] = n.z;

		//COMPUTE NORMAL ROTATION TO ALIGN WITH Z AXIS AND COPY IT OUT
		CvMat tR = rotationFromNormal(N);
//		prnMat3x3(&tR, "R matrix = ");
		for(int i = 0; i<3; ++i)
			for(int j = 0; j<3; ++j){
				cvSet2D(R,i,j,cvGet2D(&tR,i,j));
				cvSet2D(RT,i,j,cvGet2D(&tR,i,j));
			}
		prnMatF(RT,"RT = R+T:");

//		//debug -- test this
//	    CvMat* points = cvCreateMat(4, 4, CV_32FC4);
//	    CvMat* norms =  cvCreateMat(4, 4, CV_32FC4);
//	    CvMat* points_out = cvCreateMat(4, 4, CV_32FC4);
//	    CvMat* norms_out = cvCreateMat(4, 4, CV_32FC4);
////		n.x = -1.0; n.y = -1.732050808; n.z = -1.0;
////		n.x = 1.0; n.y = 1.0; n.z = 1.0;
//	    float *pp = points->data.fl;
//	    float *pn = norms->data.fl;
//	    for(int i = 0; i< 16; ++i, pp+=4, pn+=4) {
//	    	*pp =  pt3d.x;
//	    	*(pp+1) = pt3d.y;
//	    	*(pp+2) = pt3d.z;
//	    	*(pp+3) = 1.0;
//	    	*pn =  n.x;
//	    	*(pn+1) = n.y;
//	    	*(pn+2) = n.z;
//	    	*(pn+3) = 1.0;
//	    }
//	    cvSet2D(points, 0, 0, cvScalar(1.0, 0.0, 0.0, 0.0)); //first 3 points are test
//	    cvSet2D(points, 1, 0, cvScalar(0.0, 1.0, 0.0, 0.0));
//	    cvSet2D(points, 2, 0, cvScalar(0.0, 0.0, 1.0, 0.0));
//	    cvSet2D(points, 3, 0, cvScalar(0.0, 0.0, 0.0, 1.0));
//	    cvSet2D(norms, 0, 0, cvScalar(1.732050808, 0.0, 0.0, 0.0));
//	    cvSet2D(norms, 1, 0, cvScalar(0.0, 1.0, 0.0, 0.0));
//	    cvSet2D(norms, 2, 0, cvScalar(0.0, 0.0, 1.0, 0.0));
//	    cvSet2D(norms, 3, 0, cvScalar(0.0, 0.0, 0.0, 1.0));
//	    prnMat(norms,"norms in",4);
//	    rotateHomogeniousPointsInROI_2(norms, R, cvRect(0,0,4,4), norms_out);
//	    prnMat(norms_out,"Normals rotateed by R",4);
//
//
//	    rotateHomogeniousPointsInROI_2(points, RT, cvRect(0,0,4,4), points_out);
//	    prnMat(points_out,"Points by RT",4);
//
//
////	    printf("\n\nRotation matrix test:\n");
////
////	    pp = points->data.fl;
////	    pn = norms->data.fl;
////	    float *ppo = points_out->data.fl;
////	    float *pno = norms_out->data.fl;
////
////	    printf("point: (%f, %f, %f) => (%f, %f, %f)\n",*pp, *(pp+1), *(pp+2), *ppo, *(ppo+1), *(ppo+2));
////	    printf("norms: (%f, %f, %f) => (%f, %f, %f)\n",*pn, *(pn+1), *(pn+2), *pno, *(pno+1), *(pno+2));
////		printf("\n");
//
//		cvReleaseMat(&points);
//		cvReleaseMat(&norms);
//		cvReleaseMat(&points_out);
//		cvReleaseMat(&norms_out);

//		printf("x=1.7, y= 1, z=1.7. Theta_11 should be 30");
//		printf("RADIANS: ul,ur/ll,lr: [%f, %f]/[%f, %f]\n",Theta_xm1_1,Theta_x11,-Theta_xm1_1,-Theta_x11);
//		printf("DEGREES: ul,ur/ll,lr: [%f, %f]/[%f, %f]\n",Theta_xm1_1*todeg,Theta_x11*todeg,-Theta_xm1_1*todeg,-Theta_x11*todeg);
//		printf("cos(ul,ur/ll,lr): [%f, %f]/[%f, %f]\n",(float)cos(Theta_xm1_1),(float)cos(Theta_x11),(float)cos(-Theta_xm1_1),(float)cos(-Theta_x11));
//		printf("neg angle cos(ul,ur/ll,lr): [%f, %f]/[%f, %f]\n",(float)cos(-Theta_xm1_1),(float)cos(-Theta_x11),(float)cos(Theta_xm1_1),(float)cos(Theta_x11));

		return 0;
	}

	  //!!!! This function SHOULD BE REPLACED with a native plane fit from a grid, for now, use Radu           !!!!
	  //!!!! Note that I've already done the work of finding normals, so collecting them into planes is simple !!!!
	  /**
	   * \brief From an XYZA depth grid, mark points that are part of a plannar surface
	   * @param xyza        X,Y,Z,A matrix.  Found plane points marked in order of size -1, -2 ...; -10K=>point not with in z_min or z_max; 0=>invalid point; >0 => outlier (object) points
	   * @param z_min		minimum depth from camera to consider
	   * @param z_max		maximum depth from camera to consider
	   * @param num_pts		how many points must be contained in the planar surface, to be considered as a found plane
	   * @param n_max		maximum number of planes to find
	   * @param models		RETURN equations of found planes (any index model[i].size() that is zero indicates that plane failed due to small num_pts)
	   * @return The number of planes found.  Negative numbers indicate some kind of error
	   */
	  int segmentPlanesFromXYZAGrid(CvMat *xyza, float z_min, float z_max, int num_pts, int n_max, vector<vector<double> > &models)
	  {
		  PlanarFit Pfit; //Just to call Pfit.segmentPlanes()
		  int planes_found = 0;

		  //STUFF POINT CLOUD
		  PointCloud pc;
		  robot_msgs::Point32 pt3d;
		  int rows = xyza->rows;
		  int cols = xyza->cols;
		  float *fptr = xyza->data.fl;
		  vector<int> indices_in_bounds;
		  vector<int> offsets;
		  int ptcnt = 0;
		  int zoutcnt = 0;
		  for(int y = 0; y<rows; ++y)
		  {
			  for(int x = 0; x<cols; ++x, fptr += 4)
			  {
				  if(*(fptr+3) < 0.0001) {  continue;}
//				  else { if(!(x%10)){printf("X");}}
				  if((*(fptr+2) < z_min)||(*(fptr+2) > z_max))
				  {
					  zoutcnt++;
//					  if(!(zoutcnt/16))  printf("(%d,%d): zout(%d) = %f || ",x,y,zoutcnt,*(fptr+1));
					  *(fptr+3) = -10000.0; //Indicates out of range
					  continue;
				  }
				  pt3d.x = *fptr;
				  pt3d.y = *(fptr+1);
				  pt3d.z = *(fptr+2);
				  pc.pts.push_back(pt3d);
				  indices_in_bounds.push_back(ptcnt++);
				  offsets.push_back((int)(fptr - xyza->data.fl));
			  }
//			  printf("\n");
		  }
//		  printf("\n");//Points pushed back = %d = %d = %d\n",(int)pc.pts.size(),(int)indices_in_bounds.size(),offsets.size());

		  //FIND THE PLANES
		  vector<vector<int> > indices;
		  Pfit.segmentPlanes(pc, indices_in_bounds, num_pts, n_max, indices, models);
//		  printf("Total number of indices = %d\n",(int)indices.size());

		  //MARK THE PLANES ON XYZA
		  fptr = xyza->data.fl;
		 for (unsigned int i = 0; i < models.size (); i++)
		 {
			 int isize = (int)(indices[i].size());
//			 printf("i = %d, indices.size = %d\n",i,isize);
			 if(isize == 0) continue;
			 planes_found++;
			 for(int j = 0; j<isize; ++j)
			 {
				 int indx = indices[i][j];
				 int oset = offsets[indx];
//				 printf("(i%do%d:%d,%d)",i,j,indx,oset);
//				 if(*(fptr+oset+3) < 0.0001) printf("[%d][%d]y:%d,x:%d",i,j,(int)(oset/(4.0*cols)),(int)(oset - 4*cols*((int)(oset/(4.0*cols))))/4);
				 *(fptr+oset+3) =  -(float)(i+1);
			 }
		 }
		 return planes_found;
	  }


	  /**
	   * \brief This just markes points within grid_radius (in the 2D grid) that are within a 3D radius_sqr of the center point p. A list of good offsets is returned
	   * @param xyzd        	X,Y,Z,D matrix, also known as xyza where "A" is anything, mainly to zero or allow using that 3D point
	   * @param p				The 2D center point on the 2D grid
	   * @param grid_radius		The radius on the grid to check (actually, it checks all points in the box around "p"
	   * @param radius_sqr		Mark all 3D points within the grid_radius that are within radius_sqr of the center point
	   * @param offsets			This returns the offsets to the valid (floating point) points relative to the data start xyzd->data.fl
	   * @return Ratio of points that are valid within the grid_radius blocks.  NEGATIVE return, -1.0 => "p" wasn't valid
	   */
	  float markPointsInRadius(const CvMat *xyzd, const CvPoint &p, int grid_radius, float radius_sqr, vector<int> offsets)
	  {
		  if(!xyzd) return 0.0;
		  int rows = xyzd->rows;
		  int cols = xyzd->cols;
		  float *fptr = xyzd->data.fl;
		  //find how big our square can really be:
		  int r = p.y;
		  int c = p.x;
		  if(*(fptr + r*cols*4 + c*4) < 0.001) return -1.0;  //center point "p" was not a valid point
		  float pX = *fptr;
		  float pY = *(fptr+1);
		  float pZ = *(fptr+2);
		  int rstart = r - grid_radius;
		  if(rstart < 0) rstart = 0;
		  int cstart = c - grid_radius;
		  if(cstart < 0) cstart = 0;
		  int rend = r + grid_radius;
		  if(rend >= rows) rend = rows - 1;
		  int cend = c + grid_radius;
		  if(cend >= cols) cend = cols - 1;
		  //LOOP THROUGH, LOOKING FOR GOOD POINTS
		  int oset;
		  float X,Y,Z;
		  for(int R=rstart; R<rend; ++R){
			  oset = R*cols*4 + cstart*4;
			  for(int C=cstart; C<cend; ++C, oset += 4){
				  if(*(fptr+oset+3) < 0.001) continue;
				  X = *(fptr + oset)-pX;
				  Y = *(fptr + oset + 1)-pY;
				  Z = *(fptr + oset + 2)-pZ;
				  float r2 = X*X+Y*Y+Z*Z;
				  if(r2 > radius_sqr) continue;
				  offsets.push_back(oset);
			  }
		  }
		  float found = (float)(offsets.size());
		  float total = (rend - rstart)*(cend - cstart) + 0.0000001;
		  return found/total;
	  }


	//!!!! IN THE ROUTINE BELOW, CHOOSING THE MEDIAN NORMAL SHOULD BE CHANGED TO THE AVERAGE NORMAL FOR SSE EFFICIENCY ... SO SORRY !!!!
	/**
	 * \brief  Robust normal computation at point p in the X,Y,Z,D matrix
	 * @param xyzd                  X,Y,Z,D matrix, also known as xyza where "A" is anything, mainly to zero or allow using that 3D point
	 * @param p                     point (x,y) in xyzd that we're computing robust normal at
	 * @param v						precomputed list of triangle offsets to use at point p
	 * @param radius_sqr   			ignore X,Y,Z points with X^2+Y^2+Z^2 greater than this distance
	 * @param normal_pt3d           RETURN X,Y,Z location of the normal at p
	 * @param normals		        RETURN normals in A,B,C form
	 * @param normal_angles			RETURN normals in angular (0-360 degree) form: (rot around Z, rotation from X,Y plane)
	 * @param quality				RETURN 1.0 = perfect quality: found_triangles/num_triangles(that is, v.size())
	 * @return                      Index of median normal, OR: -1,median computation failure, -2,invalid dimensions, -3,no trainges, -4,no center point
	 */
	int computeNormalAtPt(const CvMat *xyzd, const CvPoint &p, const vector<triangle_offsets> &v, float radius_sqr,
			vector<float> &normal_pt3d, vector<vector<float> > &normals, vector<vector<float> > &normal_angles,
			float &quality)
	{
		//SET UP AND CHECKS
		if(!xyzd) return -5; //No matrix
		int col_offset = p.x;
		int row_offset = p.y;
		int rows = xyzd->rows;
		int cols = xyzd->cols;
		if((col_offset >= cols)||(row_offset >= rows)||(col_offset < 0)||(row_offset < 0)) return -2; //Invalid dimensions
		float *dat_start = xyzd->data.fl;         //
		float *dat_end = dat_start + rows*cols*4; //End of data range
		float *dat = dat_start + cols*row_offset*4 + col_offset*4; //Point to p (x,y) offset into xyzd
		int num_triangles = (int)v.size();
		if(num_triangles <= 0) return -3; //No triangles
		normal_pt3d.clear();
		normals.clear();
		normal_angles.clear();
		//COMPUTE THE TRIANGLES
		float Xa,Xb,Xc,Ya,Yb,Yc,Za,Zb,Zc,Da,Db,Dc; //Values we will need
		float Xpt = *dat,Ypt = *(dat+1),Zpt = *(dat+2); //Record the base position of the normal world coordinates
		if(*(dat+3) < 0.0000001) return -4; //No center point
		float Xd,Yd,Zd; //Distances from center point (Xpt,Ypt,Zpt)
		normal_pt3d.push_back(Xpt); normal_pt3d.push_back(Ypt); normal_pt3d.push_back(Zpt); //Enter normal point location
		for(int i = 0; i<num_triangles; ++i)
		{
//			printf("triangle #%d, ",i);
			//VERTEX 0
			if((dat_start > dat+v[i].v0) || (dat+v[i].v0 + 3)>=dat_end) continue; //Don't allow stepping out of xyzd
			Xa = *(dat + v[i].v0);
			Ya = *(dat + v[i].v0 + 1);
			Za = *(dat + v[i].v0 + 2);
			Xd = Xa-Xpt; //if(Xd < 0) Xd = -Xd;
			Yd = Ya-Ypt; //if(Yd < 0) Yd = -Yd;
			Zd = Za-Zpt; //if(Zd < 0) Zd = -Zd;
//			printf("V0,Offset=%d: Xa,Ya,Za=(%f, %f, %f) Xd,Yd,Zd=(%f, %f, %f), ",v[i].v0,Xa,Ya,Za,Xd,Yd,Zd);
			if(Xd*Xd+Yd*Yd+Zd*Zd > radius_sqr) continue;
			Da = *(dat + v[i].v0 + 3);
//			printf("Da=%f\n",Da);
			if(Da < 0.0000001) continue; //Invalid point
			//VERTEX 1
			if((dat_start > dat+v[i].v1) || (dat+v[i].v1 + 3)>=dat_end) continue; //Don't allow stepping out of xyzd
			Xb = *(dat + v[i].v1);
			Yb = *(dat + v[i].v1 + 1);
			Zb = *(dat + v[i].v1 + 2);
			Xd = Xb-Xpt; //if(Xd < 0) Xd = -Xd;
			Yd = Yb-Ypt; //if(Yd < 0) Yd = -Yd;
			Zd = Zb-Zpt; //if(Zd < 0) Zd = -Zd;
//			printf("V1,Offset=%d: Xa,Ya,Za=(%f, %f, %f) Xd,Yd,Zd=(%f, %f, %f), ",v[i].v1,Xa,Ya,Za,Xd,Yd,Zd);
			if(Xd*Xd+Yd*Yd+Zd*Zd > radius_sqr) continue;
			Db = *(dat + v[i].v1 + 3);
//			printf("Da=%f\n",Da);
			if(Db < 0.0000001) continue; //Invalid point
			//VERTEX 2
			if((dat_start > dat+v[i].v2) || (dat+v[i].v2 + 3)>=dat_end) continue; //Don't allow stepping out of xyzd
			Xc = *(dat + v[i].v2);
			Yc = *(dat + v[i].v2 + 1);
			Zc = *(dat + v[i].v2 + 2);
			Xd = Xc-Xpt; //if(Xd < 0) Xd = -Xd;
			Yd = Yc-Ypt; //if(Yd < 0) Yd = -Yd;
			Zd = Zc-Zpt; //if(Zd < 0) Zd = -Zd;
//			printf("V3,Offset=%d: Xa,Ya,Za=(%f, %f, %f) Xd,Yd,Zd=(%f, %f, %f), ",v[i].v2,Xa,Ya,Za,Xd,Yd,Zd);
			if(Xd*Xd+Yd*Yd+Zd*Zd > radius_sqr) continue;
			Dc = *(dat + v[i].v2 + 3);
//			printf("Da=%f\n",Da);
			if(Dc < 0.0000001) continue; //Invalid point
			//COMPUTE THE NORMAL
			vector<float> n;
			float A = (((Yb-Ya)*(Zc-Za)) - ((Yc-Ya)*(Zb-Za))); //A (x coef)
			float B = (((Xc-Xa)*(Zb-Za)) - ((Xb-Xa)*(Zc-Za))); //B (y coef)
			float C = (((Xb-Xa)*(Yc-Ya)) - ((Xc-Xa)*(Yb-Ya))); //C (z coef)
			float ABC = sqrt(A*A+B*B+C*C); //Normalize the normal
			if(ABC == 0.0){ n.push_back(0.0);n.push_back(0.0);n.push_back(0.0);}
			else {n.push_back(A/ABC); n.push_back(B/ABC); n.push_back(C/ABC);  }
//			printf("A=%f, B=%f, C=%f\n",n[0],n[1],n[2]);
			normals.push_back(n);
			//COMPUTE THE ANGLE FORM OF THE NORMAL IF ASKED
			vector<float> a;
			a.push_back(cvFastArctan(B,A));       // Angles 0-360 for Y/X  -- rotation around the z axis
			float rxy = cvSqrt(A*A + B*B);        // sqrt of X^2 + Y^2
			a.push_back(cvFastArctan( rxy, C ));  // Angle of from X,Y plane arctan(sqrt(X^2+Y^2)/Z)
//			printf("Angles: (%f, %f)\n",a[0],a[1]);
			normal_angles.push_back(a);
		}
		int median_index = -1;
		vector<float> median = findMedianND(normal_angles,median_index);
		quality = (float)(normals.size())/(float)num_triangles;
		return median_index;
	}




    int computeNormals(const CvMat *xyza, CvMat *xyzn, CvRect roi, vector<triangle_offsets> &Trioffs,
    		float radius_sqr, float quality, int skip = 1)
    {
    	//CHECK STUFF
    	if(!xyza) return -1;
        if(CV_32FC4 != cvGetElemType(xyza)) return -1;
    	if(!xyzn) return -2;
        if(CV_32FC4 != cvGetElemType(xyzn)) return -2;
    	int rows = xyza->rows;
    	int cols = xyza->cols;
    	if(rows <= 0 || rows != xyzn->rows || cols <= 0 || cols != xyzn->cols) return -2;
        int rx = roi.x;
        if(rx < 0) rx = 0;
        if(rx >= cols) rx = cols - 1;
        int rw = rx + roi.width;
        if(rw < rx) rw = rx;
        int ry = roi.y;
        if(ry < 0) ry = 0;
        if(ry >= rows) ry = rows - 1;
        int rh = ry + roi.height;
        if(rh < ry) rh = ry;
        if(rw > cols) rw = cols;
        if(rh > rows) rh = rows;
        if((int)Trioffs.size() == 0) return -4;
        if(skip <= 0) skip = 1;
        printf("rx,w,ry,h=%d,%d, %d,%d\n",rx,rw,ry,rh);

        //SET UP TO COMPUTE NORMALS
        cvSetZero(xyzn);
		vector<float> normal_pt3d;
		vector<vector<float> > normals;
		vector<vector<float> > normal_angles;
		float *fptr,*nfptr;
		int x,y,nindex,jumpby = skip*4;
		float quality_ret;
		int ret = 0;

		//COMPUTE GRID OF NORMALS
		for(y = ry; y<rh; y += skip)
		{
			nfptr = xyzn->data.fl + y*4*cols + rx*4; //Start out pointing to the correct position
			fptr = xyza->data.fl + y*4*cols + rx*4; //Start out pointing to the correct position
			for(x = rx; x<rw; x += skip, nfptr += jumpby, fptr += jumpby)
			{
					nindex = computeNormalAtPt(xyza, cvPoint(x,y), Trioffs, radius_sqr,
							normal_pt3d, normals, normal_angles, quality_ret);
					if (nindex>=0 && quality_ret >= quality) {
						*nfptr     = normals[nindex][0]; //A
						*(nfptr+1) = normals[nindex][1]; //B
						*(nfptr+2) = normals[nindex][2]; //C
						*(nfptr+3) = (float)nindex;
						++ret;
					}
			}
		}
		return ret;
    }

	/**
	 * \brief Finds the median of a set of multi-dimensional points v.
	 * @param v             The set of M points, each of dimension N
	 * @param median_index  RETURN: index of the median point
	 * @return              The median point itself
	 */
	vector<float> findMedianND(vector<vector<float> > &v, int &median_index)
	{
		vector<float> median;
		//FIND THE INITIAL CENTER OF MASS AND AVG RADIUS/WINDOW SIZE TO WORK WITH
		median = centerOfMassND(v); //Find the initial center of mass
		int med_dim; //Number of dimensions
		if((med_dim = (int)median.size()) == 0) return median; // no size means error
		int M = (int)v.size();
		if(M < 1) {median.clear(); return median;} //No points to calculate
		vector<bool> b(M,false);
		float std_radius = 0.0;
		vector<float> p(med_dim, 0.0);
		float avg_radius = 	avgCityDistanceND(v, median, std_radius);
//		printf("avg_radius = %f, std_radius=%f\n",avg_radius,std_radius);
		if(avg_radius > 99999999.0) return median;  //Oh well, center of mass is best we can do ...
		avg_radius += std_radius;
		//FIND THE MEDIAN BY, BASICALLY, MEANSHIFT ... MODIFIED BY JUMPING TO THE NEAREST POINT EACH LOOP
		float dist;
		for(int i = 0; i<7; ++i) //Stop when distance changed is low or after 7 interations
		{

			markClosePoints(v, median, avg_radius, b);
//			printf("findMedND(i=%d) b size = %d\n",i,(int)b.size());
			p = centerOfMassND(v, &b); //This is the mean shift
			dist = 0.0;
			for(int j = 0; j<med_dim; ++j)
			{
				float d = median[j] - p[j];
				if(d<0.0) d = -d;
				dist += d;
			}
			median = p;
			if(dist <= 2.0) break; //If shift is less than 2 degrees in the case of 2D angles, we stop
		}
		//SELECT ACTUAL POINT TO RETURN
//		printf("findMedianND: find_closest point\n");
		median = findClosestPointToPinV(v, median, b, median_index);
		return median;
	}

	/**
	 * \brief Find all points in v within distance of p and mark such in b
	 * @param v         List of points
	 * @param p         Point to determine distance from
	 * @param max_dist  Maximum (city block) distance |v[i] - p| allowed
	 * @param b         RETURN: boolean mark points valid
	 */
	void markClosePoints(const vector<vector<float> > &v, const vector<float> &p, const float max_dist, vector<bool> &b)
	{
//		printf("markClosePoints: p(%f, %f)\n",p[0],p[1]);
		b.clear();
		int M = (int)v.size();
		if(M < 1) return;
		int N = (int)p.size();
		if(N < 1) return;
		b.resize(M,false);
		float dist;
		for(int i=0; i<M; ++i)
		{
//			printf("i=%d, ",i);
			dist = 0.0;
			for(int j=0; j<N; ++j)
			{
				float d = v[i][j] - p[j];
				if(d<0.0) d = -d;
				dist += d; //City block distance
			}
//			printf("dist=%f, <= max_dist=%f\n",dist,max_dist);
			if(dist <= max_dist)
				b[i] = true;
		}
//		printf("b = ");
//		for(int f= 0; f<(int)b.size(); ++f){
//			printf("(%d)[%d] ",f,(int)b[f]);
//		}
//		printf("\n");
		return;
	}

	/**
	 * \brief This function returns the "N" dimensional center of mass of a list of ND points
	 * @param v   vector<vector<float> > M points each of dimension N (each sub-vector must be this length)
	 * @param b   pointer to boolean vector that marks which points are valid.  Default is NULL
	 * @return    returns the ND center of mass point.  Error if this returned vector size is 0
	 */
	vector<float>  centerOfMassND(vector<vector<float>  > &v, vector<bool> *b = NULL)
	{
		vector<float> center_of_mass;
		int M = v.size();         //Number of points
		if(M < 1) return center_of_mass;
		int N = (int)v[0].size(); //Number of dimensions
		if(N < 1) return center_of_mass;
		float M00 = (float)M;        //Central moment
		vector<float> Md(N, 0.0);         //Moments each dimension
		if(!b) //Examine all points
		{
			for(int i = 0; i<M; ++i)
			{
				for(int j=0; j<N; ++j)
				{
					Md[j] += v[i][j];
				}
			}
		} else //Examine only valid (marked) points
		{
			if((int)(b->size()) != M) return center_of_mass;
			M00 = 0.0000000001;
			for(int i = 0; i<M; ++i)
			{
				if((*b)[i])
				{
					M00 += 1.0;
					for(int j=0; j<N; ++j)
					{
						Md[j] += v[i][j];
					}
				}
			}
		}
		for(int j = 0; j<N; ++j)
			Md[j] /= M00;
		return Md;
	}

	/**
	 * \brief Return the mean and std city block distance in N dimensions of point p to a set of ND points v
	 * @param v              vector of vector of N dimensional points
	 * @param p              N Dimensional point vector
	 * @param std			 Standard deviation of the distances
	 * @return               Average distance or max possible distance on error
	 */
	float avgCityDistanceND(const vector<vector<float> > &v, vector<float> &p, float &std)
	{
		std = 0.0;
		int M = (int)v.size();
		int D = (int)p.size();
//		printf("avgCityDist M = %d, D = %d v[0].size = %d\n",M,D,(int)v[0].size());
		if((M<1)||(D<1)) return (float)numeric_limits<float>::max();
		if(D != (int)v[0].size()) return (float)numeric_limits<float>::max();
		float total_dist = 0.0, dist = 0.0, point_dist;
		for(int i=0; i<M; ++i)
		{
			point_dist = 0.0;
			for(int j=0; j<D; ++j)
			{
				dist = v[i][j] - p[j];
				if(dist < 0.0) dist = - dist;
				point_dist += dist;
			}
			total_dist += point_dist;
			std += point_dist*point_dist;
		}
		total_dist /= (float)M;
		std = cvSqrt( std/(float)M - total_dist*total_dist) + 0.000001;
		return total_dist;
	}

	/**
	 * \brief Return the closest point in v to p skipping things marked by b
	 * @param v              vector of vector of N dimensional points
	 * @param p              N dimensional point
	 * @param b              boolean marker for valid points
	 * @param index			 RETURN: Index of closest point in v; -1 is an error -- no point found
	 * @return               closest point in v to p; vector size of zero means error
	 */
	vector<float> findClosestPointToPinV(const vector<vector<float> > &v, vector<float> &p, vector<bool> &b, int &index)
	{
		index = -1;
		int M = (int)v.size();
		int D = (int)p.size();
		vector<float> closest_point;
		if((M<1)||(D<1)) return closest_point;
		if(D != (int)v[0].size()) return closest_point;
//		printf("findClosestPoint: p(%f, %f) M=%d, D=%d, v[0].size=%d\n",p[0],p[1],M,D,(int)v[0].size());
		closest_point.resize(D,0.0);
		float point_dist, min_dist = 9999999999.0, dist = 0.0;

		for(int i=0; i<M; ++i)
		{
//			printf("i(%d): ",i);
			if(b[i]) //Skip far points
			{
//				printf("b1 ");
				point_dist = 0.0;
				for(int j=0; j<D; ++j)
				{
					dist = v[i][j] - p[j];
	//				printf("j%d:dist(%f) = v[%d][%d](%f) - p[%d](%f) ",j,dist,i,j,v[i][j],i,p[j]);
					if(dist < 0.0) dist = - dist;
					point_dist += dist;
				}
//				printf("point_dist(%f) <= min_dist(%f)",point_dist,min_dist);
				if(point_dist <= min_dist)
				{
					index = i;
					min_dist = point_dist;
					closest_point = v[i];
//					printf("index=%d, min_dist=%f, pt[%f, %f]\n",index,min_dist,v[i][0],v[i][1]);
				}
			}
		}
		return closest_point;
	}




	/**
	 * \brief This function returns a vector of offsets in a 2D X,Y,Z,D Mat of N triangle vertices around a point
	 *
	 * @param N            -- Number of triangle vertices offsets to create
	 * @param width        -- Width of the 2D matrix of float values
	 * @param max_radius   -- Maximum radius of the triangle (must be > min_radius)
	 * @param min_radius   -- Minimum radius of the triangle (must be > 0.0)
	 * @param num_channels -- number of channels in the 2D matrix of floats.  Def: 4 for X,Y,Z,Disparity
	 * @return  Returns a vector of N triangle vertices offsets in the vector<triangle_offsets>
	 */
	vector<triangle_offsets> computeNTriangleOffsets(int N, int width, float max_radius, float min_radius = 1.0,
			int num_channels = 4)
	{
		vector<triangle_offsets> v;
		for(int i = 0; i<N; i++)
		{
			v.push_back(computeTriangeOffsets(width,max_radius,min_radius,num_channels));
		}
		return v;
	}

	/**
	 * \brief This function returns the offsets within a 2D X,Y,Z,D Mat that form a triangle around a point
	 *
	 * @param width        -- Width of the 2D matrix of float values
	 * @param max_radius   -- Maximum radius of the triangle (must be > min_radius)
	 * @param min_radius   -- Minimum radius of the triangle (must be > 0.0)
	 * @param num_channels -- number of channels in the 2D matrix of floats.  Def: 4 for X,Y,Z,Disparity
	 * @return  Returns the offsets in the structure triangle_offsets
	 */
	triangle_offsets computeTriangeOffsets(int width, float max_radius, float min_radius = 1.0,
			int num_channels = 4)
	{
		triangle_offsets t_o;
		t_o.v0 = 0; t_o.v1 = 0; t_o.v2 = 0;
		//CHECKS
		float h_width = (float)(width>>1);
		if(min_radius < 1.0) 						return t_o;
		if(max_radius > h_width) 					max_radius = h_width;
		if(max_radius < min_radius)					return t_o;
		if(num_channels < 1)						return t_o;
		//SET UP MATRICES
		CvMat matM,matA,matX,matY;
		float Mag[3],Ang[3],X[3],Y[3];
		float radius_range = max_radius - min_radius;
		//FILL WITH RADIUS AND ANGLE VALUES
		Mag[0] = (float)(radius_range*cvRandReal(&rng_state) + min_radius);
		Mag[1] = (float)(radius_range*cvRandReal(&rng_state) + min_radius);
		Mag[2] = (float)(radius_range*cvRandReal(&rng_state) + min_radius);
		Ang[0] = (float)(120.0*cvRandReal(&rng_state));
		Ang[1] = Ang[0]+120.0;
		Ang[2] = Ang[1]+120.0;
		cvInitMatHeader(&matM,1,3,CV_32FC1,Mag);
		cvInitMatHeader(&matA,1,3,CV_32FC1,Ang);
		cvInitMatHeader(&matX,1,3,CV_32FC1,X);
		cvInitMatHeader(&matY,1,3,CV_32FC1,Y);
		//COMPUTE VECTOR END POINTS
		cvPolarToCart(&matM, &matA, &matX, &matY,1);
		//TURN THEM INTO OFFSETS FROM A POINT ON THE GRID
		float roundX,roundY;
		if(Y[0] < 0) roundY = -0.5; else roundY = 0.5;
		if(X[0] < 0) roundX = -0.5; else roundX = 0.5;
		t_o.v0 = num_channels*((int)(Y[0]+ roundY)*width + int(X[0]+roundX));
		if(Y[1] < 0) roundY = -0.5; else roundY = 0.5;
		if(X[1] < 0) roundX = -0.5; else roundX = 0.5;
		t_o.v1 = num_channels*((int)(Y[1]+ roundY)*width + int(X[1]+roundX));
		if(Y[2] < 0) roundY = -0.5; else roundY = 0.5;
		if(X[2] < 0) roundX = -0.5; else roundX = 0.5;
		t_o.v2 = num_channels*((int)(Y[2]+ roundY)*width + int(X[2]+roundX));
		return t_o;
	}
	/**
       * \brief Converts an X,Y,Z stereo point cloud containing originating pixels (x,y) to a floating point 4 channel matrix M
       * @param M    Floating point 4 channel matrix with rows and cols able to contain the point cloud
       * @param rect Region of interest -- only record points inside this box
       * @return 0:OK; -1: no matrix; -2: matrix not 4 float channels; -3: image not floating point
       */
      int pointCloud2PointMat(CvMat *M, const CvRect & rect, const robot_msgs::PointCloud & cloud)
      {
          if(!M)
              return -1;
  //         if((M->type & CV_32FC4) != CV_32FC4)
          if(CV_32FC4 != cvGetElemType(M))
              return -2;
          cvSetZero(M); //Start with clean slate
          //FIND IF WE HAVE ARRAY INDICES (x,y) from original pixel locations
          int xchan = -1;
          int ychan = -1;
          for(size_t i = 0;i < cloud.chan.size();++i){
             if(cloud.chan[i].name == "x"){
                 xchan = i;
             }
             if(cloud.chan[i].name == "y"){
                ychan = i;
             }
             if((xchan >= 0)&&(ychan >= 0))
                 break;
          }
          //STUFF THE IMAGE ARRAY
          int rows = M->rows; //rows and cols better be big enough for max (x,y)
          int cols = M->cols;
          int rx = rect.x;
          int rw = rect.x + rect.width;
          int ry = rect.y;
          int rh = rect.y + rect.height;
          if(xchan != -1 && ychan != -1){
              for(size_t i = 0;i < cloud.pts.size();++i)
              {
                  int x   = (int)(cloud.chan[xchan].vals[i]);
                  if((x<0)||(x>=cols)){//xxx
                      ROS_INFO("x(%d)=%d out of bounds(%d)\n",i,x,cols);
                      x = rows - 1;
                  }
                  int y   = (int)(cloud.chan[ychan].vals[i]);
                  if((y<0)||(y>=rows)){
                      ROS_INFO("y(%d)=%d out of bounds(%d)\n",i,y,rows);
                      y = cols - 1;
                  }
                  float X = (float)(cloud.pts[i].x);
                  float Y = (float)(cloud.pts[i].y);
                  float Z = (float)(cloud.pts[i].z);
 //                 if(!(y%96)&&!(x%128))
 //               	  printf("pt2mat (%f, %f, %f\n",X,Y,Z);
                  //Put this value into the image if it's inside the ROI
                  if(x >= rx && x < rw && y >= ry && y < rh){
//                	  float *p = M->data.fl + y*cols*4 + x*4;
//                	  *p++ = X;
//                	  *p++ = Y;
//                	  *p++ = Z;
//                	  *p++ = 1.0;
                      cvSet2D( M, y, x, cvScalar(X,Y,Z,1.0) );
                  }//end if in ROI
              } //end for each point
          }
          return 0;
       }

    /**
     * \brief Converts an X,Y,Z stereo point cloud containing originating pixels (x,y) to a floating point 3 channel image I
     * @param I    Floating point 3 channel image with width and height able to contain the point cloud
     * @param rect Region of interest -- only record points inside this box
     * @return 0:OK; -1: no image; -2: image not 3 channels; -3: image not floating point
     */
    int pointCloud2PointImage(IplImage *I, const CvRect & rect, robot_msgs::PointCloud & cloud)
    {
       	if(!I)
        	return -1;
		if(I->nChannels != 3 )
			return -2;
		if(I->depth != IPL_DEPTH_32F)
			return -3;
		cvSetZero(I); //Start with clean slate
		//FIND IF WE HAVE ARRAY INDICES (x,y) from original pixel locations
		int xchan = -1;
		int ychan = -1;
		for(size_t i = 0;i < cloud.chan.size();++i){
		   if(cloud.chan[i].name == "x"){
			   xchan = i;
		   }
		   if(cloud.chan[i].name == "y"){
			  ychan = i;
		   }
		   if((xchan >= 0)&&(ychan >= 0))
			   break;
		}
		//STUFF THE IMAGE ARRAY
		int width = I->width; //Width and height better be big enough for max (x,y)
		int height = I->height;
		int rx = rect.x;
		int rw = rect.x + rect.width;
		int ry = rect.y;
		int rh = rect.y + rect.height;
		if(xchan != -1 && ychan != -1){
			for(size_t i = 0;i < cloud.pts.size();++i){
				int x   = (int)(cloud.chan[xchan].vals[i]);
				if((x<0)||(x>=width)){
					ROS_INFO("x(%d)=%d out of bounds(%d)\n",i,x,width);
					x = width - 1;
				}
				int y   = (int)(cloud.chan[ychan].vals[i]);
				if((y<0)||(y>=height)){
					ROS_INFO("y(%d)=%d out of bounds(%d)\n",i,y,height);
					y = height - 1;
				}
				float X = (float)(cloud.pts[i].x);
				float Y = (float)(cloud.pts[i].y);
				float Z = (float)(cloud.pts[i].z);
				//Put this value into the image if it's inside the ROI
				if(x >= rx && x < rw && y >= ry && y < rh){
					cvSet2D( I, y, x, cvScalar(X,Y,Z) );
				}//end if in ROI
			} //end for each point
		}
        return 0;
     }
};


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

void on_mouse_feature_location(int event, int x, int y, int flags, void* param);
void on_edges_low(int);
void on_edges_high(int);
void on_depth_near(int);
void on_depth_far(int);
void on_num_triangles(int val);
void on_tri_radius_max(int val);
void on_tri_radius_min(int val);
void on_quality(int val);


class RecognitionLambertian : public ros::Node
{
public:
	sensor_msgs::Image limage;
	sensor_msgs::Image rimage;
	sensor_msgs::Image dimage;
	sensor_msgs::StereoInfo stinfo;
	sensor_msgs::DisparityInfo dispinfo;
	sensor_msgs::CamInfo rcinfo;
	sensor_msgs::CvBridge lbridge;
	sensor_msgs::CvBridge rbridge;
	sensor_msgs::CvBridge dbridge;

	robot_msgs::PointCloud cloud_fetch;
	robot_msgs::PointCloud cloud;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;
	IplImage* color_depth;

	CvMat *M;  //Will hold X,Y,Z,A data.  Here, A <= 0 means no or bad data
	CvMat *nM; //Will hold A,B,C,A data.  Here, A <= 0 means no or bad data

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
	int depth_near,depth_far;
   	int num_triangles, tri_radius_max, tri_radius_min;

	typedef pair<int,int> coordinate;
	typedef vector<coordinate> template_coords_t;

	vector<CvSize> template_sizes;
	vector<template_coords_t> template_coords;

	float min_scale;
	float max_scale;
	int count_scale;

	int quality_thresh;


	CvHaarClassifierCascade* cascade;
	CvMemStorage* storage;

    RecognitionLambertian()
    :ros::Node("stereo_view"), left(NULL), right(NULL), disp(NULL), disp_clone(NULL),
    color_depth(NULL), sync(this, &RecognitionLambertian::image_cb_all,
    		ros::Duration().fromSec(0.1), &RecognitionLambertian::image_cb_timeout),
    		M(NULL), nM(NULL)
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
        depth_near = 1;
        depth_far = 20;


        min_scale = 0.7;
        max_scale = 1.2;
        count_scale = 7;

        if(display){
            cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("color_depth",CV_WINDOW_AUTOSIZE);
//            cvNamedWindow("disparity_original", CV_WINDOW_AUTOSIZE);
        	cvNamedWindow("edges",1);
        	cvCreateTrackbar("edges_low","edges",&edges_low, 500, &on_edges_low);
        	cvCreateTrackbar("edges_high","edges",&edges_high, 500, &on_edges_high);

        	cvCreateTrackbar("near","color_depth",&depth_near, 49, &on_depth_near);
        	cvCreateTrackbar("far","color_depth",&depth_far, 50, &on_depth_far);

        	num_triangles = 18;
        	tri_radius_max = 21;
        	tri_radius_min = 8;
        	quality_thresh = 2; //on 20 scale
           	cvCreateTrackbar("num_tri","color_depth",&num_triangles,   25,&on_num_triangles);
           	cvCreateTrackbar("max_tri_R","color_depth",&tri_radius_max,50,&on_tri_radius_max);
           	cvCreateTrackbar("min_tri_R","color_depth",&tri_radius_min,30,&on_tri_radius_min);
           	cvCreateTrackbar("quality","color_depth",&quality_thresh,20,&on_quality);
           	//MOUSE FEATURE INTERACTION
           	cvSetMouseCallback("color_depth", on_mouse_feature_location, 0);
         }


//        advertise<robot_msgs::PointStamped>("handle_detector/handle_location", 1);
       // advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 1);

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
        if(color_depth){
        	cvReleaseImage(&color_depth);
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

    		ROS_INFO("Level: %d, scale: %f, width: %d, height: %d\n", i, scale, width, height);

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
     * Given a disparity images and a ROI in the image t
     * his function computes the approximate real-world size
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
    }


    //NOT USED
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
            if((xchan >= 0)&&(ychan >= 0))
         	   break;
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
     * \brief Create a pretty color depth map image from a stereo point cloud that has originating (x,y) pixel indices
     * @param I 3          channel 8 bit B,G,R image
     * @param rect         region of interest -- only fill in depth here
     * @param start_depth  skip depths closer than this (in meters)
     * @param stop_depth   skip depths further than this (in meters)
     * @return 0:OK; -1: no image; -2: image not 3 channels; -3: start_depth >= stop_depth
     */
    int colorDepthImage(IplImage *I, const CvRect & rect, float start_depth = 0.1, float stop_depth = 2.0 )
    {
    	if(!I)
    		return -1;
    	if(I->nChannels != 3 )
    		return -2;
     	cvSetZero(I); //Start with clean slate
     	if(start_depth >= stop_depth)
    		return -3;
        int xchan = -1;
        int ychan = -1;
        for(size_t i = 0;i < cloud.chan.size();++i){
           if(cloud.chan[i].name == "x"){
               xchan = i;
           }
           if(cloud.chan[i].name == "y"){
              ychan = i;
           }
           if((xchan >= 0)&&(ychan >= 0))
        	   break;
        }
        float B,G,R;
        int width = I->width;
        int height = I->height;
        float zfrac = 768.0/(stop_depth - start_depth); //768 = 3*256 to divide up pixel color into R,G,B
        if(xchan != -1 && ychan != -1){
            for(size_t i = 0;i < cloud.pts.size();++i){
                int x   = (int)(cloud.chan[xchan].vals[i]);
                if((x<0)||(x>=width)){
                	ROS_INFO("x(%d)=%d out of bounds(%d)\n",i,x,width);
                	x = width - 1;
                }
                int y   = (int)(cloud.chan[ychan].vals[i]);
                if((y<0)||(y>=height)){
                	ROS_INFO("y(%d)=%d out of bounds(%d)\n",i,y,height);
                	y = height - 1;
                }
                float z = (float)(cloud.pts[i].z);
                //Put this value into the image
                if(x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height){
                	z -= start_depth;
                	if(z < 0.0)
                		continue;
                	z *= zfrac;
                	if(z <= 256.0){
                		R = 256.0 - z;
                		G = z;
                		B = 0.0;
                	} else if(z <= 512.0) {
                		R = 0.0;
                		G = 512.0 -z;
                		B = z - 256.0;
                	} else if(z <= 768.0){
                		R = 0.0;//z - 512.0;
                		G = 0.0;
                		B = 768.0 - z;
                	} else {
                		R = G = B = 0.0;
                	}
                	cvSet2D( I, y, x, cvScalar(B,G,R) );
                }//end if in ROI
            } //end for each point
        }
        return 0;
    }

    /**
     * \brief Create a pretty color depth map image from a stereo point cloud that has originating (x,y) pixel indices
     * @param I 		   3 channel 8 bit B,G,R image
     * @param M            4 channel float array containing X,Y,Z,A from which o read he data from
     * @param rect         region of interest -- only fill in depth here
     * @param start_depth  skip depths closer than this (in meters)
     * @param stop_depth   skip depths further than this (in meters)
     * @return 0:OK; -1: no image; -2: I!=3 channels; -3: start_depth >= stop_depth; -4: No M, -5 [w|h]!=[c|r]
     */
    int colorDepthImageFromMat(IplImage *I, CvMat *M, const CvRect & rect,
			float start_depth = 0.1, float stop_depth = 2.0) {
		if (!I)
			return -1;
		if (I->nChannels != 3)
			return -2;
		cvSetZero(I); //Start with clean slate
		if (start_depth >= stop_depth)
			return -3;
		float B, G, R;
		int width = I->width;
		int height = I->height;
		if (!M)
			return -4;
		int rows = M->rows;
		int cols = M->cols;
		if ((cols != width) || (rows != height))
			return -5;

		float zfrac = 768.0 / (stop_depth - start_depth); //768 = 3*256 to divide up pixel color into R,G,B
//		printf("colorDepthIMfromMat zfrac=%f\n",zfrac);
		float *pM = M->data.fl + 2; //+2 so it points to z of x,y,z,d
		uchar *pI = (uchar *)(I->imageData);
		char p_jumpby = I->widthStep - width * 3;
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x, pI+=3)
			{
				//				CvScalar xyza = cvGet2D(M, y, x); //Slow way
				//				float z = (float) (xyza.val[2]);
				float z = *pM; //Fast way
				pM += 4;
				//Put this value into the image
				if (x >= rect.x && x < rect.x + rect.width && y >= rect.y && y
						< rect.y + rect.height) {
					z -= start_depth;
					if (z <= 0.0)
						continue;
					z *= zfrac;
					if (z <= 256.0) {
						R = 256.0 - z;
						G = z;
						B = 0.0;
					} else if (z <= 512.0) {
						R = 0.0;
						G = 512.0 - z;
						B = z - 256.0;
					} else if (z <= 768.0) {
						R = 0.0;//z - 512.0;
						G = 0.0;
						B = 768.0 - z;
					} else {
						R = G = B = 0.0;
					}
//					cvSet2D(I, y, x, cvScalar(B, G, R)); //Slow way
					*pI = (uchar)B; //Fast way
					*(pI + 1) = (uchar)G;
					*(pI + 2) = (uchar)R;
				}//end if in ROI
//				else
//					pI += 3;
			}
			pI += p_jumpby;
		} //end for each point
		return 0;
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
            if(left && (color_depth == NULL)){
            	color_depth = cvCreateImage(cvGetSize(left), IPL_DEPTH_8U, 3);
            }
        }
        if(rbridge.fromImage(rimage, "bgr")){
            if(right != NULL)
                cvReleaseImage(&right);

            right = cvCloneImage(rbridge.toIpl());
        }
        if(dbridge.fromImage(dimage)){
            if(disp != NULL)
                cvReleaseImage(&disp);

            disp = cvCloneImage(dbridge.toIpl());
        }

        cloud = cloud_fetch;

        runRecognitionLambertian();
        printf("image_cb_all->displayColorDephtImage\n");
        displayColorDepthImage();
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

#define subsample 5  //skip factor for normal computation
    void displayColorDepthImage()
    {
    	CvRect R = cvRect(0,0,color_depth->width, color_depth->height);
     	float Dn = depth_near/10.0;
    	float Df = depth_far/10.0;
    	float quality = (float)quality_thresh/20.0;
    	if(M)
        	cvReleaseMat( &M );
    	if(nM)
        	cvReleaseMat(&nM);
    	CvMat *M = cvCreateMat( color_depth->height, color_depth->width, CV_32FC4 );
    	CvMat *nM = cvCreateMat( color_depth->height, color_depth->width, CV_32FC4 ); //Holds normals
    	//FILL THE POINT CLOUD
    	StereoPointCloudProcessing spc;
    	spc.pointCloud2PointMat(M, R, cloud);

    	//COMPUTE PLANES:
    	vector<vector<double> > models;
    	int num_pts = color_depth->height*color_depth->width/20;
  	    int npl= spc.segmentPlanesFromXYZAGrid(M, 0.0, 50.0, num_pts, 10, models); //M has planes marked out
 	    printf("Number of planes found = %d, models = %d\n",npl, (int)models.size());
    	colorDepthImageFromMat(color_depth,M,R,Dn,Df);

    	//PRE-COMPUTE TRIANGLE OFFSETS TO USE
		vector<triangle_offsets> Trioffs = spc.computeNTriangleOffsets(num_triangles, 640, tri_radius_max, tri_radius_min);
		ROS_INFO("triangle offsets: %d\n",Trioffs.size());

		//COMPUTE NORMALS
  	    int num_norms = spc.computeNormals(M, nM, R, Trioffs, 400.0, quality, subsample);
  	    printf("Number of normals = %d\n",num_norms);
  	    if(num_norms < 0) return;

//  	    CvMat *RT = cvCreateMat(4,4,CV_32FC1);
//  	    CvMat *Rot = cvCreateMat(4,4,CV_32FC1);;
//  	    printf("spc.computeRTtoCameraFromNormal\n");
//  		spc.computeRTtoCameraFromNormal(cvPoint3D32f(1.0,2.0,3.0), cvPoint3D32f(3.0,2.0,1.0), RT, Rot );
//  		cvReleaseMat(&RT);
//  		cvReleaseMat(&Rot);

  	    //SET UP:
  	    int rx = R.x;
  	    int ry = R.y;
  	    int rw = rx + R.width;
  	    int rh = ry + R.height;
  	    float *fM,*fnM;
  	    int jumpby = subsample*4;
  	    int cols = color_depth->width;
		vector<Vector3 > coef;
		PointCloud points;
		points.header.frame_id = cloud.header.frame_id;
		points.header.stamp = cloud.header.stamp;
		Vector3 v3;
		Point32 p3;

		//Send normals to rviz
		for (int y=ry;y<rh;y+=subsample)
		{
			fM = M->data.fl + 4*y*cols + rx*4;
			fnM = nM->data.fl + 4*y*cols + rx*4;
			for (int x=rx;x<rw;x+=subsample, fM += jumpby, fnM += jumpby )
			{
				int nindex =  (int)(*(fnM + 3));

				if (nindex>=0) {

					p3.x = *fM;
					p3.y = *(fM + 1);
					p3.z = *(fM+2);
					v3.x = *fnM;
					v3.y = *(fnM + 1);
					v3.z = *(fnM + 2);
					points.pts.push_back(p3);
					coef.push_back(v3);
				}
			}
		}

		publishNormals(this, points, coef, -0.04);

//    	colorDepthImage(color_depth,R,Dn,Df);
//    	printf("colorDepthImage ret = %d",r);
    	cvShowImage("color_depth",color_depth);
    }

    void do_mouse(int event, int x, int y)
    {
    	if(event == CV_EVENT_LBUTTONUP){
    		cvCircle(color_depth, cvPoint(x,y), 10, CV_RGB(255,255,255), 2);
        	cvShowImage("color_depth",color_depth);
    	}
    }
};

RecognitionLambertian* node;

void on_mouse_feature_location(int event, int x, int y, int flags, void* param)
{
	node->do_mouse(event,x,y);
}



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

// To control color coded depth viewing
void on_depth_near(int value)
{
	node->depth_near = value;
	node->displayColorDepthImage();
}

void on_depth_far(int value)
{
	node->depth_far = value;
	node->displayColorDepthImage();
}


void on_num_triangles(int val)
{
	node->num_triangles = val;
	node->displayColorDepthImage();
}
void on_tri_radius_max(int val)
{
	node->tri_radius_max = val;
	node->displayColorDepthImage();
}
void on_tri_radius_min(int val)
{
	node->tri_radius_min = val;
	node->displayColorDepthImage();
}

void on_quality(int val)
{
	node->quality_thresh = val;
	node->displayColorDepthImage();
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

