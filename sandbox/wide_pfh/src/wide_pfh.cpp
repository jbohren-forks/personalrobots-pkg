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

#include "wide_pfh/wide_pfh.h"
#include "wide_pfh/point_cloud_mapping.h"

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>

#include <point_cloud_mapping/geometry/point.h>
#include <vector>


using namespace std;



typedef struct _triangle_offsets
{
	int  v0;
	int  v1;
	int  v2;
}
triangle_offsets;


CvRNG rng_state = cvRNG(0xffffffff);


void onePFH(float *ps, float *u_, float *pt, float *nt_, vector<float> &results, bool norm = true);


vector<triangle_offsets> computeNTriangleOffsets(int N, int width, int fstep, float max_radius, float min_radius = 1.0,
		int num_channels = 4);

triangle_offsets computeTriangeOffsets(int width, int fstep, float max_radius, float min_radius = 1.0,
		int num_channels = 4);

int computeNormals(const CvMat *xyza, CvMat *xyzn, CvRect roi, vector<triangle_offsets> &Trioffs,
		float radius_sqr, float quality, int skip = 1);


void normalize(float *src, float *dst);


void crossProd3D(const float *A, const vector<float> &B, vector<float> &res);

void crossProd3D(const vector<float> &A, const float *B, vector<float> &res);

void crossProd3D(const vector<float> &A, const vector<float> &B, vector<float> &res);

int computeNormalAtPt(const CvMat *xyzd, const CvPoint &p, const vector<triangle_offsets> &v, float radius_sqr,
		vector<float> &normal_pt3d, vector<vector<float> > &normals, vector<vector<float> > &normal_angles,
		float &quality);


vector<float> findMedianND(vector<vector<float> > &v, int &median_index);

vector<float>  centerOfMassND(vector<vector<float>  > &v, vector<bool> *b = NULL);

float avgCityDistanceND(const vector<vector<float> > &v, vector<float> &p, float &std);

void markClosePoints(const vector<vector<float> > &v, const vector<float> &p, const float max_dist, vector<bool> &b);

vector<float> findClosestPointToPinV(const vector<vector<float> > &v, vector<float> &p, vector<bool> &b, int &index);

//**********************vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*************************//
//**********************FUNCTIONS SPECIFIC TO THE WPFH FEATURES*************************//
//**********************vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*************************//


#define NBINS1 8     //Number of bins for alpha, phi and theta (see Page 51 of Radu Rusu's thesis//
#define NBINS2 8     // This seems to be a good compromise between accuracy and speed. You can change these independently
#define NBINS3 8
#define BFACT1 NBINS1/2.0   // (NBINS*[Range(0-2)]/2.0)
#define BFACT2 NBINS2/2.0   // (NBINS*[Range(0-2)]/2.0)
#define BFACT3 NBINS3/2.0   // (NBINS*[Range(0-2)]/2.0)
/**
* \brief Turns a vector, pna, of pt.x,pt.y,pt.z,n.x,n.y,n.z (points and normals) into a histogram vector of len NBINS1*NBINS2*NBINS3 IN ORDER: alpha, phi,theta
* @param pna	-- "point,normal,array" vector of locations pt.x,pt.y,pt.z,n.x,n.y,n.z (points and normals), no bad points allowed
* @param h		-- vector containing the histogram with indexing alpha+phi*NBINS2+theta*NBINS3
* @return		-- <0 => bad pna
*/
int assembleWPFH(vector<vector<float> > &pna, vector<float> &h){
	if(h.size() != NBINS1*NBINS2*NBINS3) {
		h.resize(NBINS1*NBINS2*NBINS3);
	}
	h.assign(NBINS1*NBINS2*NBINS3,0.0); //Clear her out
	int bin3sqr = NBINS3*NBINS3;
	vector<float> angles(4);
	float *ps, *u_,*pt, *nt_,count = 0.0;
	unsigned int size = pna.size();
	if(size < 4) return -size - 1;
	int indxa,indxp,indxt,indx;
	//PUT IN THE (N-1)*(N-2)/2 VALUES:
	for(unsigned int i = 0; i<size; ++i){
		ps  = &pna[i][0];
		u_  = &pna[i][3];
		if((*u_ == 0.0)&&(*(u_+1) == 0.0)&&(*(u_+2) == 0)) continue;
		for(unsigned int j = i+1; j<size; ++j){
			pt  = &pna[j][0];
			nt_ = &pna[j][3];
			if((*nt_ == 0.0)&&(*(nt_+1) == 0.0)&&(*(nt_+2) == 0.0)) continue; //skip null normals
			onePFH(ps,u_,pt,nt_,angles,true);		//All angles come back [-1,1)
			indxa  = (int)((angles[0] + 1.0)*BFACT1);
			indxp  = (int)((angles[1] + 1.0)*BFACT2);
			indxt  = (int)((angles[2] + 1.0)*BFACT3);
			indx = indxa + indxp*NBINS2 + indxt*bin3sqr;
			h[indx] += 1.0;
			count += 1.0;
		}
	}
	//NORMALIZE
	for(unsigned int i = 0; i<h.size(); ++i){
		h[i] /= count;
	}
	return 0;
}



#define LEAF_DIM 0.01    			//Dimension of a voxel cube LEAF_DIM x LEAF_DIM x LEAF_DIM -- set to get ~~100 points on a object
#define CUT_DISTANCE 2.0 			//Ignore Z distances greater than this (in meters
#define NUM_TRIANGES 18  			//We compute the median normal of this many triangles [Adjust so that the normals look good]
#define MIN_SUB_SAMPLED_POINTS 18 	//When grid is sub_sampled, we must have at least this many points (note we have to fill up to 512 bins
#define MAXIMUM_TRIANGLE_RADIUS (2.0*LEAF_DIM) //Min radius is 1/3 of this for computing normals [Adjust this so that the normals look good]
#define TRIANGLE_QUALITY 0.1		//successful_normals_computed/num_triangles >= quality, accept the normal
/**
* \brief Compute the Wide (large, sparse) PFH (WPFH) for an grid point cloud
* @param xyzd_	Pointer to a 2D float CvMat grid of X,Y,Z,D channels where D = 0 => invalid point
* @param xyzn	Pointer to same size and type matrix as xyzd_. Dense normals will be computed here.
* @param wpfh	Return: Will fill this with a 512 long histogram of alpha*8^0 + phi*8^1 + theta*8^2 entries
* @param roi	CvRect. If set, use only that part of xyzd. If NULL (Default), use all xyzd
* @return: <=0 => error, else good
*/
int calcWPFH(const CvMat *xyzd_, CvMat *xyzn, vector<float> &wpfh,/* vector<int> &down_size,*/ CvRect *roi)
{
	if(CV_MAT_TYPE(xyzd_->type) != CV_32FC4) {
		ROS_ERROR ("Wrong type matrix passed to calcWPFH. It should be %d but was %d",CV_32FC4, CV_MAT_TYPE(xyzd_->type));
		return -1;
	}
	//FIRST SET xyzd SO THAT IT FITS THE GIVEN roi
	float *base_ = xyzd_->data.fl;
	int rows_ = xyzd_->rows;
	int cols_ = xyzd_->cols;
	int fstep = xyzd_->step/4;  //Increment to move a float pointer one row
	int xs,xe,ys,ye;  //Xstart and End
	if(!roi){ //Default to whole array
		xs = 0; xe = cols_;
		ys = 0; ye = rows_;
	}
	else { //Deal with ROI if set
		xs = roi->x;
		xe = xs + roi->width;
		ys = roi->y;
		ye = ys + roi->height;
		if((xs < 0)||(xs >= cols_)) xs = 0; //clip roi to be within array
		if((xe < 0)||(xe >= cols_)) xe = cols_;
		if((ys < 0)||(ys >= rows_)) ys = 0;
		if((ye < 0)||(ye >= rows_)) ye = rows_;
	}
	int rows = ye - ys;
	int cols = xe - xs;
	CvRect R = cvRect(xs,ys,cols,rows);
	CvMat xyzd;
	cvInitMatHeader(&xyzd, rows, cols, xyzd_->type, base_ + ys*fstep + xs*4, xyzd_->step);

	//NOW CREATE INDICES OF EXISTING POINTS
	float *fb = xyzd.data.fl; //new base pointer
	vector<int> indices;
	for(int y = 0; y<xyzd.rows; ++y){
		int offset = y*fstep;
		for(int x = 0; x<xyzd.cols; ++x, offset += 4){
			if(*(fb + offset + 3) < 0.001) continue; //Skip points where "D" = 0 (further offset + 3)
			indices.push_back(offset);
		}
	}

	//SUBSAMPLE THESE POINTS
	vector<int> sub_sampled_pts; //This will contain our downsampled offsets to the points
	geometry_msgs::Point leaf_size; leaf_size.x = LEAF_DIM; leaf_size.y = LEAF_DIM; leaf_size.z = LEAF_DIM;
	std::vector<cloud_geometry::Leaf> leaves;
	double cut_distance = CUT_DISTANCE;
	cloud_geometry::downsamplePointCloud (&xyzd, indices, sub_sampled_pts, leaf_size, leaves, cut_distance);
	int num_sub_sampled = (int)sub_sampled_pts.size();
	if(num_sub_sampled < MIN_SUB_SAMPLED_POINTS){
		ROS_WARN("Warning: In calcWPFH, minimum subsampled points was %d but must be at least %d\n",num_sub_sampled,MIN_SUB_SAMPLED_POINTS);
		return -1;
	}

	//TRY TO COMPUTE A RESONABLE MAX AND MIN RADIUS FOR TRIANGLES USING THE COMPUTED INDICES.
	//We need to know how much world distance is between adjacent pixels on average in order to set sensible, adaptive values
	float dist2_avg = 0.0;
	int dcnt = 0;
	float dX,dY,dZ,X,Y,Z;
	int offset_min_limit = fstep + 4;
	int offset_max_limit = (xyzd.rows-2)*fstep;
	int direction = -1, offset, doffset = 0;
	for(int i = num_sub_sampled/2; i<3*num_sub_sampled/4; ++i){ //elaborate routine to get average pairwise grid pts average world distance
		offset = sub_sampled_pts[i];
		X = *(fb + offset); Y = *(fb + offset + 1); Z = *(fb + offset + 2);
		for(int j = 0; j<4; ++j){
			++direction;
			if(direction > 7) direction = 0;
			switch(direction)
			{
			case 0:
				doffset = 4;
				break;
			case 1:
				doffset = -fstep + 4;
				break;
			case 2:
				doffset = -fstep;
				break;
			case 3:
				doffset = -fstep - 4;
				break;
			case 4:
				doffset = -4;
				break;
			case 5:
				doffset = fstep - 4;
				break;
			case 6:
				doffset = fstep;
				break;
			case 7:
				doffset = fstep + 4;
				break;
			}
			if(((offset + doffset)<offset_min_limit) || ((offset + doffset)>offset_max_limit)) continue;
			if(*(fb + offset + doffset + 3) < 0.001) continue;
			dX = X - *(fb + offset + doffset);
			dY = Y - *(fb + offset + doffset + 1);
			dZ = Z - *(fb + offset + doffset + 2);
			dist2_avg += dX*dX+dY*dY+dZ*dZ;
			++dcnt;
		}
	}
	float dist_avg = sqrt(dist2_avg/(float)dcnt);
	float tri_radius_max = MAXIMUM_TRIANGLE_RADIUS/dist_avg;
	float tri_radius_min = tri_radius_max/3.0 + 1;
	//		cout << "dist_avg = " << dist_avg << ", dcnt =" << dcnt << ", radius_max = " << tri_radius_max << ", radius_min = " << tri_radius_min << endl;

	//PRE-COMPUTE TRIANGLE OFFSETS TO USE
	vector<triangle_offsets> Trioffs = computeNTriangleOffsets(NUM_TRIANGES, xyzd.cols, xyzd.step/4, tri_radius_max, tri_radius_min);
	ROS_INFO("triangle offsets: %d\n",Trioffs.size());

	//COMPUTE NORMALS
	int num_norms = computeNormals(xyzd_, xyzn, R, Trioffs, MAXIMUM_TRIANGLE_RADIUS, TRIANGLE_QUALITY, 1);
	if(num_norms <= 0){
		ROS_WARN("Did not find any normals in calcWPFH() while computeNormals() just finished");
		return num_norms - 1;
	}

	//ADJUST SUBSAMPLED OFFSETS BACK INTO ORIGINAL (NON-ROI) POINT GRID:
	int roi_adjust_offset = fstep*ys + xs*4;
	for(unsigned int i = 0; i<sub_sampled_pts.size(); ++i){
		sub_sampled_pts[i] += roi_adjust_offset;
	}

	//COMPUTE OUR FEATURE VECTORS
	vector<vector<float> > one_obj;
	vector<float> feature(6);
	float *pbase = xyzd_->data.fl, *nbase = xyzn->data.fl; //base pointers
	int oset;
	for(unsigned int i = 0; i<sub_sampled_pts.size(); ++i){
		oset = sub_sampled_pts[i];
		feature[3] = *(nbase + oset); feature[4] = *(nbase  + oset + 1); feature[5] = *(nbase + oset + 2);
		if((feature[3] == 0.0 && feature[4] == 0.0 && feature[5] == 0)|| (*(pbase + oset + 3) < 0.001)) continue; //filter bad values
		feature[0] = *(pbase + oset); feature[1] = *(pbase  + oset + 1); feature[2] = *(pbase + oset + 2);
		one_obj.push_back(feature);
	}
	int wpfhres =  assembleWPFH(one_obj, wpfh);  //This is the core function
	if(wpfhres < 0){
		ROS_WARN("Not enough points to assembleWPFH() in calcWPFH()");
		return wpfhres;
	}

	//AND REPORT THE NUMBER OF VECTORS USED
	//	    down_size = sub_sampled_pts;
	//	    cout << "Number of subsampled points: " << down_size.size() << " vs. number with good normals: " << one_obj.size() << endl;
	return 1;
}

//******************************************************************************************//
//**************************UTILITIES FOR PATTERN RECOGNITION*******************************//
//******************************************************************************************//
//        Chi Sqr is giving me 100% for the first 10 ranks on the 3x57 wine glasses         //

/**
* \brief Compare 2 vector histograms together by Min intersect, Bhattacharrya or Chi Sqr (DEFAULT)
* @param H1		vector Histogram 1
* @param H2		vector Histogram 2
* @param method	0 Min Intersection, 1 Bhattacharrya, 2 Chi Sqr (DEFAULT)
* @return	Score, low is better. > 1.0 => error
*/
float minH4(vector<float> &H1, vector<float> &H2, int method){
	if(method < 0 || method > 2) method = 2;
	float val1,val2,accum = 0.0,accum1=0.0,accum2=0.0;
	unsigned int size = H1.size();
	if(size != H2.size()) return 2.0;
	for(unsigned int i = 0; i<size; ++i){
		val1 = H1[i];
		val2 = H2[i];
		switch(method){
		case 0: //MIN INT
			if(val1 < val2) accum += val1;
			else accum += val2;
			break;
		case 1:  //BHATTACHARYYA
			accum += sqrt(val1*val2);
			accum1 += val1;
			accum2 += val2;
			break;
		case 2:  //CHI SQR
			float valdif = val1 - val2;
			accum += (valdif*valdif)/(val1+val2+0.0000001);
			break;
		}
	}
	switch(method)
	{
	case 0: //MIN INTERSECTION
		accum = 1.0-accum;
		break;
	case 1:  //BHATTACHARYYA
		accum = sqrt(1.0 - (accum/sqrt(accum1*accum2)));
		break;
	case 2:  //CHI SQR
		break;
	}
	return accum;
}

/**
* \brief Compare 2 OpenCV histograms together by Min intersect (DEFAULT), Bhattacharrya or Chi Sqr
* @param H1		OpenCV Histogram 1
* @param H2		OpenCV Histogram 2
* @param binlen	int binlen[4] of number of bins in each dimension
* @param method	0 (Default) Min Intersection, 1 Bhattacharrya, 2 Chi Sqr
* @return	Score, low is better.
*/
float minH4(CvHistogram *H1, CvHistogram *H2, int binlen[], int method = 0){
	float val1,val2,accum = 0.0,accum1=0.0,accum2=0.0;
	int idx[4];
	for(int a = 0; a<binlen[0]; ++a){
		idx[0] = a;
		for(int p = 0; p<binlen[1]; ++p){
			idx[1] = p;
			for(int t = 0; t<binlen[2]; ++t){
				idx[2] = t;
				for(int d = 0; d<binlen[3]; ++d){
					idx[3] = d;
					val1 = cvQueryHistValue_nD(H1, idx);
					val2 = cvQueryHistValue_nD(H2, idx);
					switch(method){
					case 0: //MIN INT
						if(val1 < val2) accum += val1;
						else accum += val2;
						break;
					case 1:  //BHATTACHARYYA
						accum += sqrt(val1*val2);
						accum1 += val1;
						accum2 += val2;
						break;
					case 2:  //CHI SQR
						float valdif = val1 - val2;
						accum += (valdif*valdif)/(val1+val2+0.0000001);
						break;
					}
				}
			}
		}
	}
	switch(method)
	{
	case 0: //MIN INTERSECTION
		accum = 1.0-accum;
		break;
	case 1:  //BHATTACHARYYA
		accum = sqrt(1.0 - (accum/sqrt(accum1*accum2)));
		break;
	case 2:  //CHI SQR
		break;
	}
	return accum;
}

//******************************************************************************************//
//*****CORE FUNCTION COMPUTING PAIRWISE ANGLUAR RELATIONS BETWEEN TWO SURFACE NORMALS*******//
//******************************************************************************************//

/**
* \brief Compute the Point Feature Histogram values between a pair of normals
* See page 51 of Radu Rusu's thesis for the math
* @param ps		Pointer to float X,Y,Z point location of source normal
* @param u_		Pointer to float X,Y,Z source normal parameters
* @param pt		Pointer to float X,Y,Z point location of target normal
* @param nt_		Pointer to float X,Y,Z target normal parameters
* @param results	Will contain alpha, phi, theta, distance in meters. All angles scaled between [-1,1)
* @param norm		If set true (default) then u_ and nt_ will be L2 normed prior to computation
*/
void onePFH(float *ps, float *u_, float *pt, float *nt_, vector<float> &results, bool norm )
{
	vector<float> v(3, 0.0);
	vector<float> w(3, 0.0);
	vector<float> pt_ps(3, 0.0);
	float Nu[4],Nnt[4];
	float *u, *nt;
	if(norm) { //Need to normalize
		normalize(u_,Nu);
		normalize(nt_,Nnt);
		u = Nu;
		nt = Nnt;
	} else { //No need to normalize
		u = u_;
		nt = nt_;
	}
	pt_ps[0] = pt[0] - ps[0]; pt_ps[1] = pt[1] - ps[1];  pt_ps[2] = pt[2] - ps[2];
	float d = sqrt(pt_ps[0]*pt_ps[0] +  pt_ps[1]*pt_ps[1] + pt_ps[2]*pt_ps[2]);
	pt_ps[0] /= d; pt_ps[1] /= d; pt_ps[2] /= d;
	crossProd3D(pt_ps, u, v); //Compute v = pt-ps x u
	crossProd3D(u, v, w);     //Compute w = u x v
	results[0] = v[0]*nt[0] + v[1]*nt[1] + v[2]*nt[2]; //v . nt
	results[1] = u[0]*pt_ps[0] + u[1]*pt_ps[1] + u[2]*pt_ps[2]; //u . (pt - ts)/d
	float deg = cvFastArctan(w[0]*nt[0] + w[1]*nt[1] + w[2]*nt[2], u[0]*nt[0] + u[1]*nt[1] + u[2]*nt[2]);
	results[2] = (deg - 180.0)/180.0; //arctan(w . nt,  u . nt);
	results[3] = d;
}


/////////////////////////////////////////////////////////////////////////////////
//////////////////////////1st tier support functions/////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//             That is, everything below here can be made private              //

/**
* \brief This function returns a vector of offsets in a 2D X,Y,Z,D Mat of N triangle vertices around a point
*
* @param N            -- Number of triangle vertices offsets to create
* @param width        -- Width of the 2D matrix of float values
* @param fstep		   -- Stride or step of matrix in floats (CvMat.step/4).
* @param max_radius   -- Maximum radius of the triangle on teh grid (must be > min_radius)
* @param min_radius   -- Minimum radius of the triangle on the grid (must be > 0.0)
* @param num_channels -- number of channels in the 2D matrix of floats.  Def: 4 for X,Y,Z,Disparity
* @return  Returns a vector of N triangle vertices offsets in the vector<triangle_offsets>
*/
vector<triangle_offsets> computeNTriangleOffsets(int N, int width, int fstep, float max_radius, float min_radius ,
		int num_channels )
{
	vector<triangle_offsets> v;
	for(int i = 0; i<N; i++)
	{
		v.push_back(computeTriangeOffsets(width, fstep, max_radius,min_radius,num_channels));
	}
	return v;
}

/**
* \brief Compute normals in an roi on an xyza 3D point grid
* @param xyza		-- Array of 3D points in 2D float xyza matrix of 4 channels: x,y,z,d where d <= 0 means ignore the point
* @param xyzn		-- Return: Normals will be computed in this array of exact same size and type as xyzn
* @param roi		-- CvRect region of interest -- only calculate points here
* @param Trioffs	-- These are precomputed relative offsets for normal computation triangles computed in computeNTriangleOffsets()
* @param radius_sqr - ignore X,Y,Z points with X^2+Y^2+Z^2 greater than this distance
* @param quality	-- Threshold, if successful_normals_computed/num_triangles >= quality, accept the normal, esle not
* @param skip		-- Skip this many points between normal computations.  DEFAULT = 1 (NO SKIP).
* @return  Returns how many normals found. Zero indicates error/no surface to compute on
*/
int computeNormals(const CvMat *xyza, CvMat *xyzn, CvRect roi, vector<triangle_offsets> &Trioffs,
		float radius_sqr, float quality, int skip )
{
	//CHECK STUFF
	if(!xyza) {
		ROS_ERROR ("In computeNormals(), xyza is NULL");
		return -1;
	}
	if(CV_32FC4 != cvGetElemType(xyza)){
		ROS_ERROR ("Wrong type xyza matrix passed to computeNormals(). It should be %d but was %d",CV_32FC4, CV_MAT_TYPE(xyza->type));
		return -1;
	}
	if(!xyzn){
		ROS_ERROR ("In computeNormals(), xyzn is NULL");
		return -2;
	}
	if(CV_32FC4 != cvGetElemType(xyzn)){
		ROS_ERROR ("Wrong type xyzn matrix passed to computeNormals(). It should be %d but was %d",CV_32FC4, CV_MAT_TYPE(xyzn->type));
		return -2;
	}
	int rows = xyza->rows;
	int cols = xyza->cols;
	if(rows <= 0 || rows != xyzn->rows || cols <= 0 || cols != xyzn->cols){
		ROS_ERROR ("Rows and/or Cols missmatch between xyza and xyzn in computeNormals()");
		return -2;
	}
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
	if((int)Trioffs.size() == 0){
		ROS_WARN("In computeNormals(), Trioffs is empty (no triangles to compute normals in");
		return -4;
	}
	if(skip <= 0) skip = 1;

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
	int fnstep = xyzn->step/4;
	int fastep = xyza->step/4;
	for(y = ry; y<rh; y += skip)
	{
		nfptr = xyzn->data.fl + y*fnstep + rx*4; //Start out pointing to the correct position
		fptr = xyza->data.fl + y*fastep + rx*4; //Start out pointing to the correct position
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
* \brief res[] = A x B. Just a simple 3D cross (or outer) product
* SUPPORT FOR onePFH()
* res = A x B = <Ay*Bz - Az*By, Az*Bx - Ax*Bz, Ax*By - Ay*Bx>
* @param A		Float pointer to be 3 long array of floats  representing X,Y,Z
* @param B		STL vector of 3 floats representing X,Y,Z
* @param res	Results, stl vector of 3 floats containing the i,j,k cross product on exit
*/
void crossProd3D(const float *A, const vector<float> &B, vector<float> &res)
{
	res[0] = A[1]*B[2] - A[2]*B[1];
	res[1] = A[2]*B[0] - A[0]*B[2];
	res[2] = A[0]*B[1] - A[1]*B[0];
}

/**
* \brief res[] = A x B. Just a simple 3D cross (or outer) product
* SUPPORT FOR onePFH()
* res = A x B = <Ay*Bz - Az*By, Az*Bx - Ax*Bz, Ax*By - Ay*Bx>
* @param A		STL vector of 3 floats representing X,Y,Z
* @param B		Float pointer to be 3 long array of floats representing X,Y,Z
* @param res	Results, stl vector of 3 floats containing the i,j,k cross product on exit
*/
void crossProd3D(const vector<float> &A, const float *B, vector<float> &res)
{
	res[0] = A[1]*B[2] - A[2]*B[1];
	res[1] = A[2]*B[0] - A[0]*B[2];
	res[2] = A[0]*B[1] - A[1]*B[0];
}

/**
* \brief res[] = A x B. Just a simple 3D cross (or outer) product
* SUPPORT FOR onePFH()
* res = A x B = <Ay*Bz - Az*By, Az*Bx - Ax*Bz, Ax*By - Ay*Bx>
* @param A		STL vector of 3 floats representing X,Y,Z
* @param B		STL vector of 3 floats representing X,Y,Z
* @param res	Results, stl vector of 3 floats containing the i,j,k cross product on exit
*/
void crossProd3D(const vector<float> &A, const vector<float> &B, vector<float> &res)
{
	res[0] = A[1]*B[2] - A[2]*B[1];
	res[1] = A[2]*B[0] - A[0]*B[2];
	res[2] = A[0]*B[1] - A[1]*B[0];
}

/**
* \brief Compute the L2 norm of src into dst
* SUPPORT FOR onePFH()
* @param src	pointer to 3 long float vector
* @param dst   Result, pointer to 3 long float vector
*/
void normalize(float *src, float *dst){
	float x,y,z;
	x = *src;
	y = *(src+1);
	z = *(src+2);
	float rlen2 = 1.0/sqrt(x*x+y*y+z*z);
	*dst = x*rlen2;
	*(dst + 1) = y*rlen2;
	*(dst + 2) = z*rlen2;
	*(dst + 3) = *(src + 3); //This is just the A value of XYZA.  0=> no actual data
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////  2nd tier support functions ///////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
* \brief This function returns the offsets within a 2D X,Y,Z,D Mat that form a triangle around a point
* SUPPORTING FUNCTION FOR computeNTriangleOffsets(...)
* @param width        -- Width of the 2D matrix of float values
* @param fstep		   -- Stride or step of the CvMat in floats (or CvMat.step/4)
* @param max_radius   -- Maximum radius of the triangle on the grid (must be > min_radius)
* @param min_radius   -- Minimum radius of the triangle on the grid (must be > 0.0)
* @param num_channels -- number of channels in the 2D matrix of floats.  Def: 4 for X,Y,Z,Disparity
* @return  Returns the offsets in the structure triangle_offsets
*/
triangle_offsets computeTriangeOffsets(int width, int fstep, float max_radius, float min_radius ,
		int num_channels )
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
	Ang[1] = Ang[0]+120.0;   //Step the other angles to be always 120 degrees apart
	Ang[2] = Ang[1]+120.0;
	cvInitMatHeader(&matM,1,3,CV_32FC1,Mag);
	cvInitMatHeader(&matA,1,3,CV_32FC1,Ang);
	cvInitMatHeader(&matX,1,3,CV_32FC1,X);
	cvInitMatHeader(&matY,1,3,CV_32FC1,Y);
	//COMPUTE VECTOR END POINTS
	cvPolarToCart(&matM, &matA, &matX, &matY,1);  //Fills X[]s and Y[]s with Cartisian offsets
	//TURN THEM INTO OFFSETS FROM A POINT ON THE GRID
	float roundX,roundY;
	if(Y[0] < 0) roundY = -0.5; else roundY = 0.5;
	if(X[0] < 0) roundX = -0.5; else roundX = 0.5;
	t_o.v0 = (int)(Y[0]+ roundY)*fstep + (int)(X[0]+roundX)*num_channels;
	if(Y[1] < 0) roundY = -0.5; else roundY = 0.5;
	if(X[1] < 0) roundX = -0.5; else roundX = 0.5;
	t_o.v1 = (int)(Y[1]+ roundY)*fstep + (int)(X[1]+roundX)*num_channels;
	if(Y[2] < 0) roundY = -0.5; else roundY = 0.5;
	if(X[2] < 0) roundX = -0.5; else roundX = 0.5;
	t_o.v2 = (int)(Y[2]+ roundY)*fstep + (int)(X[2]+roundX)*num_channels;
	return t_o;
}


//!!!! IN THE ROUTINE BELOW, CHOOSING THE MEDIAN NORMAL SHOULD BE CHANGED TO THE AVERAGE NORMAL FOR SSE EFFICIENCY ... SO SORRY !!!!
/**
* \brief  Robust normal computation at point p in the X,Y,Z,D matrix
* SUPPORT FOR FUNCTION computeNormals(...)
* @param xyzd                  X,Y,Z,D matrix, also known as xyza where "A" is anything, mainly to zero or allow using that 3D point
* @param p                     point (x,y) in xyzd that we're computing robust normal at
* @param v						precomputed list of triangle offsets to use at point p
* @param radius_sqr   			ignore X,Y,Z points with X^2+Y^2+Z^2 greater than this distance
* @param normal_pt3d           RETURN X,Y,Z location of the normal at p
* @param normals		        RETURN normals in A,B,C form
* @param normal_angles			RETURN normals in angular (0-360 degree) form: (rot around Z, rotation from X,Y plane)
* @param quality				RETURN 1.0 = perfect quality: successful_normals_computed/num_triangles(that is, v.size())
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
	int fstep = xyzd->step/4;
	if((col_offset >= cols)||(row_offset >= rows)||(col_offset < 0)||(row_offset < 0)) return -2; //Invalid dimensions
	float *dat_start = xyzd->data.fl;         //
	float *dat_end = dat_start + (rows - 1)*fstep + cols*4; //End of data range
	float *dat = dat_start + row_offset*fstep + col_offset*4; //Point to p (x,y) offset into xyzd
	int num_triangles = (int)v.size();
	if(num_triangles <= 0) return -3; //No triangles
	normal_pt3d.clear();
	normals.clear();
	normal_angles.clear();
	//COMPUTE THE TRIANGLES
	float Xa,Xb,Xc,Ya,Yb,Yc,Za,Zb,Zc,Da,Db,Dc; //Values we will need
	if(*(dat+3) < 0.001) return -4; //No center point
	float Xpt = *dat,Ypt = *(dat+1),Zpt = *(dat+2); //Record the base position of the normal world coordinates
	float Xd,Yd,Zd; //Distances from center point (Xpt,Ypt,Zpt)
	normal_pt3d.push_back(Xpt); normal_pt3d.push_back(Ypt); normal_pt3d.push_back(Zpt); //Enter normal point location
	for(int i = 0; i<num_triangles; ++i)
	{
		//VERTEX 0    the v[]'s are integer offsets relative to a float pointer in xyzd
		if((dat_start > dat+v[i].v0) || (dat+v[i].v0 + 3)>=dat_end) continue; //Don't allow stepping out of xyzd
		Xa = *(dat + v[i].v0);
		Ya = *(dat + v[i].v0 + 1);
		Za = *(dat + v[i].v0 + 2);
		Xd = Xa-Xpt; //if(Xd < 0) Xd = -Xd;
		Yd = Ya-Ypt; //if(Yd < 0) Yd = -Yd;
		Zd = Za-Zpt; //if(Zd < 0) Zd = -Zd;
		if(Xd*Xd+Yd*Yd+Zd*Zd > radius_sqr) continue; //Point is too far away
		Da = *(dat + v[i].v0 + 3);
		if(Da < 0.0000001) continue; //Invalid point
		//VERTEX 1
		if((dat_start > dat+v[i].v1) || (dat+v[i].v1 + 3)>=dat_end) continue; //Don't allow stepping out of xyzd
		Xb = *(dat + v[i].v1);
		Yb = *(dat + v[i].v1 + 1);
		Zb = *(dat + v[i].v1 + 2);
		Xd = Xb-Xpt; //if(Xd < 0) Xd = -Xd;
		Yd = Yb-Ypt; //if(Yd < 0) Yd = -Yd;
		Zd = Zb-Zpt; //if(Zd < 0) Zd = -Zd;
		if(Xd*Xd+Yd*Yd+Zd*Zd > radius_sqr) continue;
		Db = *(dat + v[i].v1 + 3);
		if(Db < 0.0000001) continue; //Invalid point
		//VERTEX 2
		if((dat_start > dat+v[i].v2) || (dat+v[i].v2 + 3)>=dat_end) continue; //Don't allow stepping out of xyzd
		Xc = *(dat + v[i].v2);
		Yc = *(dat + v[i].v2 + 1);
		Zc = *(dat + v[i].v2 + 2);
		Xd = Xc-Xpt; //if(Xd < 0) Xd = -Xd;
		Yd = Yc-Ypt; //if(Yd < 0) Yd = -Yd;
		Zd = Zc-Zpt; //if(Zd < 0) Zd = -Zd;
		if(Xd*Xd+Yd*Yd+Zd*Zd > radius_sqr) continue;
		Dc = *(dat + v[i].v2 + 3);
		if(Dc < 0.0000001) continue; //Invalid point
		//COMPUTE THE NORMAL
		vector<float> n;
		float A = (((Yb-Ya)*(Zc-Za)) - ((Yc-Ya)*(Zb-Za))); //A (x coef)
		float B = (((Xc-Xa)*(Zb-Za)) - ((Xb-Xa)*(Zc-Za))); //B (y coef)
		float C = (((Xb-Xa)*(Yc-Ya)) - ((Xc-Xa)*(Yb-Ya))); //C (z coef)
		float ABC = sqrt(A*A+B*B+C*C); //Normalize the normal
		if(ABC == 0.0){ n.push_back(0.0);n.push_back(0.0);n.push_back(0.0);}
		else {n.push_back(A/ABC); n.push_back(B/ABC); n.push_back(C/ABC);  }
		normals.push_back(n);
		//COMPUTE THE ANGLE FORM OF THE NORMAL IF ASKED
		vector<float> a;
		a.push_back(cvFastArctan(B,A));       // Angles 0-360 for Y/X  -- rotation around the z axis
		float rxy = cvSqrt(A*A + B*B);        // sqrt of X^2 + Y^2
		a.push_back(cvFastArctan( rxy, C ));  // Angle of from X,Y plane arctan(sqrt(X^2+Y^2)/Z)
		normal_angles.push_back(a);
	}
	int median_index = -1;
	vector<float> median = findMedianND(normal_angles,median_index);
	quality = (float)(normals.size())/(float)num_triangles;
	return median_index;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////  3RD tier support functions ///////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
* \brief Finds the median of a set of multi-dimensional points v.
* SUPPORT FUNCTION FOR computeNormalAtPt() WHICH SUPPORTS computeNormals(...)
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
	if(avg_radius > 99999999.0) return median;  //Oh well, center of mass is best we can do ...
	avg_radius += std_radius;
	//FIND THE MEDIAN BY, BASICALLY, MEANSHIFT ... MODIFIED BY JUMPING TO THE NEAREST POINT EACH LOOP
	float dist;
	for(int i = 0; i<7; ++i) //Stop when distance changed is low or after 7 interations
	{
		markClosePoints(v, median, avg_radius, b);
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
	median = findClosestPointToPinV(v, median, b, median_index);
	return median;
}

/**
* \brief Find all points in v within distance of p and mark such in b
* SUPPORT FUNCTION FOR findMedianND() WHICH SUPPORTS computeNormalAtPt() WHICH SUPPORTS computeNormals(...)
* @param v         List of points
* @param p         Point to determine distance from
* @param max_dist  Maximum (city block) distance |v[i] - p| allowed
* @param b         RETURN: boolean mark points valid
*/
void markClosePoints(const vector<vector<float> > &v, const vector<float> &p, const float max_dist, vector<bool> &b)
{
	b.clear();
	int M = (int)v.size();
	if(M < 1) return;
	int N = (int)p.size();
	if(N < 1) return;
	b.resize(M,false);
	float dist;
	for(int i=0; i<M; ++i)
	{
		dist = 0.0;
		for(int j=0; j<N; ++j)
		{
			float d = v[i][j] - p[j];
			if(d<0.0) d = -d;
			dist += d; //City block distance
		}
		if(dist <= max_dist)
			b[i] = true;
	}
	return;
}

/**
* \brief Return the mean and std city block distance in N dimensions of point p to a set of ND points v
* SUPPORT FUNCTION FOR findMedianND() WHICH SUPPORTS computeNormalAtPt() WHICH SUPPORTS computeNormals(...)
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
* \brief This function returns the "N" dimensional center of mass of a list of ND points
* SUPPORT FUNCTION FOR findMedianND() WHICH SUPPORTS computeNormalAtPt() WHICH SUPPORTS computeNormals(...)
* @param v   vector<vector<float> > M points each of dimension N (each sub-vector must be this length)
* @param b   pointer to boolean vector that marks which points are valid.  Default is NULL
* @return    returns the ND center of mass point.  Error if this returned vector size is 0
*/
vector<float>  centerOfMassND(vector<vector<float>  > &v, vector<bool> *b)
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
* \brief Return the closest point in v to p skipping things marked by b
* SUPPORTING FUNCTION FOR findMedianND(...) WHICH SUPPORTS computeNormalAtPt() WHICH SUPPORTS computeNormals(...)
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
	closest_point.resize(D,0.0);
	float point_dist, min_dist = 9999999999.0, dist = 0.0;

	for(int i=0; i<M; ++i)
	{
		if(b[i]) //Skip far points
		{
			point_dist = 0.0;
			for(int j=0; j<D; ++j)
			{
				dist = v[i][j] - p[j];
				if(dist < 0.0) dist = - dist;
				point_dist += dist;
			}
			if(point_dist <= min_dist)
			{
				index = i;
				min_dist = point_dist;
				closest_point = v[i];
			}
		}
	}
	return closest_point;
}


//**********************^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*************************//
//**********************FUNCTIONS SPECIFIC TO THE WPFH FEATURES*************************//
//**********************^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*************************//



