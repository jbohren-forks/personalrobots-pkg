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
#include "robot_msgs/Vector3.h"
#include "robot_msgs/PointStamped.h"
#include "robot_msgs/Door.h"
//#include "robot_msgs/VisualizationMarker.h"

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


typedef struct _triangle_offsets
{
    int  v0;
    int  v1;
    int  v2;
}
triangle_offsets;


class StereoPointCloudProcessing{

public:

	CvRNG rng_state;
	StereoPointCloudProcessing()
	{
		rng_state = cvRNG();
	}



	int find2dRoiFrom3d(CvMat *xyzd, const CvPoint &p, float radius_sqr, CvRect &roi)
	{
		//SETUP AND CHECKUP
		if(!xyzd) return -1;             //No data
		if(radius_sqr <= 0.0) return -2; //Neg radius
		int r = p.y;
		int c = p.x;
		int rows = xyzd->rows;
		int cols = xyzd->cols;
		if((r < 0)||(r >= rows)) return -3;// p.y is out of bounds
		if((c < 0)||(c >= cols)) return -4;// p.x is out of bounds
		//FIND EXTENTS OF ROI
//		int up = r;  //Find extents of possible exploration
//		int down = rows - r - 1;
//		int left = c;
//		int right = cols - c - 1;
		int widthstep = cols*4;
		float *fbase = xyzd->data.fl + r*widthstep + c*4; //pointns to (c,r)
		float *fptr;
		if(*(fbase+3) == 0.0) return -5; //We don't have a central point
		float Xp = *fbase, Yp = *(fbase + 1), Zp = *(fbase + 2),Xd,Yd,Zd;
		int x,y;
		//Find how far up
		for(y = r,fptr = fbase - widthstep; y>0; --y, fptr -= widthstep) {
			if(*(fptr+3) == 0.0) continue;  //Skip over points that we don't have
			Xd = (*fptr) - Xp; Yd = (*(fptr+1)) - Yp; Zd = (*(fptr+2)) - Zp;
			if((Xd*Xd + Yd*Yd + Zd*Zd) > radius_sqr) break; //We found a point too far away
		}
		int up = y;
		//Find out how far down
		for(y=r, fptr=fbase + widthstep; y < rows - 1; ++y, fptr += widthstep){
			if(*(fptr+3) == 0.0) continue;  //Skip over points that we don't have
			Xd = (*fptr) - Xp; Yd = (*(fptr+1)) - Yp; Zd = (*(fptr+2)) - Zp;
			if((Xd*Xd + Yd*Yd + Zd*Zd) > radius_sqr) break; //We found a point too far away
		}
		int down = y;
		//Find out how far left
		for(x = c,fptr = fbase - 4; x>0; --x, fptr -= 4) {
			if(*(fptr+3) == 0.0) continue;  //Skip over points that we don't have
			Xd = (*fptr) - Xp; Yd = (*(fptr+1)) - Yp; Zd = (*(fptr+2)) - Zp;
			if((Xd*Xd + Yd*Yd + Zd*Zd) > radius_sqr) break; //We found a point too far away
		}
		int left = x;
		//Find out how far right
		for(x=c, fptr=fbase + 4; x < cols - 1; ++x, fptr += 4){
			if(*(fptr+3) == 0.0) continue;  //Skip over points that we don't have
			Xd = (*fptr) - Xp; Yd = (*(fptr+1)) - Yp; Zd = (*(fptr+2)) - Zp;
			if((Xd*Xd + Yd*Yd + Zd*Zd) > radius_sqr) break; //We found a point too far away
		}
		int right = x;
		//Record and out
		roi.x = left;
		roi.width = right - left + 1;
		roi.y  = up;
		roi.height = down - up + 1;
		return 0;
	}

	int compute3dOffsetsFeature(CvMat *xyzd, vector<triangle_offsets> &v, CvPoint &p, float radius3D)
	{

		return 0;
	}


	void makePlaneData(CvMat *xyzd, float A, float B, float C, float Xa, float Ya, float Za)
	{
		float X,Y,Z;
		if(!xyzd) return;
		if(C == 0.0) return;
		int rows = xyzd->rows;
		int cols = xyzd->cols;
//		printf("r,c = (%d, %d)\n",rows,cols);
		for(int y=0; y<rows; ++y){
			for(int x= 0; x<cols; ++x){
				X = (float)x;
				Y = (float)y;
				Z = Za - ((A*(X-Xa)+B*(Y-Ya))/C); //Fill in depth that satisfies the equation of the plane

//				if((!(y%48))&&(!(x%128)))
//					printf("(%d, %d, %f\n",x,y,Z);
 //				Z = 0.9;
 				cvSet2D( xyzd, y, x, cvScalar(X,Y,Z,1.0) ); //Stick it into the array
			}
		}
	}



	/**
	 * \brief  Robust normal computation at point p in the X,Y,Z,D matrix
	 * @param xyzd                  X,Y,Z,D matrix
	 * @param p                     point (x,y) in xyzd that we're computing robust normal at
	 * @param v						precomputed list of triangle offsets to use at point p
	 * @param radius_sqr   			ignore X,Y,Z points with X^2+Y^2+Z^2 greater than this distance
	 * @param normal_pt3d           RETURN X,Y,Z location of the normal at p
	 * @param normals		        RETURN normals in A,B,C form
	 * @param normal_angles			RETURN normals in angular (0-360 degree) form: (rot around Z, rotation from X,Y plane)
	 * @return                      Index of median normal, OR: -1,median computation failure, -2,invalid dimensions, -3,no trainges, -4,no matrix
	 */
	int computeNormals(const CvMat *xyzd, const CvPoint &p, vector<triangle_offsets> &v, float radius_sqr,
			vector<float> &normal_pt3d, vector<vector<float> > &normals, vector<vector<float> > &normal_angles)
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
		float *dat = dat_start + rows*row_offset*4 + col_offset*4; //Point to p (x,y) offset into xyzd
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
//		printf("In computeNormals\n");
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
		return median_index;
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
		if(M < 1) return median; //No points to calculate
		vector<bool> b(M,false);
		float std_radius = 0.0;
		vector<float> p(med_dim, 0.0);
		float avg_radius = 	avgCityDistanceND(v, median, std_radius);
//		printf("avg_radius = %f, std_radius=%f\n",avg_radius,std_radius);
		if(avg_radius > 99999999.0) return median;  //Oh well, center of mass is best we can do ...
		avg_radius += std_radius;
		//FIND THE MEDIAN BY, BASICALLY, MEANSHIFT ... MODIFIED BY JUMPING TOO THE NEAREST POINT EACH LOOP
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
		int M = (int)v.size();
		int D = (int)p.size();
		vector<float> closest_point;
		if((M<1)||(D<1)) return closest_point;
		if(D != (int)v[0].size()) return closest_point;
//		printf("findClosestPoint: p(%f, %f) M=%d, D=%d, v[0].size=%d\n",p[0],p[1],M,D,(int)v[0].size());
		closest_point.resize(D,0.0);
		float point_dist, min_dist = 9999999999.0, dist = 0.0;
		index = -1;
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
                  if((x<0)||(x>=cols)){
                      printf("x(%d)=%d out of bounds(%d)\n",i,x,cols);
                      x = rows - 1;
                  }
                  int y   = (int)(cloud.chan[ychan].vals[i]);
                  if((y<0)||(y>=rows)){
                      printf("y(%d)=%d out of bounds(%d)\n",i,y,rows);
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
					printf("x(%d)=%d out of bounds(%d)\n",i,x,width);
					x = width - 1;
				}
				int y   = (int)(cloud.chan[ychan].vals[i]);
				if((y<0)||(y>=height)){
					printf("y(%d)=%d out of bounds(%d)\n",i,y,height);
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

void on_edges_low(int);
void on_edges_high(int);
void on_depth_near(int);
void on_depth_far(int);








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
	IplImage* color_depth;

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
    :ros::Node("stereo_view"), left(NULL), right(NULL), disp(NULL), disp_clone(NULL), color_depth(NULL), sync(this, &RecognitionLambertian::image_cb_all, ros::Duration().fromSec(0.1), &RecognitionLambertian::image_cb_timeout)
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
        depth_near = 3;
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
                	printf("x(%d)=%d out of bounds(%d)\n",i,x,width);
                	x = width - 1;
                }
                int y   = (int)(cloud.chan[ychan].vals[i]);
                if((y<0)||(y>=height)){
                	printf("y(%d)=%d out of bounds(%d)\n",i,y,height);
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
//				if(!(y%96)&&!(x%128))
//					printf("z = %f",z);
				//Put this value into the image
				if (x >= rect.x && x < rect.x + rect.width && y >= rect.y && y
						< rect.y + rect.height) {
					z -= start_depth;
					if (z < 0.0)
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
//					if(!(y%96)&&!(x%128))
//						printf("colorDepthImgFromMat: z=%f at xy (%d,%d) = [%d, %d, %d]\n",z,x,y,(int)B, (int)G, (int)R);
//										cvSet2D(I, y, x, cvScalar(B, G, R)); //Slow way
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

//            disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
            disp = cvCloneImage(dbridge.toIpl());
//            cvCvtScale(dbridge.toIpl(), disp, 4.0 / dispinfo.dpp);
        }

        cloud = cloud_fetch;

//        images_ready.notify_all();
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

    void displayColorDepthImage()
    {
    	CvRect R = cvRect(0,0,color_depth->width, color_depth->height);
     	float Dn = depth_near/10.0;
    	float Df = depth_far/10.0;
    	CvMat *M = cvCreateMat( color_depth->height, color_depth->width, CV_32FC4 );
    	StereoPointCloudProcessing spc;
    	spc.pointCloud2PointMat(M, R, cloud);
//    	spc.makePlaneData(M, 0.3, 0.4, 500.0, 320, 240, 1.0); //Visualize this
    	colorDepthImageFromMat(color_depth,M,R,Dn,Df);

		vector<triangle_offsets> Trioffs = spc.computeNTriangleOffsets(9, 640, 10, 4);
		vector<float> normal_pt3d;
		vector<vector<float> > normals;
		vector<vector<float> > normal_angles;
		vector<Vector3 > coef;

		PointCloud points;
		points.header.frame_id = cloud.header.frame_id;
		points.header.stamp = cloud.header.stamp;


		printf("triangle offsets: %d\n",Trioffs.size());

		Vector3 v3;
		Point32 p3;


		for (int y=0;y<480;y+=15) {
			for (int x=0;x<640;x+=15) {

				int nindex =  spc.computeNormals(M, cvPoint(x,y), Trioffs, 400.0, normal_pt3d, normals, normal_angles);

//				printf("nindex: %d\n", nindex);
				if (nindex>=0) {
					p3.x = normal_pt3d[0];
					p3.y = normal_pt3d[1];
					p3.z = normal_pt3d[2];

					v3.x = normals[nindex][0];
					v3.y = normals[nindex][1];
					v3.z = normals[nindex][2];
					points.pts.push_back(p3);
					coef.push_back(v3);

//					printf("\n------------------------------------------------------------------------\n");
//					printf("sizes (N=9) TriOffsize=%d[%d, %d, %d] [[%d, %d]]\n",Trioffs.size(),normal_pt3d.size(),normals.size(),normal_angles.size(),
//							normals[0].size(),normal_angles[0].size());
//					printf("X,Y,Z=(%f, %f, %f)\n",normal_pt3d[0],normal_pt3d[1], normal_pt3d[2]);
//					printf("nindex=%d, A=%f, B=%f, C=%f",nindex, normals[nindex][0],normals[nindex][1],normals[nindex][2]);
//					printf("Angles = (%f, %f)\n",normal_angles[nindex][0],normal_angles[nindex][1]);
//					printf("------------------------------------------------------------------------\n");

				}
			}
		}


		publishNormals(this, points, coef, -0.04);

//    	vector<float> p1(2);
//    	vector<vector<float> > ps(8,p1);
//    	ps[0][0] = 1;
//    	ps[0][1] = 0;
//    	ps[1][0] = 0;
//    	ps[1][1] = 1;
//    	ps[2][0] = 0.5;
//    	ps[2][1] = 0.5;
//       	ps[3][0] = 3.2;
//		ps[3][1] = 2.7;
//		ps[4][0] = 3.4;
//		ps[4][1] = 3.0;
//		ps[5][0] = 3.1;
//		ps[5][1] = 2.6;
//		ps[6][0] = 4.4;
//		ps[6][1] = 4.0;
//		ps[7][0] = 4.1;
//		ps[7][1] = 4.6;
//    	int median_index = 1000;
//    	vector<float> med = spc.findMedianND(ps,median_index);
//    	printf("ps[1][1]=%f, Median index = %d\n",ps[1][1],median_index);
//    	printf("size of return vector = %d\n",(int)(med.size()));
//    	printf("med (%f, %f)\n",med[0],med[1]);


//    	int ret_toM = spc.pointCloud2PointMat(M, R, cloud);
//    	int ret_toI = colorDepthImageFromMat(color_depth,M,R,Dn,Df);
//    	printf("ret_toM=%d, ret_toI=%d\n",ret_toM,ret_toI);
    	cvReleaseMat( &M );
       	spc.computeTriangeOffsets(100,5,2);
       	spc.computeTriangeOffsets(100,2,1);
       	spc.computeTriangeOffsets(100,10,5);

//    	colorDepthImage(color_depth,R,Dn,Df);
//    	printf("colorDepthImage ret = %d",r);
    	cvShowImage("color_depth",color_depth);
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

