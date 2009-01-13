/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Matei Ciocarlie */

#ifndef _datatypes_h_
#define _datatypes_h_

#include <std_msgs/Point32.h> //ROS native format for a point cloud
#include <vector>
#include "assert.h"

namespace scan_utils{

/*!  Convenience class for storing a Triangle as a list of three
  vertices stored each as a Point32. Will probably be replaced
  at some point by a more elaborate ROS native data type.

  Defines copy contructor and assignment operator, thus allowing
  instances of this class to be placed in STD containers.
 */
class Triangle{
 public:
	std_msgs::Point32 p1,p2,p3;
	
        Triangle() : p1(), p2(), p3(){}
        Triangle(const Triangle &t) : p1(t.p1), p2(t.p2), p3(t.p3) {}
	Triangle& operator=(const Triangle &t) {p1=t.p1; p2=t.p2; p3=t.p3; return *this;}

	Triangle(double c[]) {
		p1.x = c[0]; p1.y = c[1]; p1.z = c[2];
		p2.x = c[3]; p2.y = c[4]; p2.z = c[5];
		p3.x = c[6]; p3.y = c[7]; p3.z = c[8];
	}
	Triangle(double c1[], double c2[], double c3[]){
		p1.x = c1[0]; p1.y = c1[1]; p1.z = c1[2];
		p2.x = c2[0]; p2.y = c2[1]; p2.z = c2[2];
		p3.x = c3[0]; p3.y = c3[1]; p3.z = c3[2];
	}
	Triangle(double p1x, double p1y, double p1z,
		 double p2x, double p2y, double p2z,
		 double p3x, double p3y, double p3z){
		p1.x = p1x; p1.y = p1y; p1.z = p1z;
		p2.x = p2x; p2.y = p2y; p2.z = p2z;
		p3.x = p3x; p3.y = p3y; p3.z = p3z;
	}
};

/*! A couple of utility functions which will probably be replaced by a
    ROS-native math library 
*/

 float norm(const std_msgs::Point32 &f);
 std_msgs::Point32 normalize(const std_msgs::Point32 &f);
 float dot(const std_msgs::Point32 &f1, const std_msgs::Point32 &f2);
 std_msgs::Point32 cross(const std_msgs::Point32 &f1, const std_msgs::Point32 &f2);

/*! A 1D histogram.
 */
class Grid1D 
{
 private:
	int mD1;
	int *mData;
 public:
        Grid1D(int d1);
	Grid1D(int d1, int *data, bool copy = false);
	~Grid1D();

	int getSize () const {return mD1;}

	int getElement(int n1) const {assert(n1<mD1); return mData[n1];}
	void setElement(int n1, int val){assert(n1>=0 && n1<mD1); mData[n1]=val;}
	void addElement(int n1, int val){assert(n1>=0 && n1<mD1); mData[n1]+=val;}
	void addGrid(int n1, const Grid1D *g);

	void getMax(int &max, int &n1);

	static const Grid1D MASK;
};

/*! A 2D histogram
 */
class Grid2D 
{
 private:
	int mD1,mD2;
	int *mData;
 public:
        Grid2D(int d1, int d2);
	Grid2D(int d1, int d2, int *data, bool copy = false);
	~Grid2D();

	void getSize(int &d1, int &d2) const {d1 = mD1; d2 = mD2;}

	int getElement(int n1, int n2) const {assert(n1<mD1 && n2<mD2); return mData[n1*mD2 + n2];}
	void setElement(int n1, int n2, int val){assert(n2<mD2 && n1<mD1); mData[n1*mD2 + n2]=val;}
	void addElement(int n1, int n2, int val){assert(n2<mD2 && n1<mD1); mData[n1*mD2 + n2]+=val;}
	void addGrid(int n1, int n2, const Grid2D *g);

	void getMax(int &max, int &n1, int &n2);

	static const Grid2D MASK;
};

} //namespace scan_utils
#endif
