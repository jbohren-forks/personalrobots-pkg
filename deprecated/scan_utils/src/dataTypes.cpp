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

#include "dataTypes.h"

namespace scan_utils {

float norm(const std_msgs::Point32 &f)
{
	return sqrt( f.x*f.x + f.y*f.y + f.z*f.z);
}

std_msgs::Point32 normalize(const std_msgs::Point32 &f)
{
	float n = scan_utils::norm(f);
	std_msgs::Point32 p;
	p.x = f.x / n;
	p.y = f.y / n;
	p.z = f.z / n;
	return p;
}

float dot(const std_msgs::Point32 &f1, const std_msgs::Point32 &f2)
{
	return f1.x*f2.x + f1.y*f2.y + f1.z*f2.z;
}

std_msgs::Point32 cross(const std_msgs::Point32 &f1, const std_msgs::Point32 &f2)
{
	std_msgs::Point32 c;
	c.x = f1.y * f2.z - f1.z * f2.y;
	c.y = f1.z * f2.x - f1.x * f2.z;
	c.z = f1.x * f2.y - f1.y * f2.x;
	return c;
}

static int mask1d[5] = {0,1,2,1,0};
	const Grid1D Grid1D::MASK(5,mask1d,true);

Grid1D::Grid1D(int d1)
{
	assert(d1>0);
	mD1 = d1;
	mData = new int[d1];
	for (int i=0; i<d1; i++) {
		mData[i] = 0;
	}
}

Grid1D::Grid1D(int d1, int *data, bool copy)
{
	assert(d1>0);
	mD1 = d1;
	if (!copy) {
		mData = data;
	} else {
		mData = new int[d1];
		for (int i=0; i<d1; i++) {
			mData[i] = data[i];
		}
	}
}

Grid1D::~Grid1D()
{
	delete [] mData;
}

void Grid1D::addGrid(int n1, const Grid1D *g)
{
	int start = n1 - g->getSize() / 2;

	for (int k = 0; k< g->getSize(); k++){
		int a = start + k;
		if (a<0 || a>=mD1) continue;
		addElement( a, g->getElement(k) );
	}
}

void Grid1D::getMax(int &max, int &n1)
{
	max = 0; n1=-1;
	for (int i=0; i<mD1; i++){
		if (getElement(i) > max) {
			max = getElement(i);
			n1=i;
		}
	}
}

static int mask2d[25] = { 0,1,1,1,0,
			  1,1,2,1,1,
			  1,2,3,2,1,
			  1,1,2,1,1,
			  0,1,1,1,0 };
	const Grid2D Grid2D::MASK(5,5,mask2d,true);

Grid2D::Grid2D(int d1, int d2)
{
	assert(d1>0 && d2>0);
	mD1 = d1; mD2 = d2;
	mData = new int[d1*d2];
	for (int i=0; i<d1; i++) {
		for(int j=0; j<d2; j++)
			mData[i*d2 + j] = 0;
	}
}

Grid2D::Grid2D(int d1, int d2, int *data,bool copy)
{
	assert(d1>0 && d2>0);
	mD1 = d1; mD2 = d2;
	if (!copy) {
		mData = data;
	} else {
		mData = new int[d1*d2];
		for (int i=0; i<d1; i++) {
			for(int j=0; j<d2; j++) {
				mData[i*d2 + j] = data[i*d2 + j];
			}
		}
	}
}

Grid2D::~Grid2D()
{
	delete [] mData;
}

void Grid2D::addGrid(int n1, int n2, const Grid2D *g)
{
	int gd1, gd2;
	g->getSize(gd1, gd2);

	int s1 = n1 - gd1 / 2;
	int s2 = n2 - gd2 / 2;

	for (int k1 = 0; k1<gd1; k1++){
		int a1 = s1 + k1;
		if (a1<0 || a1>=mD1) continue;
		for (int k2 = 0; k2<gd2; k2++) {
			int a2 = s2 + k2;
			if (a2<0 || a2>=mD2) continue;
			addElement(a1,a2,g->getElement(k1,k2));
		}
	}
}

void Grid2D::getMax(int &max, int &n1, int &n2)
{
	max = 0; n1 = n2 = -1;
	for (int i1=0; i1<mD1; i1++){
		for (int i2=0; i2<mD2; i2++) {
			if (getElement(i1,i2) > max) {
				max = getElement(i1,i2);
				n1=i1; n2=i2;
			}
		}
	}
}
       
} //namespace scan_utils
