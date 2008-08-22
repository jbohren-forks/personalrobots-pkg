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

#ifndef _objectdetector_h_
#define _objectdetector_h_

/**
   @mainpage 
   @b A simple object detector that cleans up point clouds
   and then segments into connected components. See ObjectDetector
   class for details.
 **/

#include <vector>

class SmartScan;

namespace grasp_module {

class Object;

/*!
  A simple detector for objects inside point clouds.

  The main function of this class is \a getObjects(...) which takes in
  a point cloud and returns the Objects found inside. However, this
  functionality needs a set of parameters. The empty constructor will
  initialize all these parameters to some default value, but it is
  recommended to study the parameters and decide what are the optimal
  values for the desired application.

  The \a getObjects(...) function also makes a series of assumptions
  about that is in the point cloud. In this sense, this is a very
  naive object detector, more of a stub for better things to come. See
  \a getObjects(...) function for all details.
 */
class ObjectDetector {
 private:
	//! Parameter for segmentation operation
	float mGrazThresh;
	//! Parameter for segmentation operation
	float mGrazRad;
	//! Parameter for segmentation operation
	int mGrazNbrs;
	//! Parameter for segmentation operation
	float mRemThresh;
	//! Parameter for segmentation operation
	float mFitThresh;
	//! Parameter for segmentation operation
	float mConnThresh;
	//! Parameter for segmentation operation
	int mSizeThresh;
	
 public:
	//! Constructor. Will initialize all unspecified parameters to a default value.
	ObjectDetector();
	//! Core function of this class, segments a point cloud into objects.
	std::vector<Object*> *getObjects(SmartScan *scan);

	//! Set parameters for removing grazing points
	void setParamGrazing(float thresh, float rad, int nbrs);
	//! Set paramentes for plane detection and removal
	void setParamPlane(float thresh, float fit);
	//! Set paramenters for connected components
	void setParamComponents(float thresh, int points);
};

} //namespace grasp_module

#endif
