/*********************************************************************
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

// Author: Min Sun

#include <iostream>
#include <fstream>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "cvaux.h"
#include <math.h>
#include "RandomForest.h"
#include "point_cloud_mapping/cloud_io.h"
#include "point_cloud_mapping/kdtree/kdtree_ann.h"
#include "descriptors_3d/all_descriptors.h"
#include "write.h"

using namespace cv;
using namespace std;

class shape{
public:
    shape(): NRadius(0.02){}
    shape( double NRadius_ ):
        NRadius( NRadius_){ }
    ~shape(){}

    // methods
    // for detection task
    void compute( const sensor_msgs::PointCloud& pcd_pc, Vector< geometry_msgs::Point32>& interest_pts, 
	cv::Vector<cv::Vector<float> >& all_descriptor_results);

    // properties
    double NRadius;

    // descriptor 3d stuffs
    SpectralAnalysis::SpectralAnalysis sa;
    ShapeSpectral::ShapeSpectral shape_spectral;
    sensor_msgs::PointCloud pcd_pc;
    Vector< geometry_msgs::Point32> interest_pts;
};
