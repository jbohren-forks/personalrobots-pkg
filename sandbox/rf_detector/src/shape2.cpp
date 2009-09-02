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

#include "shape2.h"
#include "write.h"

using namespace cv;
using namespace std;

void shape::compute( const sensor_msgs::PointCloud& pcd_pc, Vector< geometry_msgs::Point32>& interest_pts, 
	cv::Vector<cv::Vector<float> >& all_descriptor_results)
{
    unsigned int num_interest_pts = interest_pts.size();
    cout << "num_interest_pts" << num_interest_pts <<endl;
    all_descriptor_results.resize(num_interest_pts);
    cloud_kdtree::KdTreeANN pcd_pc_kdtree( pcd_pc);

    Vector<const geometry_msgs::Point32*> interest_pts_tmp(num_interest_pts);
    for (size_t i = 0 ; i < num_interest_pts ; i++)
    {
        interest_pts_tmp[i] = &(interest_pts[i]);
    }

    SpectralAnalysis::SpectralAnalysis sa(NRadius);
    ShapeSpectral::ShapeSpectral shape_spectral(sa);

    Descriptor3D* descriptors_3d = &shape_spectral;
    double t = (double)cv::getTickCount();
    descriptors_3d->compute(pcd_pc, pcd_pc_kdtree, interest_pts_tmp, all_descriptor_results);
    t = (double)cv::getTickCount() - t;
    printf("extract shape spectral feature time = %gms\n", t*1000./cv::getTickFrequency());
}
