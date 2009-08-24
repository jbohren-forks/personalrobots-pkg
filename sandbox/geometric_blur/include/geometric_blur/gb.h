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

// Author Min Sun

#include <iostream>
#include <fstream>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "cvaux.h"
#include <math.h>

using namespace cv;
using namespace std;

class gb{
public:
    gb(): nbins(4), DenseSample(true), alpha(0.5), beta(1){}
    gb( int nbins_, bool gammaCorrection_, Size paddingTL_, Size paddingBR_,
        bool DenseSample_, float ScaleRatio_, int NumScale_, Size winSize_, Size winStride_,
        float alpha_, float beta_):
        nbins(nbins_),  gammaCorrection(gammaCorrection_), paddingTL(paddingTL_), paddingBR(paddingBR_),
        DenseSample(DenseSample_), ScaleRatio(ScaleRatio_), NumScale(NumScale_), winSize(winSize_), winStride(winStride_),
        alpha(alpha_), beta(beta_){ setDefaultRsNthetas();}
    ~gb(){}

    // methods
    void setDefaultRsNthetas();
    void compute_orient_edges(Mat& Img, vector<Mat>& fbr);
    bool load_orient_edges( string& orient_edges_filename, vector<Mat>& fbr);
    void compute_sample_rois(Size CurrImgSize, vector<Rect>& rois, vector<Point>& roi_center);
    void compute_gb(vector<Mat>& fbr, vector< vector<float> >& features);
    void compute_gb_single_scale(vector<Mat>& fbr, vector< vector<float> > & features,Size CurrImgSize, vector<Rect>& rois, vector<Point>& roi_center);
    void compute_sample_points( vector<Point>& sample_points, vector< vector<int> >& sample_points_sigma_level);
    void normalizeBlockHistogram(vector<float>& _hist);

    // properties
    // oriented edge
    int nbins;
    bool gammaCorrection;
    Size paddingTL;
    Size paddingBR;

    // roi samples
    bool DenseSample;
    float ScaleRatio;
    int NumScale;
    Size winSize;
    Size winStride;

    // gb related
    float alpha, beta;
    vector<float> rs;
    vector<int> nthetas;

    Size imgSize;
    vector<Rect> ROI;
};
