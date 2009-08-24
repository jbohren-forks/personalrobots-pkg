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

// --------------------------------------------------------------
/*!
 * \file gb.h
 *
 * \brief The abstract base class geometric blur 2d-image descriptors
 *        that operate on 2-D data
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief gb is the abstract base class for geometric blur 2d descriptors.
 */
// --------------------------------------------------------------
class gb{
public:
    // --------------------------------------------------------------
    /*!
     * \brief Abstract constructor initialize only the parameters for gb descriptor
     */
    // --------------------------------------------------------------
    gb(): nbins(4), DenseSample(true), alpha(0.5), beta(1){ setDefaultRsNthetas();}

    // --------------------------------------------------------------
    /*!
     * \brief Abstract constructor initialize both the parameters for gb descriptor
     * and the parameters for sampling image patches.
     */
    // --------------------------------------------------------------
    gb( int nbins_, bool gammaCorrection_, Size paddingTL_, Size paddingBR_,
        bool DenseSample_, float ScaleRatio_, int NumScale_, Size winSize_, Size winStride_,
        float alpha_, float beta_):
        nbins(nbins_),  gammaCorrection(gammaCorrection_), paddingTL(paddingTL_), paddingBR(paddingBR_),
        DenseSample(DenseSample_), ScaleRatio(ScaleRatio_), NumScale(NumScale_), winSize(winSize_), winStride(winStride_),
        alpha(alpha_), beta(beta_){ setDefaultRsNthetas();}

    ~gb(){}

    // --------------------------------------------------------------
    /*!
     * \brief set the default rs( vector of radius) and nthetas 
     * (vector of bin size of each radius) using the values in Alex Bergs 
     * original matlab code
     */
    // --------------------------------------------------------------
    void setDefaultRsNthetas();

    // --------------------------------------------------------------
    /*!
     * \brief comput the oriented edges response using the same method in
     * opencv's Hog descriptor
     *
     * \param Img image of interest
     * \param fbr returned oriented edges responses, where the size of the vector
     * is the number of edge orientation
     */
    // --------------------------------------------------------------
    void compute_orient_edges(Mat& Img, vector<Mat>& fbr);

    // --------------------------------------------------------------
    /*!
     * \brief load the precomputed oriented edge responses
     *
     * \param orient_edges_filename filename of the precomputed
     * oriented edge responses
     * \param fbr returned oriented edges responses, where the size of the vector
     * is the number of edge orientation
     */
    // --------------------------------------------------------------
    bool load_orient_edges( string& orient_edges_filename, vector<Mat>& fbr);

    // --------------------------------------------------------------
    /*!
     * \brief Compute the region of interest. 
     * Now it will only densely sample uniformly on the image.
     *
     * \param CurrImgSize The current image scale after rescaling the image patch 
     * to a fix window size
     * \param rois a vector of region of interest
     * \param roi_center a vector of the centers of the region of interest
     */
    // --------------------------------------------------------------
    void compute_sample_rois(Size CurrImgSize, vector<Rect>& rois, vector<Point>& roi_center);

    // --------------------------------------------------------------
    /*!
     * \brief Compute the geometric blur features
     *
     * \param fbr The precomputed oriented edges responses, where the size of the vector
     * to a fix window size
     * \param features The returned feature matrix, 
     * which is the number of features by the feature dimension
     */
    // --------------------------------------------------------------
    void compute_gb(vector<Mat>& fbr, vector< vector<float> >& features);

    // --------------------------------------------------------------
    /*!
     * \brief Compute the geometric blur features using region of interest with the 
     * same window size
     *
     * \param fbr The precomputed oriented edges responses, where the size of the vector
     * to a fix window size
     * \param features The returned feature matrix, 
     * which is the number of features by the feature dimension
     * \param rois The returned region of interest 
     * \param roi_center The returned center of the region of interest 
     *
     */
    // --------------------------------------------------------------
    void compute_gb_single_scale(vector<Mat>& fbr, vector< vector<float> > & features,Size CurrImgSize, vector<Rect>& rois, vector<Point>& roi_center);

    // --------------------------------------------------------------
    /*!
     * \brief Compute sample points for each region of interest to calculate
     * geometric blur descriptor
     *
     * \param sample_points The returned sampled points according to
     * rs and ntheta
     * \param sample_points_sigma_level The returned variance of gaussian blur
     * for each sampled point
     */
    // --------------------------------------------------------------
    void compute_sample_points( vector<Point>& sample_points, vector< vector<int> >& sample_points_sigma_level);

    // --------------------------------------------------------------
    /*!
     * \brief The method L2 normalized the geometric blur descriptor for each 
     * region of interest
     *
     * \param _hist The raw geometric blur descriptor
     *
     */
    // --------------------------------------------------------------
    void normalizeBlockHistogram(vector<float>& _hist);

    // oriented edge
    /*! \brief The number of edge orientation that the descriptor computes */
    int nbins;
    /*! \brief The flag indicates whether to do gamma correction when computing 
     * oriented edges or not*/    
    bool gammaCorrection;
    /*! \brief Addtional padding alon the top and left corner of the image of interest */
    Size paddingTL;
    /*! \brief Addtional padding alon the bottom and right corner of the image of interest */
    Size paddingBR;

    // roi samples
    /*! \brief The flag indicates whether to do dense sampling of region of interests*/
    bool DenseSample;
    /*! \brief The ratio of scale search when sampling region of interest across scale. It has to be bigger than 1.*/
    float ScaleRatio;
    /*! \brief The number of scale to search for*/
    int NumScale;
    /*! \brief the window size of each image patch*/
    Size winSize;
    /*! \brief the window stride of consecutive image patchies*/
    Size winStride;

    // gb related
    /*! \brief The parameter for calculating the variance of the gaussian blur kernel*/
    float alpha;
    /*! \brief The parameter for calculating the variance of the gaussian blur kernel*/
    float beta;
    /*! \brief The vector of radius to sample*/
    vector<float> rs;
    /*! \brief The vector of the number of orientation bin to sample for each radius*/
    vector<int> nthetas;

    /*! \brief The height and width of the image of interest*/
    Size imgSize;
    /*! \brief The sampled region of interest*/
    vector<Rect> ROI;
};
