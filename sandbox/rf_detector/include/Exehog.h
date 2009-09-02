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

// Extracted from Opencv and Modified by Min Sun

#include "highgui.h"
#include "cv.h"
#include "cvaux.h"
#include "cxcore.h"
#include "RandomForest.h"

using namespace cv;

/****************************************************************************************\
*            HOG (Histogram-of-Oriented-Gradients) Descriptor and Object Detector        *
\****************************************************************************************/

struct HogFast
{
    enum { L2Hys=0 };

    HogFast() : winSize(64,128), blockSize(16,16), blockStride(8,8),
        cellSize(8,8), nbins(9), derivAperture(1), winSigma(-1),
        histogramNormType(L2Hys), L2HysThreshold(0.2), gammaCorrection(true)
    {}

    HogFast(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize,
        int _nbins, int _derivAperture=1, double _winSigma=-1,
        int _histogramNormType=L2Hys, double _L2HysThreshold=0.2, bool _gammaCorrection=false)
        : winSize(_winSize), blockSize(_blockSize), blockStride(_blockStride), cellSize(_cellSize),
        nbins(_nbins), derivAperture(_derivAperture), winSigma(_winSigma),
        histogramNormType(_histogramNormType), L2HysThreshold(_L2HysThreshold),
        gammaCorrection(_gammaCorrection)
    {}

    HogFast(const String& filename)
    {
        load(filename);
    }

    ~HogFast() {}

    size_t getDescriptorSize() const;
    bool checkDetectorSize() const;
    double getWinSigma() const;

    void setSVMDetector(const Vector<float>& _svmdetector);
    void setRFDetector( std::string& RFmodel_filename, std::string& FgBgProb_filename);

    bool load(const String& filename, const String& objname=String());
    void save(const String& filename, const String& objname=String()) const;

    size_t compute(const Mat& img, Vector<Point>& locations,
                         Vector<float>& descriptors,
                         Size winStride=Size(), Size paddingTL=Size(), Size paddingBR=Size(),
                         const string& EdgeMapFilename_filename="", bool PbFlag=true
                         );
    void detect(const Mat& img, Vector<Point>& foundLocations, Vector<Point>& foundIds,
                        vector<float>& hitsConfs, double hitThreshold=0, Size winStride=Size(),
                        Size padding=Size(), const vector<Mat>& fbr=vector<Mat>(), bool PbFlag=false,
                        const Vector<Point>& searchLocations=Vector<Point>());
    bool load_orient_edges( const string& orient_edges_filename, vector<Mat>& fbr );
    void detectMultiScale(const Mat& img, Vector<Rect>& foundLocations,
                                  Vector<Point>& foundIds,
                                  vector< float>& foundConfs, vector<int>& ScaleChangeBoundary,
                                  double hitThreshold=0, Size winStride=Size(),
                                  Size padding=Size(), double scale=1.05,
                                  int groupThreshold=2, float ObjHeight2winHeightRatio=3,
                                  const string& EdgeMapFilename_filename="", bool PbFlag=false);
    void detectSetOfScale(const Mat& img, Vector<Rect>& foundLocations,
                                  Vector<Point>& foundIds,
                                  vector< float>& foundConfs, vector<int>& ScaleChangeBoundary,
                                  double hitThreshold=0, Size winStride=Size(),
                                  Size padding=Size(), const vector< Vector<Point> >& locations= vector< Vector<Point> >(),
                                  const vector<Size>& scaledWinSize= vector<Size>(),
                                  int groupThreshold=2, float ObjHeight2winHeightRatio=3,
                                  const string& EdgeMapFilename_filename="", bool PbFlag=false);
    void detectAppendFea(const Mat& img,
        Vector<Point>& hits, Vector<Point>& hitsIds,
        vector<float>& hitsConfs, double hitThreshold,
        Size winStride, Size padding, const vector<Mat>& fbr, bool PbFlag, const Vector<Point>& locations, vector< cv::Vector<float>* >& des_ptr_3d);
    void detectSetOfScaleAppendFea(
        const Mat& img, Vector<Rect>& foundLocations, Vector<Point>& foundIds,
        vector< float>& foundConfs,
        cv::Vector<cv::Vector<float> >& descriptors_3d,
        vector<int>& ScaleChangeBoundary,
        double hitThreshold, Size winStride, Size padding,
        const vector<Vector<Point> >& locations, const vector<Size>& scaledWinSize,
        int groupThreshold, float ObjHeight2winHeightRatio, const string& EdgeMapFilename_filename="", bool PbFlag=false);

    void computeGradient_fbr(const vector<Mat>& fbr, Mat& grad, Mat& qangle,
                                Size paddingTL=Size(), Size paddingBR=Size()) const;
    void computeGradient(const Mat& img, Mat& grad, Mat& angleOfs,
                                 Size paddingTL=Size(), Size paddingBR=Size()) const;
    void normalizeBlockHistogram(Vector<float>& histogram) const;

    Size winSize;
    Size blockSize;
    Size blockStride;
    Size cellSize;
    int nbins;
    int derivAperture;
    double winSigma;
    int histogramNormType;
    double L2HysThreshold;
    bool gammaCorrection;
    Vector<float> svmDetector;

    // msun add for RF detector
    CRandomForest rf;
    vector< vector<float> > FgBgProb;
};

template<class T> struct index_cmp{
    index_cmp(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { return arr[a] > arr[b]; }
    const T arr;
};
