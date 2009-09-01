
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

    void cast_vots( Mat& Img, Vector<Rect>& rect, Vector<Point>& treeInfo, vector<float>& conf,
        vector< int>& scale_count, vector< vector<float> >& NormVotes, vector<int>& ViewIds, int MaxViewId, vector<double>& AvergeKernel,
        int nCandidatePerDim, Size winStride, float winHeight2sinStrideRatio, float Scale, int MeanShiftMaxIter, vector< Vector<Rect> >& ObjCenterWindowsAll, vector< vector<float> >& ObjCenterConfAll, bool VisualizationFlag);

    static Vector<float> getDefaultPeopleDetector();

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
