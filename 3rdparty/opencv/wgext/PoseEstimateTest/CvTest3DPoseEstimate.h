#ifndef WGTEST3DPOSEESTIMATE_H_
#define WGTEST3DPOSEESTIMATE_H_

#include "CvStereoCamModel.h"
#include <vector>
// star detector
#include <star_detector/include/keypoint.h>
#include <opencv/cvwimage.h>
using namespace cv;
using namespace std;


// http://www.videredesign.com/vision/stereo_manuals.htm
//  Small Vision System Calibration Addendum - Version 4.x
// http://www.videredesign.com/docs/calibrate_4.4d.pdf
// This supplement to the User Manual for SVS contains more detailed information on the calibration procedure.

class CvTest3DPoseEstimate: public CvStereoCamModel
{
public:
    typedef CvStereoCamModel Parent;
    typedef enum {
    	Cartesian,
    	Disparity,
    	CartAndDisp, Video} TestType;
    CvTest3DPoseEstimate(double Fx, double Fy, double Tx, double Clx = 0.0, double Crx = 0.0, double Cy = 0.0);
    CvTest3DPoseEstimate();
    virtual ~CvTest3DPoseEstimate();
    bool testPointClouds();
    bool testVideos1();
    bool testVideos();
    bool test();
    TestType mTestType;
protected:
    void _init();
    void transform(CvMat *points0, CvMat *points1);
    void transform(CvMat *R, CvMat *points0, CvMat *T, CvMat *points1);
    void randomize(CvMat *xyzs, int num, double maxVal);
    double randReal(double min, double max);
    void disturb(const CvMat *xyzs, CvMat *xyzsNoised);
    static void MyMouseCallback(int event, int x, int y, int flagsm, void *param);
    bool showDisparityMap(WImageBuffer1_16s & dispMap, string & winname, string & outputDirname, int frameIndex, int maxDisp);
    bool drawKeypoints(WImage3_b & image, vector<Keypoint> & keyPointsLast, vector<Keypoint> & keyPointsCurr);
    bool drawTrackablePairs(WImage3_b & image, vector<pair<CvPoint3D64f,CvPoint3D64f> > & trackablePairs);
    void loadStereoImagePair(string & dirname, int & frameIndex, WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage);
    CvPoint3D64f mEulerAngle;
    CvPoint3D64f mTranslation;
    double mRotData[9];
    CvMat mRot;
    double mTransData[3];
    CvMat mTrans;
    CvRNG  mRng;
	double mDisturbScale;
	double mOutlierScale;
	double mOutlierPercentage;

	bool mStop;
};
#endif /*WGTEST3DPOSEESTIMATE_H_*/
