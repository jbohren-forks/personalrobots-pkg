#ifndef WGTEST3DPOSEESTIMATE_H_
#define WGTEST3DPOSEESTIMATE_H_

#include "CvStereoCamModel.h"
#include <vector>
#include <keypoint.h>
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
    	CartAndDisp, // two point clouds. One of each kind
    	Video
    } TestType;
    /**
     *  Fx  - focal length in x direction of the rectified image in pixels.
     *  Fy  - focal length in y direction of the rectified image in pixels.
     *  Tx  - Translatation in x direction from the left camera to the right camera.
     *  Clx - x coordinate of the optical center of the left  camera
     *  Crx - x coordinate of the optical center of the right camera
     *  Cy  - y coordinate of the optical center of both left and right camera
     */
    CvTest3DPoseEstimate(double Fx, double Fy, double Tx, double Clx=0.0, double Crx=0.0, double Cy=0.0);
    CvTest3DPoseEstimate();
	virtual ~CvTest3DPoseEstimate();
	bool testPointClouds();
	bool testVideos();
	bool testVideos2();
    bool test();

    // move the following two the appropriate place
    bool eulerAngle(const CvMat& rot, CvPoint3D64f &euler);
    typedef enum {
    	BadKeyFrame    = 0x0,
    	GoodKeyFrame   = 0x1,
    	KeyFrameNeeded = 0x2,
    	KeyFrameNotNeeded = 0x0,
    	GoodFrameButNotNeeded = GoodKeyFrame,
    	GoodFrameAndNeeded    = GoodKeyFrame | KeyFrameNeeded,
    	BadFrameButNeeded     = KeyFrameNeeded,
    	BadFrameAndNotNeeded  = BadKeyFrame | KeyFrameNotNeeded
    } KeyFramingDecision;
    KeyFramingDecision keyFrameDecision(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
    		vector<Keypoint>& keyPoints,
			int numInliers, const CvMat& rot, const CvMat& shift);

    TestType mTestType;

    static const int    defMaxDisparity  = 15;
    static const int    defMinNumInliersForGoodFrame = 10;
    static const int    defMinNumInliers = 50;
    static const double defMinInlierRatio  = .3;  // ratio between # inliers and # keypoints

//    static const double defMinAngleAlpha = 20.; // degree (0, 180)
//    static const double defMinAngleBeta  = 20.; // degree (0, 180)
//    static const double defMinAngleGamma = 20.; // degree (0, 180)
//    static const double defMinShift      = 500.; // mm
    static const double defMaxAngleAlpha = 2.; // degree (0, 180)
    static const double defMaxAngleBeta  = 2.; // degree (0, 180)
    static const double defMaxAngleGamma = 2.; // degree (0, 180)
    static const double defMaxShift      = 100.; // mm

    int mMinNumInliersForGoodFrame;
    int mMinNumInliers;
    int mMinInlierRatio;
    int mMaxAngleAlpha;
    int mMaxAngleBeta;
    int mMaxAngleGamma;
    int mMaxShift;

protected:
	void _init();
	void transform(CvMat *points0, CvMat *points1);
	void transform(CvMat* R, CvMat *points0, CvMat* T, CvMat *points1);
	void randomize(CvMat *xyzs, int num, double maxVal);
	double randReal(double min, double max);
	void disturb(const CvMat *xyzs, CvMat *xyzsNoised);
	static void MyMouseCallback(int event, int x, int y, int flagsm, void* param);

	CvPoint3D64f mEulerAngle;
	CvPoint3D64f mTranslation;
	double mRotData[9];
	CvMat  mRot;
	double mTransData[3];
	CvMat  mTrans;

	// Generating simulation data
	CvRNG  mRng;
	double mDisturbScale;
	double mOutlierScale;
	double mOutlierPercentage;

	bool mStop;
};
#endif /*WGTEST3DPOSEESTIMATE_H_*/
