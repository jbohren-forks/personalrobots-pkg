#ifndef WGTEST3DPOSEESTIMATE_H_
#define WGTEST3DPOSEESTIMATE_H_

#include "CvStereoCamModel.h"

// http://www.videredesign.com/vision/stereo_manuals.htm
//  Small Vision System Calibration Addendum - Version 4.x 
// http://www.videredesign.com/docs/calibrate_4.4d.pdf 
// This supplement to the User Manual for SVS contains more detailed information on the calibration procedure. 

class CvTest3DPoseEstimate: public CvStereoCamModel
{
public:
    typedef CvStereoCamModel Parent;
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
    bool test();
    
    bool mTestCartesian;
    
protected:
	void _init();
	void transform(CvMat *points0, CvMat *points1);
	void transform(CvMat* R, CvMat *points0, CvMat* T, CvMat *points1);
	void randomize(CvMat *xyzs, int num, double maxVal);
	double randReal(double min, double max);
	void disturb(const CvMat *xyzs, CvMat *xyzsNoised);
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
};
#endif /*WGTEST3DPOSEESTIMATE_H_*/
