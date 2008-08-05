#ifndef CV3DPOSEESTIMATEREF_H_
#define CV3DPOSEESTIMATEREF_H_

#include <opencv/cxcore.h>
#include "CvMatUtils.h"

class Cv3DPoseEstimateRef
{
public:
	Cv3DPoseEstimateRef();
	virtual ~Cv3DPoseEstimateRef();
	
	/**
	 * set up the error measurement to decide whether a point is inlier
	 */
	void configureErrorMeasurement(CvMat *mapping, 
			double threshold=mDefErrThreshold, int normType=CV_C);
	
	/**
	 * INPUT:
 	 * pionts0: 3D points in Cartesian coordinates from camera 0
 	 * points1: 3D points in Cartesian coordinates from camera 1
 	 * 
 	 * The points are stored as 
 	 *       x0, y0, z0
 	 *       x1, y1, z1
 	 * 
 	 *       xi, yi, zi
 	 * 
	 * 
	 * OUTPUT:
	 * rot: rotation matrix
 	 * trans: translation matrix
 	 * 
 	 * RETURN:
 	 * number of inliers
	 * 
	 */
	int estimate(CvMat *points0, CvMat *points1, CvMat *rot, CvMat *trans,
			CvMat *& inliers0, CvMat *& inliers1);
	bool estimateLeastSquare(CvMat *P0, CvMat *P1, CvMat *rot, CvMat *trans);
	static const double mDefErrThreshold  = 2.0;

	CvMat* getFinalTransformation() { return &mT;};
	CvMat* getBestTWithoutNonLinearOpt() { return &mRTBestWithoutLevMarq;};
	void setInlierErrorThreshold(double threshold) {
		this->mErrThreshold = threshold;
	}
protected:
	// a utility function that shall not belong here
	bool tooCloseToColinear(CvMat* points);
	bool estimateLeastSquareInCol(CvMat *P0, CvMat *P1, CvMat *rot, CvMat *trans);
	bool pick3RandomPoints(CvMat* points0, CvMat* points1, CvMat* P0, CvMat* P1, bool fInputPointsInRows=true);
	static bool constructRT(CvMat *R, CvMat *T, CvMat *RT);
	virtual int checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation);
	virtual int getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
	    CvMat* points0Inlier, CvMat* points1Inlier);
	bool isInLier(CvMat *points0, CvMat *points1, int i);
	int    mNumIterations; // num of iteration in RANSAC
	double mMinDet; // to decide if 3 points are tooCloseToColinear
	double mMinAngleForRansacTriple;
	int mNumTriesForRandomTriple; // max num of tries to get a group of 3 random points
	
	// parameters for deciding the inliers
	// the error is define as norm(mMapping * residue), or
	// norm(residue) if mMapping is NULL;
	CvMat* mErrMapping;
	int    mErrNormType; // defined in cxcore.h
	double mErrThreshold;
	
	// buffers
	double  mResidue1_Data[3];
	CvMat   mResidue1;
	double  mResidue2_Data[3];
	CvMat   mResidue2;
	double  mW1_Data[4];
	CvMat   mW1;   // 4x1 matrix to hold a point in homogenous coordinates
	CvMyReal mT_Data[4*4];
	CvMat    mT;
	
	CvRNG   mRng;  // random number generator
	
	CvMyReal mRTBestWithoutLevMarqData[16]; // store the best candidate before levmarq
	CvMat   mRTBestWithoutLevMarq;
};

#endif /*CV3DPOSEESTIMATEREF_H_*/
