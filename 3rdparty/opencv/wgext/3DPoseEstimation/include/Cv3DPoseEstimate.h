#ifndef WG3DPOSEESTIMATETIGHT_H_
#define WG3DPOSEESTIMATETIGHT_H_

#include "Cv3DPoseEstimateRef.h"

/**
 * A class to estimate pose transformations, given 2 list of corresponding
 * 3D points.
 */
class Cv3DPoseEstimate : public Cv3DPoseEstimateRef
{
public:
	Cv3DPoseEstimate();
	virtual ~Cv3DPoseEstimate();

	// see overridden method in parent
	int estimate(
	    CvMat *points0,
	    CvMat *points1,
	    CvMat *rot,
	    CvMat *trans);

	// SVD decomposition for 3x3 matrix
//	int svd3x3(double Q[3*3], double U[3*3], double W[3], double V[3*3], int flag=NULL);
protected:
	virtual int checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation);
	virtual int getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
	    CvMat* points0Inlier, CvMat* points1Inlier);
//	int prodOwnTrans(double X[3*3], double XXt[3*3]);
};

#endif /*WG3DPOSEESTIMATETIGHT_H_*/
