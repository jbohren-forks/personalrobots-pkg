#ifndef CV3DPOSEESTIMATEDISPREF_H_
#define CV3DPOSEESTIMATEDISPREF_H_

#include "opencv/cxcore.h"
#include "CvStereoCamModel.h"
#include "Cv3DPoseEstimateRef.h"

/**
 * Reference implmentation using OpenCV.
 * This implementation is not optimized.
 */
class Cv3DPoseEstimateDispSpaceRef: public Cv3DPoseEstimateRef, public CvStereoCamModel
{
public:
	typedef Cv3DPoseEstimateRef PoseParent;
	typedef CvStereoCamModel Parent;
	Cv3DPoseEstimateDispSpaceRef();
	virtual ~Cv3DPoseEstimateDispSpaceRef();

	/*
	 * INPUT:
 	 * pionts0: 3D points in parity space from camera 0,
 	 * points1: 3D points in parity space from camera 1
 	 *
 	 * The points are stored as
 	 *       u0, v0, d0
 	 *       u1, v1, d1
 	 *
 	 *       ui, vi, di
 	 *
	 *
	 * OUTPUT:
	 * rot: rotation matrix
 	 * trans: translation matrix
 	 *
 	 * RETURN:
 	 * number of inliers
 	 */
	int estimate(CvMat *points0, CvMat *points1,
		CvMat *rot, CvMat *trans, CvMat *& inliers0, CvMat *& outliers1);

	int estimateMixedPointClouds(CvMat *xyzs0, CvMat *uvds1,
			int numRefGrps, int refPoints[],
			CvMat *rot, CvMat *shift, CvMat *& inliers0, CvMat *& outliers1);

	/*
	 * A Convenient function to map z to d, at the optical center
	 */
	double getD(double z);
	/*
	 * A convenient function to map disparity d to Z, at the optical center
	 */
	double getZ(double d);

protected:
	int estimate(CvMat *xyzs0, CvMat *xyzs1,
			CvMat *uvds0, CvMat *uvds1,
			int numRefGrps, int refPoints[],
			CvMat *rot, CvMat *shift, CvMat *& inliers0, CvMat *& inliers1);
    bool constructDisparityHomography(CvMat *R, CvMat *T, CvMat *H);
};

#endif /*CV3DPOSEESTIMATEDISPREF_H_*/
