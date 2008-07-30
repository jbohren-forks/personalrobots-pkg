#ifndef CV3DPOSEESTIMATEDISPREF_H_
#define CV3DPOSEESTIMATEDISPREF_H_

#include "cxcore.h"
#include "CvStereoCamParams.h"
#include "Cv3DPoseEstimateRef.h"

/**
 * Reference implmentation using OpenCV.
 * This implementation is not optimized.
 */
class Cv3DPoseEstimateDispSpaceRef: public Cv3DPoseEstimateRef, public CvStereoCamParams
{
public:
	typedef Cv3DPoseEstimateRef PoseParent;
	typedef CvStereoCamParams Parent;
	Cv3DPoseEstimateDispSpaceRef();
	virtual ~Cv3DPoseEstimateDispSpaceRef();
	
	/*
	 *  (Clx, Cy) center of the left image plane
	 *  (Crx, Cy) center of the right image plane
	 *  Fx: focal length in x direction
	 *  Fy: focal length in y direction
	 *  Tx: baseline length (sometimes referred as B ).
	 */
	bool setCameraParams(double Fx, double Fy, double Tx, double Clx=0, double Crx=0, double Cy=0);
	
	bool projection(CvMat *XYZs, CvMat *uvds);
	bool reprojection(CvMat *uvds, CvMat *XYZs);
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
		
protected:
#if 0
	int checkInLiers(CvMat *uvds0, CvMat *uvds1, CvMat* DisparitySpaceHomography);
    int getInLiers(CvMat *uvds0, CvMat *uvds1, CvMat* H, 
        CvMat* uvds0Inlier, CvMat* uvds1Inlier);
#endif
	int estimate(CvMat *xyzs0, CvMat *xyzs1,
			CvMat *uvds0, CvMat *uvds1, 
			int numRefGrps, int refPoints[],
			CvMat *rot, CvMat *shift, CvMat *& inliers0, CvMat *& inliers1);
    bool constructDisparityHomography(CvMat *R, CvMat *T, CvMat *H);
    // re-projection matrix from disparity space to cartesian space 
	CvMat* mDispToCart;
	// projection matrix from Cartesian coordinate to disparity coordinate
	CvMat* mCartToDisp;
};

#endif /*CV3DPOSEESTIMATEDISPREF_H_*/
