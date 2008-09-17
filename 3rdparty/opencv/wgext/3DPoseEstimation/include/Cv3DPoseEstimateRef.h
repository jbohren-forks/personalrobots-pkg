#ifndef CV3DPOSEESTIMATEREF_H_
#define CV3DPOSEESTIMATEREF_H_

#include <opencv/cxcore.h>
//#include "CvMatUtils.h"

typedef double CvMyReal;
#define CV_XF CV_64F

/**
 * Estimate transformation between corresponding 3D point clouds.
 * This is a reference implementation is less efficient, but maybe more readable. Please
 * use its extended class Cv3DPoseEstimate for practical use.
 * @warning we may consolidate this class with Cv3DPoseEstimate.
 */
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
	 * The method that estimates transformation between two corresponding
	 * point clouds.
 	 * @param points0: 3D points in Cartesian coordinates from camera 0, i.e before
 	 * transformation.
 	 * @param points1: 3D points in Cartesian coordinates from camera 1, i.e.after
 	 * transformation.
 	 *
 	 * The points are stored in rows as
 	 *       - x0, y0, z0
 	 *       - x1, y1, z1
 	 *       - ...
 	 *       - xi, yi, zi
 	 *
 	 * We estimate the "best" of rot and trans for the following tranformation
 	 * rot * points0 + trans => points1.
 	 *
 	 * The main flow of the algorithm is:
 	 * - Perform a RANSAC to select inliers
	 * - Perform Levenberge-Marquardt on the inliers
	 *
	 * @param rot:  (Output) rotation matrix
 	 * @param trans: (Output) translation matrix
 	 *
 	 * @return number of inliers
	 */
	int estimate(CvMat *points0, CvMat *points1, CvMat *rot, CvMat *trans);

	/**
	 *  Estimate the transformation matrices rot and trans by least squares
	 *  over all points.
	 *  The points are stored in rows, as in Cv3DPoseEstimateRef::estimate
	 */
	bool estimateLeastSquare(
	    /// 3D points in Cartesian coordinates, before transformation.
	    CvMat *P0,
	    /// 3D points in Cartesian coordinates, after transformation
	    CvMat *P1,
	    /// (OUTPUT) rotation matrix. 3x3
	    CvMat *rot,
	    /// (OUTPUT) translation (shift) vector. 3x1
	    CvMat *trans);
	/** Default error threshold used in RANSAC */
	static const double mDefErrThreshold  = 2.0;

	/**
	 *  @return a reference to the 4x4 transformation matrix estimated.
	 */
	CvMat* getFinalTransformation() { return &mT;};
	/**
	 *  @return a reference to the 4x4 transformation matrix estimated
	 *  by the RANSAC step, right before the Levenberg-Marquardt
	 *  optimization.
	 */
	CvMat* getBestTWithoutNonLinearOpt() { return &mRTBestWithoutLevMarq;};
	void setInlierErrorThreshold(double threshold) {
		this->mErrThreshold = threshold;
	}
	/**
	 * Set the number of iterations in the RANSAC step
	 */
	void setNumRansacIterations(
	    /// The number of iterations in the RANSAC step/
	    int numIterations) {
		this->mNumIterations = numIterations;
	}
	/**
	 * Get the references to the 2 corresponding lists of inliers.
	 * The points are stored in rows, as in Cv3DPoseEstimateRef::estimate
	 * @return -true if the inliers are available. false otherwise.
	 */
	bool getInliers(CvMat *& inliers0, CvMat*& inliers1){
		if (mInliers0 == NULL || mInliers1 == NULL ){
			return false;
		} else {
			inliers0 = mInliers0;
			inliers1 = mInliers1;
			return true;
		}
	}
	/**
	 *  A convenient utility to construct a 4x4 transformation matrix
	 *  from a 3x3 rotation matrix and a 3x1 translation matrix
	 */
	static bool constructTransform(const CvMat& rot, const CvMat& shift, CvMat& transform);

protected:
	/// a utility function to decide if the 3 points are too close to be
  /// co-linear
  /// @warning this function may be moved to somewhere more appropriate for it.
	bool tooCloseToColinear(CvMat* points);
	/// Same as Cv3DPoseEstimateRef::estimateLeastSquareInCol except
	/// that the points are stored in columns.
	bool estimateLeastSquareInCol(CvMat *P0, CvMat *P1, CvMat *rot, CvMat *trans);
	/// Pick 3 random non-colinear pairs from the two corresponding lists.
	/// The 3 points in each list needs to be far away enough from being
	/// co-linear.
	bool pick3RandomPoints(CvMat* points0, CvMat* points1, CvMat* P0, CvMat* P1, bool fInputPointsInRows=true);
	/// An internal interfacing method that construct the 4x4 homography matrix from
	/// rotation matrix R and translation matrix T
	static bool constructRT(CvMat *R, CvMat *T, CvMat *RT);
	/// Check and return the number of inliers in the 2 corresponding list
	/// of 3D point clouds.
	/// @return The number of inliers.
	virtual int checkInLiers(
      /// point cloud before transformation
	    CvMat *points0,
      /// point cloud after transformation
	    CvMat *points1,
      /// the transformation matrix. 4x4  homography, or 4x3 for affine.
	    CvMat* transformation);
	/// Check and return inliers.
	/// @return The number of inliers
	virtual int getInLiers(
      /// point cloud before transformation
	    CvMat *points0,
      /// point cloud after transformation
	    CvMat *points1,
      /// the transformation matrix. 4x4  homography, or 4x3 for affine.
	    CvMat* transformation,
	    /// correspoding inliers in points0
	    CvMat* points0Inlier,
	    /// corresponding inliers in point1
	    CvMat* points1Inlier);
	/// Check if the point is an inlier.
	/// @return true if point pair i in the 2 corresponding list
	/// is an inlier. False otherwise.
	bool isInLier(
	    /// point cloud before transformation
	    CvMat *points0,
	    /// point cloud after transformation
	    CvMat *points1,
	    /// index of the point.
	    int i);
	int    mNumIterations; //< num of iteration in RANSAC
	double mMinDet; //< to decide if 3 points are tooCloseToColinear
	/// minimum angle to decide if 3 points are too close to being
	/// co-linear.
	double mMinAngleForRansacTriple;
	int mNumTriesForRandomTriple; //< max num of tries to get a group of 3 random points

	/// parameters for deciding the inliers
	/// the error is define as norm(mMapping * residue), or
	/// norm(residue) if mMapping is NULL;
	CvMat* mErrMapping;
	/// The type of norm used in error measurement.
	/// The type enum is defined in cxcore.h
	int    mErrNormType;
	/// Threshold to decide is a point is inlier or not.
	double mErrThreshold;

	// buffers use in computations
	double  mResidue1_Data[3];
	CvMat   mResidue1;
	double  mResidue2_Data[3];
	CvMat   mResidue2;
	double  mW1_Data[4];
	CvMat   mW1;   // 4x1 matrix to hold a point in homogenous coordinates
	CvMyReal mT_Data[4*4];
	CvMat    mT;

	/// random number generator used in RANSAC
	CvRNG   mRng;

	/// store the best candidate before levmarq
	CvMyReal mRTBestWithoutLevMarqData[16];
	CvMat   mRTBestWithoutLevMarq;

	CvMat*  mInliers0;
	CvMat*  mInliers1;
};

#endif /*CV3DPOSEESTIMATEREF_H_*/
