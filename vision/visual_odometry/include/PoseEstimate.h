#ifndef CV3DPOSEESTIMATEREF_H_
#define CV3DPOSEESTIMATEREF_H_


#include <opencv/cxcore.h>
//#include "CvMatUtils.h"

#include "CvRandomTripletSetGenerator.h"
#include "VisOdom.h"

namespace cv { namespace willow {

/**
 * Estimate transformation between corresponding 3D point clouds.
 * This is a reference implementation is less efficient, but maybe more readable. Please
 * use its extended class Cv3DPoseEstimate for practical use.
 * @warning we may consolidate this class with Cv3DPoseEstimate.
 */
class PoseEstimate : public PoseEstimator
{
public:
	PoseEstimate();
	virtual ~PoseEstimate();

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
 	 * @param smoothed: (Input) if true, Levenberg-Marquardt will be
 	 * applied over all the inliers to get a better estimation.
 	 *
 	 * @return number of inliers
	 */
	int estimate(CvMat *points0, CvMat *points1, CvMat *rot, CvMat *trans, bool smoothed=true);

	/// TODO not implemented yet
  virtual int estimate(
      /// key point list 0
      const Keypoints& keypoints0,
      /// key point list 1
      const Keypoints& keypoints1,
      /// index pairs of matching key points
      const vector<pair<int, int> >& matchIndexPairs,
      /// (Output) rotation matrix
      CvMat& rot,
      /// (Output) translation vector
      CvMat& shift,
      /// If true, Levenberg-Marquardt is used at the end for smoothing
      bool smoothed){return 0;}

	bool estimateWithLevMarq(const CvMat& points0inlier, const CvMat& points1inlier,
	    CvMat& rot, CvMat& trans);
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

	/// Default angle threshold for check for co-linearity among 3 points.
	static const double mDefMinAngleForNonColinearity = 15.0;

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
		this->mNumRansacIter = numIterations;
	}
	/**
	 * Get the references to the 2 corresponding lists of inliers.
	 * The points are stored in rows, as in Cv3DPoseEstimateRef::estimate.
	 * This method only gets a view to these objects, not ownership
	 * @return -true if the inliers are available. false otherwise.
	 */
	bool getInliers(CvMat *& inliers0, CvMat*& inliers1) const {
		if (mInliers0 == NULL || mInliers1 == NULL ){
			return false;
		} else {
			inliers0 = mInliers0;
			inliers1 = mInliers1;
			return true;
		}
	}
	int* getInliers() const {
	  return mInlierIndices;
	}
  /**
   * Get the references to the 2 corresponding lists of inliers.
   * The points are stored in rows, as in Cv3DPoseEstimateRef::estimate.
   * This method also assumes ownership of the matrices of inliers.
   * @return -true if the inliers are available. false otherwise.
   */
  bool fetchInliers(CvMat *& inliers0, CvMat*& inliers1){
    if (mInliers0 == NULL || mInliers1 == NULL ){
      return false;
    } else {
      inliers0 = mInliers0;
      inliers1 = mInliers1;
      mInliers0 = NULL;
      mInliers1 = NULL;
      return true;
    }
  }
  int* fetchInliers() {
    int * inliers = mInlierIndices;
    mInlierIndices = NULL;
    return inliers;
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
	/// This object is keeping track of the triplet sets that have been picked,
	/// to avoid duplicates.
	/// @return false if all possible triplet sets have been picked.
	bool pick3RandomPoints(
	    /// list 0 of input points, stored in rows, i.e. nx3
	    CvMat* points0,
	    /// list 1 of input points, stored in rows, i.e. nx3
	    CvMat* points1,
	    /// picked points from list 0, stored in column
	    CvMat* P0,
	    /// picked points from list 1, stored in column
	    CvMat* P1
	);
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
	/// a fast implementation of of inlier check, customized for specific
	/// data structures (see specifications of arguments.)
  static int checkInliers(
      /// the number of input points
      int numPoints,
      /// pointer to point cloud 0 coordinates as x0,y0,z0,x1,y1,z1,..., in doubles
      double *_P0,
      /// pointer to point cloud 1 coordinates as x0,y0,z0,x1,y1,z1,..., in doubles
      double *_P1,
      /// transformation matrix stored as a double array of 16, row first.
      double *_T,
      /// the threshold use to check for inliers
      double threshold
  );
	/// Check and return inliers.
	/// @return The number of inliers
  virtual int getInLiers(
      /// point cloud before transformation
      CvMat *points0,
      /// point cloud after transformation
      CvMat *points1,
      /// the transformation matrix. 4x4  homography, or 4x3 for affine.
      CvMat* transformation,
      /// max number of inliers need to be returned from the call.
      /// Or, the size of the buffer(s) allocated for returning inliers.
      int maxNumInliersReturned,
      /// correspoding inliers in points0
      CvMat* points0Inlier,
      /// corresponding inliers in point1
      CvMat* points1Inlier,
      /// the indices of the inliers
      int    inlierIndices[]
  );
  /// a fast implementation of of inlier  checking and getting, customized for specific
  /// data structures (see specifications of arguments.)
  static int getInliers(
      /// the number of input points
      int numPoints,
      /// pointer to point cloud 0 coordinates as x0,y0,z0,x1,y1,z1,..., in doubles
      double *_P0,
      /// pointer to point cloud 1 coordinates as x0,y0,z0,x1,y1,z1,..., in doubles
      double *_P1,
      /// transformation matrix stored as a double array of 16, row first.
      double *_T,
      /// the threshold use to check for inliers
      double threshold,
      /// The maximum number of inlier returned in inliers0, inliers1, or inlierIndices.
      /// Basically, the length of these arrays.
      int maxNumInlierReturned,
      /// An array to store the inlier list from point cloud 0. Null if not interested.
      /// The inliers are stored as x0,y0,z0,x1,y1,z1,....
      double inliers0[],
      /// An array to store the inlier list from point cloud 1. Null if not interested.
      double inliers1[],
      /// An array to store the list of inlier pairs, indexed into the input lists.
      int    inlierIndices[]
  );
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
	/// Update the record of inliers with current input.
	/// Old data is erased and deallocated properly.
	void updateInlierInfo(
	    CvMat* points0Inlier,
	    CvMat* points1Inlier,
	    int* inlierIndices);

	int    mNumRansacIter; //< num of iteration in RANSAC
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
	double mT_Data[4*4];
	CvMat    mT;

#if 0
	/// random number generator used in RANSAC
	CvRNG   mRng;
#endif
	CvRandomTripletSetGenerator mRandomTripletSetGenerator;

	/// store the best candidate before levmarq
	double mRTBestWithoutLevMarqData[16];
	CvMat   mRTBestWithoutLevMarq;

	/// inliers0 from list 0 of input points
	CvMat*  mInliers0;
	/// inliers1 from list 1 of input points.
	CvMat*  mInliers1;
	/// indices of the inliers in the input lists.
	int*    mInlierIndices;
};
}// namespace willow
} // namespace cv
#endif /*CV3DPOSEESTIMATEREF_H_*/
