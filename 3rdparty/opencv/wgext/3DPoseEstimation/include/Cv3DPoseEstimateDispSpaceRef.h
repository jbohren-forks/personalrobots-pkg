#ifndef CV3DPOSEESTIMATEDISPREF_H_
#define CV3DPOSEESTIMATEDISPREF_H_

#include "opencv/cxcore.h"
#include "CvStereoCamModel.h"
#include "Cv3DPoseEstimateRef.h"

/**
 * Reference implementation using OpenCV.
 * This implementation is not optimized, but may be more readable
 */
class Cv3DPoseEstimateDispSpaceRef: public Cv3DPoseEstimateRef, public CvStereoCamModel
{
public:
	typedef Cv3DPoseEstimateRef PoseParent;
	typedef CvStereoCamModel Parent;
	Cv3DPoseEstimateDispSpaceRef();
	virtual ~Cv3DPoseEstimateDispSpaceRef();

	/**
	 * The method that estimates transformation between two corresponding
   * point clouds, all in disparity coordinates.
   *
 	 * @param points0 3D points in parity space from camera 0, i.e. before
 	 * transformation
 	 * @param points1 3D points in parity space from camera 1, i.e. after
 	 * transformation
 	 *
 	 * The points are stored as
 	 *       -u0, v0, d0
 	 *       -u1, v1, d1
 	 *
 	 *       -ui, vi, di
 	 *
 	 * We estimate the "best" of rot and trans for the following tranformation
   * T * points0 => points1.
   * T is the transformation constructed from rot, trans and the projection
   * matrix.
	 *
	 * @param rot: (OUTPUT)rotation matrix
 	 * @param trans: (OUTPUT)translation matrix
 	 *
 	 * @return number of inliers
 	 */
	int estimate(CvMat *points0, CvMat *points1,
		CvMat *rot, CvMat *trans);

	/**
	 * The method that estimates transformation between two corresponding
	 * point clouds, one in Cartesian coordinates, the other in
	 * disparity coordinates.
	 * As an option, this method allows caller to specify triplet sets of
	 * points for inlier determination (in replacement of the random set)
	 * in the RANSAC step. This option is typically useful in calibration
	 * with checker board, where we know the non-colinear points.
	 * (This method is used in calibration between a stereo rig and a
	 * laser range finder)
	 */
	int estimateMixedPointClouds(
	    /// point cloud before transformation, in Cartesian coordinates.
	    CvMat *xyzs0,
	    /// point cloud after transformation, in disparity coordinates.
	    CvMat *uvds1,
	    /// The number of reference groups of 3. 0 if no reference group
	    /// is given.
			int numRefGrps,
			/// Indices of the reference groups. Group i is stored in
			/// refPoints[i*3], refPoints[i*3+1], and refPoints[i*3+2]
			int refPoints[],
      /// (Output) estimated rotation matrix.
			CvMat *rot,
      /// (Output) estimated translation matrix.
			CvMat *shift);

	static bool constructHomography(const CvMat& R, const CvMat& T,
			const CvMat& dispToCart, const CvMat& cartToDisp, CvMat& H);
	/**
	 * A Convenient function to map z to d, at the optical center
	 */
	double getD(double z) const;
	/*
	 * A convenient function to map disparity d to Z, at the optical center
	 */
	double getZ(double d) const;

protected:
  /// An internal method that performs estimation for all
  /// interface calls for estimation.
  /// @return number of inliers.
  /// @see estimateMixedPointClouds() and estimate(), who end up calling
  /// this method.
	int estimate(
	    /// point cloud before transformation, in Cartesian coordinates.
	    CvMat *xyzs0,
	    /// point cloud after transformation, in Cartesian coordinates.
	    CvMat *xyzs1,
	    /// point cloud before transformation, same points as in xyzs0, but
	    /// in disparity coordinates.
			CvMat *uvds0,
			/// point cloud after transformation, same points as in xyzs1, but
			/// in disparity coordinates.
			CvMat *uvds1,
			/// number of reference groups of 3. 0 if no reference group
      /// is given.
			int numRefGrps,
			/// Indices of the reference groups. Group i is stored in
			/// refPoints[i*3], refPoints[i*3+1], and refPoints[i*3+2]
			int refPoints[],
      /// (Output) estimated rotation matrix.
			CvMat *rot,
      /// (Output) estimated translation matrix.
			CvMat *shift);
	/// Construct disparity homography, given rotation matrix, translation
	/// vector and camera parameters.
	bool constructDisparityHomography(
	    /// rotation matrix
	    const CvMat *R,
	    /// translation matrix
	    const CvMat *T,
	    /// constructed homography
	    CvMat *H);
};

#endif /*CV3DPOSEESTIMATEDISPREF_H_*/
