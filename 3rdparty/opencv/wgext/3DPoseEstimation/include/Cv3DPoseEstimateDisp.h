#ifndef CV3DPOSEESTIMATEDISP_H_
#define CV3DPOSEESTIMATEDISP_H_

#include "CvStereoCamModel.h"
#include "Cv3DPoseEstimateRef.h"

#include <cxtypes.h>
#include <vector>
using namespace std;

/**
 *  Pose estimation between 2 corresponding point clouds in
 *  disparity coordinates.
 *  The point clouds are stored in rows as
 *   - u0, v0, d0
 *   - u1, v1, d1
 *   - ...
 *   - ui, vi, di
 *   - ...
 */
class Cv3DPoseEstimateDisp : public Cv3DPoseEstimateRef, public CvStereoCamModel
{
public:
  typedef Cv3DPoseEstimateRef PoseParent;
  typedef CvStereoCamModel Parent;
	Cv3DPoseEstimateDisp();
	virtual ~Cv3DPoseEstimateDisp();

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
	 * @param rot:   (OUTPUT)rotation matrix
	 * @param trans: (OUTPUT)translation matrix
	 *
	 * @return number of inliers
	 */
	int estimate(CvMat *points0, CvMat *points1,
	    CvMat *rot, CvMat *trans);

  /// Method to estimate transformation based pairs of 3D points, in
  /// disparity coordinates
  /// @return number of inliers
  int estimate(
      /// pairs of 3D locations, each for the same point, before
      /// and after the transformation
      vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
      /// (Output) rotation matrix.
      CvMat& rot,
      /// (Output) translation matrix.
      CvMat& shift,
      /// If true, the reverse transformation is estimated instead.
      bool reversed=false);

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
  /// Check and return the number of inliers in the 2 corresponding list
  /// of 3D point clouds.
  /// Points are given in disparity coordinates (u, v, d).
  /// @return number of inliers
	virtual int checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation);
	/// Check and return inliers.
  /// Points are given in disparity coordinates (u, v, d).
	/// @return number of inliers
	virtual int getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
	    CvMat* points0Inlier, CvMat* points1Inlier,
	    int *inlierIndices);
};

#endif /*CV3DPOSEESTIMATEDISP_H_*/
