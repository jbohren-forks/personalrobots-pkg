#ifndef CV3DPOSEESTIMATEDISP_H_
#define CV3DPOSEESTIMATEDISP_H_

#include "Cv3DPoseEstimateDispSpaceRef.h"

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
class Cv3DPoseEstimateDisp : public Cv3DPoseEstimateDispSpaceRef
{
public:
  typedef Cv3DPoseEstimateDispSpaceRef Parent;
	Cv3DPoseEstimateDisp();
	virtual ~Cv3DPoseEstimateDisp();

	using Parent::estimate;

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
protected:
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
