#ifndef WGLEVMARQDISPSPACE_H_
#define WGLEVMARQDISPSPACE_H_

#include "LevMarqTransform.h"

namespace cv {
namespace willow {

/**
 * Levenberg-Marquardt optimization optimized for estimate
 * transformation of 3d point clouds in disparity coordinates.
 * @see LevMarqTransform for similar detail description.
 */
class LevMarqTransformDispSpace : public LevMarqTransform
{
public:
	LevMarqTransformDispSpace(
	    /// transformation from disparity space to cartesian space
	    const CvMat *disparityTo3D,
	    /// transformaton from cartesian to disparity space
	    const CvMat *threeDToDisparity,
	    /// max number of RANSAC iterations
	    int numMaxInter = defNumMaxIter,
	    /// use rodrigues parameter or euler angles as optimization variables
	    /// regarding rotation.
	    AngleType angleType=Rodrigues);
	virtual ~LevMarqTransformDispSpace();
  /**  A routine that performs optimization.
   *  @param P0  - Nx3 matrix stores data point list P0, one point (x, y, z) each row
   *  @param P1  - Nx3 matrix stores data point list P1, one point (x, y, z) each row
   *  @param param - Transformation parameters. [alpha, beta, gamma, dx, dy, dz] if Euler angles are used, or
   *  [rx, ry, rz, dx, dy, dz] if Rodrigues parameters are used for rotation representation, where
   *  [alpha, beta, gamma] are the Euler angles, [rx, ry, rz] the Rodrigues parameters, and
   *  [dx, dy, dz] the shift (translation) vector.
   *  They are input as initial parameters, output as optimized parameters.
   *  @return   true if optimization is successful.
   */
	bool optimize(
	    const CvMat* P0,
	    const CvMat* P1,
	    double param[]) {
		return optimizeAlt(P0, P1, param);
	}
protected:
	bool constructHomographyMatrix(const CvMat* params);
	bool constructHomographyMatrix(const CvMat* params, double H[]);
	virtual bool constructTransformationMatrix(const CvMat *param);
	virtual bool constructTransformationMatrix(const CvMat * param, double T[]);
  virtual bool constructTransformationMatrices(const CvMat *param, double delta);
  virtual void constructTransformationMatrices(const CvMat *param, double delta,
      double* transf, double* transf_fwd);
	virtual bool computeResidueVector(const CvMat *xyzws0, const CvMat *xyzws1, CvMat* resVector);
	virtual bool computeResidue(const CvMat *xyzs0, const CvMat *xyzs1, CvMat* resVector);
	virtual bool computeResidue(const CvMat *xyzs0, const CvMat *xyzs1, const CvMat *T, CvMat* resVector);
	bool optimizeAlt(const CvMat *xyzs0, const CvMat *xyzs1, double _param[]);
	double _Homography[16];
	CvMat mHomography;

	const CvMat *m3DToDisparity; // projection from 3D coordinate to disparity coordinates
	const CvMat *mDisparityTo3D; // projection from disparity coordinate to 3D coordinates
};
}
}
#endif /*WGLEVMARQDISPSPACE_H_*/
