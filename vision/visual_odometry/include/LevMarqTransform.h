#ifndef CVLEVMARQ3D_H_
#define CVLEVMARQ3D_H_

#include "LevMarq2.h"
#include "CvMatUtils.h"

namespace cv {
namespace willow {

/**
 *  A form of Levenberg-Marquardt that specializes on optimized estimation of transformation of 3D point clouds.
 *  There is 6 parameters for a transformation. The first 3 are for rotation
 * (either euler or rodrigues) and last 3 are shift (translation) vector.
 * For the case of Cartesian coordinates, the last 3 (shift vector) parameters are linearly related to
 * the error function, which give rise to sparsity in the matrix, which can be exploited for speeding-up.
 *
 * More specifically, the cost function to be optimized (minimized) is the 2-norm of the residue vector
 * \f[
 *      R = T \cdot P_0^T - P_1^T,
 * \f]
 * where \f$T\f$ is the transformation matrix, and \f$P_0, P_1\f$,  are two lists of 1-1 corresponding
 * points before and after the transformation. \f$ P_0 \f$ and \f$ P_1 \f$ are given as
 * nx3 matrices, where each row \f$ P_{0,i} \f$  is a point, and \f$ n \f$ is the
 * number of points.
 * The 6 parameters to be optimized over  \f$ C_j, j={0,5} \f$, where the first 3 are for rotation
 * and the last 3 are for translation.
 * The Jacobian of the residue vector is therefore denoted as
 * \f[ J_{i,j} = \frac{\partial}{\partial{C_j}} R_i, i=0,..., (n-1), j=0,...,5  \f]
 * where \f$ R_i \f$ is the residue vector w.r.t. to point i. Hence please note that
 * \f$ J_{i,j} \f$ is a vector of length 3.
 * In the following context,
 * \f$ R_{i,d}, d \in \{x,y,z\} \f$ is used to refer to the 3 component of \f$ R_i \f$ for each dimension.
 *
 * Levenberg-Marquardt optimization involves computation of the following matrices
 * \f$ JtJ \f$ and \f$ JtErr \f$, as defined below, whose structures shall be studied to exploit their
 * sparsity for performance purposed.
 * \f{equation}
 *      JtJ = J^T J,
 *      \label{eqn:JtJ}
 * \f}
 * each entry of which,
 * \f{equation}
 *      JtJ_{s,t} = \sum_{i=0, d\in\{x,y,z\}}^n \frac{\partial R_{i,d}}{\partial s} \frac{\partial R_{i,d}}{\partial t}
 *      \label{eqn:JtJ_st}
 * \f}
 * and
 * \f{equation}
 *      JtErr = J^T R,
 *      \label{eqn:JtErr}
 * \f}
 * which entry of which
 * \f{equation}
 *      JtErr_s = { \sum_{i=0, d\in\{x,y,z\}}^n \frac{\partial R_{i,d}}{\partial s} R_{i,d}}
 *      \label{eqn:JtErr_s}
 * \f}
 *
 * From equations \f$(\ref{eqn:JtJ_st})\f$ and \f$(\ref{eqn:JtErr_s})\f$, we can
 * see that we incrementally update this entries point by point.
 *
 * As aforth mentioned, the last parameters (translation vector) are linearly related
 * to the residue vector. In fact
 * \f{eqnarray}
 *    \frac{\partial}{\partial{s}}{R_{i,d}} & = & 1, s=d,\\
 *                                          & = & 0, s \neq d
 * \f}
 * In other words, in matrix JtJ, in the lower right 3x3 sub matrix,
 * \f{eqnarray}
 *    JtJ_{s,t} &=& n, s=t,\\
 *              &=& 0, s\neq t
 * \f}
 * and in the upper right 3x3 sub matrix, where s is one of the rotation
 * parameters and t is one of the translation parameters,
 * \f{equation}
 *    JtJ_{s,t} = ( \
 *    \sum_{i=0}^n \frac{\partial}{\partial{s}}{R_{i,x}},\
 *    \sum_{i=0}^n \frac{\partial}{\partial{s}}{R_{i,y}},\
 *    \sum_{i=0}^n \frac{\partial}{\partial{s}}{R_{i,z}} \
 *   )
 * \f}
 *
 * Similarly, we simplify the computation of JtErr by exploiting the same
 * property.
 *
 * @author Jindong (JD) Chen (jdchen@willowgarage.com)
 */
class LevMarqTransform
{
public:
  /// Types of representations of 3D rotations.
  typedef enum {
    Euler,      ///< use Euler angles to represent 3D rotation
    Rodrigues   ///< use Rodrigues parameters to represent 3D rotation
  } AngleType;
	/**
	 * @param numMaxIter - maximum number of iterations in LevMar optimization
	 * @param angleType  - use euler angles or rodrigues parameters as optimization
	 * variables regarding rotation.
	 */
	LevMarqTransform(int numMaxIter = defNumMaxIter,
	    AngleType angleType=Rodrigues);
	virtual ~LevMarqTransform();
	const static int numNonLinearParams = 3;   ///< the num of nonlinear parameters, namely rotation related
	const static int numParams = 6;	      ///< Total num of parameters
	const static int defNumMaxIter = 50;  ///< Maximum num of iterations
	const static int defMaxTimesOfUpdates = 800; //<maximum num of times update() or updateAlt() is called
	const static double defDeltaForNumericDeriv = 1.e-10;

	/**
	 *  A routine that performs optimization.
   *  @param P0  - Nx3 matrix stores data point list P0, one point (x, y, z) each row
   *  @param P1  - Nx3 matrix stores data point list P1, one point (x, y, z) each row
	 *  @param param - Transformation parameters. [alpha, beta, gamma, dx, dy, dz] if Euler angles are used, or
	 *  [rx, ry, rz, dx, dy, dz] if Rodrigues parameters are used for rotation representation, where
	 *  [alpha, beta, gamma] are the Euler angles, [rx, ry, rz] the Rodrigues parameters, and
	 *  [dx, dy, dz] the shift (translation) vector.
	 *  They are input as initial parameters, output as optimized parameters.
	 *  @return   true if optimization is successful.
	 */
	bool optimize(const CvMat* P0, const CvMat* P1, double param[]);
	/**
	 *  A routine that performs optimization.
   *  @param P0  - Nx3 matrix stores data point list P0, one point (x, y, z) each row
   *  @param P1  - Nx3 matrix stores data point list P1, one point (x, y, z) each row
	 *  @param rot, trans - rotation and shift matrices
	 *       they input as initial parameters, output as optimized parameters
	 */
	bool optimize(const CvMat* P0, const CvMat* P1, CvMat *rot, CvMat* trans);

	/// a convenient routine to convert rotation matrix and translation matrix
	/// into optimization parameters.
	void rotAndShiftMatsToParams(
	    /// rotation matrix
	    const CvMat& rot,
	    /// translation matrix
	    const CvMat& trans,
	    /// optimization parameters
	    double params[6]) const;
	/// a convenient routine to convert from transformation matrix into
	/// optimization parameters.
	void transfToParams(
	    /// transformation matrix
	    const CvMat& transf,
	    /// optimization parameters
	    double params[6]) const;
	/// a convenient routine to convert from optimization parameters to
	/// rotatiom matrix and translation matrix
	void paramsToRotAndShiftMats(
	    /// optimization parameters
	    const double params[6],
	    /// rotation matrix
	    CvMat& rot,
	    /// translation matrix
	    CvMat& trans) const;

	const AngleType mAngleType;   /// The 3D rotation representation used in optimization.

protected:
	bool constructRTMatrix(const CvMat* param);
	bool constructRTMatrices(const CvMat* param, double delta);
	/// Given parameters, construct the transformation matrix.
	bool constructRTMatrix(const CvMat * param, double _RT[]) const;
	bool computeForwardResidues(const CvMat *xyzs0, const CvMat *xyzs1, CvMat *res);
	virtual bool constructTransformationMatrix(const CvMat *param);
	virtual bool constructTransformationMatrix(const CvMat *param, double T[]);
	virtual bool constructTransformationMatrices(const CvMat *param, double delta);
	/**
	 * compute the residue error \f$ R = T \cdot P0 - P1 \f$, where
	 * \f$ T \f$ is LevMarqTransform::mRT3x4.
   * @param P0 - Input points before transformation. An n by 3 matrix where
   * each row is a point \f$ (x,y,z) \f$.
   * @param P1 - Input points after transformation. An n by 3 matrix where
   * each row is a point \f$ (x,y,z) \f$.
   * @param R  - (Output) The residue.
	 */
	virtual bool computeResidue(const CvMat *P0, const CvMat *P1, CvMat* R);
	/**
	 * Compute the residue error of \f$ R = T \cdot P_0 - P_1 \f$. \f$ T \f$
	 *  is a 4x3 transformation matrix.
   * @param P0 - Input points before transformation. An n by 3 matrix where
   * each row is a point \f$ (x,y,z) \f$.
   * @param P1 - Input points after transformation. An n by 3 matrix where
   * each row is a point \f$ (x,y,z) \f$.
   * @param T - The transformation matrix
   * @param R  - (Output) The residue.
	 */
	virtual bool computeResidue(const CvMat *P0, const CvMat *P1, const CvMat* T, CvMat* R);
	/**
	 *  Use CvLevMarq.updateAlt() to do optimization.
	 *  This function takes on the burden to compute \f$J^T J\f$,
	 *  the inner product of the transpose of the Jacobian and the Jacobian itself,
	 *  and \f$J^T R\f$, the inner product of the Jacobian and the residue vector.
	 *  At the
	 *  same time the opportunity to exploit the sparsity of \f$J^T J\f$ and \f$J^T R\f$ for
	 *  speed performance.
	 *  In this setting, \f$J^T J\f$ is a 6x6 matrix, with the following structure,
	 *   - The \f$[s,t]\f$ entry of the matrix is
	 *     \f[
	 *     \sum_{i=0, d=\{x,y,z\}}^n (\partial R_{i,d} / \partial s) (\partial R_{i,d} / \partial t)
	 *     \f]
	 *   - Obviously it is symmetric.
   *   - Because the last 3 parameters are linear, the lower right 3x3 sub matrix
   *     is diagonal and all the diagonal element equals to the number of data points.
   *   - For the same reason above, the upper right 3x3 sub matrix
	 */
	bool optimizeAlt(const CvMat* A, const CvMat* B,
			double param[]);

	/*** A member object of Levenberg Marqardt in its general form */
	LevMarq mLevMarq;
	double mRTData[16]; ///< Data part of LevMarqTransform::mRT
	CvMat mRT;  ///< A transient buffer for transformation matrix. @warning Do not assume it is updated.
	CvMat mRT3x4; ///< A transient view for the upper 3x4 transformation matrix. @warning Do not assume it is updated.

	/// Buffer for transformation matrix of current param plus a delta vector
	/// use in Jacobian approximation
	double mFwdTData[numParams][16];
	/// Transformation matrices with respect to an delta change on each parameters.
	/// Used for computation of partial differentials.
	CvMat mFwdT[numParams];
	/// Upper 3x4 views of LevMarqTransform::mFwdT
	CvMat mFwdT3x4[numParams];
};
}
}
#endif /*CVLEVMARQ3D_H_*/
