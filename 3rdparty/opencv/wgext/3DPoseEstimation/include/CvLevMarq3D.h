#ifndef CVLEVMARQ3D_H_
#define CVLEVMARQ3D_H_

#include "calib_stereo.h"
#include "CvMatUtils.h"

class CvLevMarqTransform
{
public:
	/**
	 * Constructor
	 * numErrors:  num of errors to minimize over. Used for buffer allocation if the Jacobian matrix is
	 * use directly (CvLevMarq.update() is used for optimization instead of JtJ and JtErr.
	 * If 0 is set, JtJ and JtErr are used CvLevMarq.updateAlt() is used instead of CvLevMarq.update().
	 * For now please set it to zero, the other option is not fully test/implemented yet.
	 * numMaxIter: maximum number of iterations in LevMar optimization
	 */
	CvLevMarqTransform(int numErrors=0, int numMaxIter = defNumMaxIter);
	virtual ~CvLevMarqTransform();
	// There is 6 parameters for camera pose estimation. The first 3 are for rotation
	// (either euler or rodrigues) and last 3 are shift vector
	// For the case of Cartesian coordinates, the last 3 (shift vector) parameters are linearly related to
	// the error function, which give rise to sparsity in the matrix, which can be exploited for speeding-up.
	const static int numNonLinearParams = 3;   // the num of nonlinear parameters, namely rotation related
	const static int numParams = 6;	      // total num of parameters
	const static int defNumMaxIter = 50;  // maximum num of iterations
	const static int defMaxTimesOfUpdates = 300; //maximum num of times update() is called

	/**
	 *  A  - data point list A, one point each row
	 *  B  - data point list B, one point each row
	 *  param - pose parameters, e.g. [alpha, beta, gamma, x, y, z] if euler angle is used
	 *        - input as initial parameters, output as optimized parameters
	 */
	bool optimize(const CvMat* A, const CvMat* B, double param[]);
	/**
	 *  A  - data point list A, one point each row
	 *  B  - data point list B, one point each row
	 *  rot and trans - rotation and shift matrices
	 *        - input as initial parameters, output as optimized parameters
	 */
	bool optimize(const CvMat* A, const CvMat* B, CvMat *rot, CvMat* trans);

	typedef enum {
		Euler, Rodrigues
	} AngleType;
	AngleType mAngleType;

protected:
	bool constructRTMatrix(const CvMat* param);
	bool constructRTMatrices(const CvMat* param, CvMyReal delta);
	bool constructRTMatrix(const CvMat * param, CvMyReal _RT[]);
	bool computeForwardResidues(const CvMat *xyzs0, const CvMat *xyzs1, CvMat *res);
	virtual bool constructTransformationMatrix(const CvMat *param);
	virtual bool constructTransformationMatrix(const CvMat *param, CvMyReal T[]);
	virtual bool constructTransformationMatrices(const CvMat *param, CvMyReal delta);
	virtual bool computeResidue(const CvMat *xyzs0, const CvMat *xyzs1, CvMat* res);
	/**
	 * compute the residue error of T*xyzs0 - xyzs1. T is a 4x3 transformation matrix
	 */
	virtual bool computeResidue(const CvMat *xyzs0, const CvMat *xyzs1, const CvMat* T, CvMat* res);
	/**
	 *  use CvLevMarq.updateAlt() to do optimization
	 */
	bool optimizeAlt(const CvMat* A, const CvMat* B,
			double param[]);
	/**
	 * use CvLevMarq.update() to do optimization
	 */
	bool optimizeDefault(const CvMat* A, const CvMat* B, double param[]);

	const bool mUseUpdateAlt;
	CvLevMarq_JDC mLevMarq;
	CvMyReal mRTData[16];
	CvMat mRT;  // this is a transient buffer. Do not assume it is updated
	CvMat mRT3x4;

	// buffer for transformation matrix of current param plus a delta vector
	// use in Jacobian approximation
	CvMyReal mFwdTData[numParams][16];
	CvMat mFwdT[numParams];
	// a view of mFwdT of the upper 3x4 of the matrix
	CvMat mFwdT3x4[numParams];
};

#endif /*CVLEVMARQ3D_H_*/
