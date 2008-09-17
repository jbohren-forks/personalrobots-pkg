#include <opencv/cxtypes.h>
/*
 * CvPoseEstErrMeas.h
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#ifndef CVPOSEESTERRMEAS_H_
#define CVPOSEESTERRMEAS_H_

/**
 * Error measurement of pose estimation, in Cartesian space.
 * Used for debugging and analysis of pose estimation tools.
 */
class CvPoseEstErrMeas {
public:
	CvPoseEstErrMeas();
	virtual ~CvPoseEstErrMeas();

	/** Set up the transformations */
	bool setTransform(const CvMat& rot, const CvMat& shift);
	/** Perform transformation */
	void transform(const CvMat &src, CvMat &dst);
	/** Measure errors */
	void measure(const CvMat& xyzs0,  const CvMat& xyzs1);
	/** Compare two point clouds for error analysis */
	void compare(const CvMat& xyzs11, const CvMat& xyzs1);

	CvMat mRotation;
	CvMat mShift;

	double mErrL1Norm;
	double mErrL2Norm;
	double mErrInfNorm;

	double mErrL1NormXY;
	double mErrL1NormZ;
	double mErrL2NormXY;
	double mErrL2NormZ;
	double mErrInfNormXY;
	double mErrInfNormZ;


protected:
	double _mRotData[9];
	double _mShiftData[3];
};

#endif /* CVPOSEESTERRMEAS_H_ */
