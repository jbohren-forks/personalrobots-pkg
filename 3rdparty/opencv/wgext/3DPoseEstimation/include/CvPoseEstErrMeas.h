#include <opencv/cxtypes.h>
/*
 * CvPoseEstErrMeas.h
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#ifndef CVPOSEESTERRMEAS_H_
#define CVPOSEESTERRMEAS_H_

class CvPoseEstErrMeas {
public:
	CvPoseEstErrMeas();
	virtual ~CvPoseEstErrMeas();

	bool setTransform(const CvMat& rot, const CvMat& shift);
	void transform(const CvMat &src, CvMat &dst);
	void measure(const CvMat& xyzs0,  const CvMat& xyzs1);
	void compare(const CvMat& xyzs11, const CvMat& xyzs1);

	CvMat mRotation;
	CvMat mShift;

protected:
	double _mRotData[9];
	double _mShiftData[3];
};

#endif /* CVPOSEESTERRMEAS_H_ */
