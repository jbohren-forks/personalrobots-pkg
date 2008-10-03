/*
 * CvPoseEstErrMeasDisp.h
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#ifndef CVPOSEESTERRMEASDISP_H_
#define CVPOSEESTERRMEASDISP_H_

#include "CvPoseEstErrMeas.h"
#include "CvStereoCamModel.h"

/**
 * Error measurement of pose estimation, in disparity space.
 */
class CvPoseEstErrMeasDisp: public CvPoseEstErrMeas, public CvStereoCamModel {
public:
	CvPoseEstErrMeasDisp();
	virtual ~CvPoseEstErrMeasDisp();
	void transform(const CvMat& src, CvMat& dst);
	void measure(const CvMat& uvds0, const CvMat& uvds1);
	void compare(const CvMat& uvds0, const CvMat& uvds1);
	void measureMixed(const CvMat& xyzs0, const CvMat& uvds1);

	CvMat mHomography;
protected:
	double _mHomography[16];
};

#endif /* CVPOSEESTERRMEASDISP_H_ */
