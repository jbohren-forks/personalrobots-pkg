/*
 * PoseEstErrMeas.cpp
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#include "CvPoseEstErrMeas.h"
#include <opencv/cxcore.h>

CvPoseEstErrMeas::CvPoseEstErrMeas() :
	mRotation(cvMat(3, 3, CV_64FC1, _mRotData)),
	mShift(cvMat(3, 1, CV_64FC1,    _mShiftData))
{
	// TODO Auto-generated constructor stub

}

CvPoseEstErrMeas::~CvPoseEstErrMeas() {
	// TODO Auto-generated destructor stub
}

void CvPoseEstErrMeas::transform(const CvMat& src, CvMat& dst) {
	CvMat xyzs0Reshaped;
	CvMat xyzs1Reshaped;
	cvReshape(&src, &xyzs0Reshaped,  3, 0);
	cvReshape(&dst, &xyzs1Reshaped,  3, 0);
	cvTransform(&xyzs0Reshaped, &xyzs1Reshaped, &mRotation, &mShift);
}

bool CvPoseEstErrMeas::setTransform(const CvMat& rot, const CvMat& shift){
	int status = true;
	cvCopy(&rot, &mRotation);
	cvCopy(&shift, &mShift);
	return status;
}

void CvPoseEstErrMeas::measure(const CvMat& xyzs0, const CvMat& xyzs1){
	int n = xyzs0.rows;

	double _xyzs11[3*n];
	CvMat xyzs11 = cvMat(n, 3, CV_64F, _xyzs11);

	CvMat xyzs0Reshaped;
	CvMat xyzs1Reshaped;
	cvReshape(&xyzs0, &xyzs0Reshaped, 3, 0);
	cvReshape(&xyzs11, &xyzs1Reshaped, 3, 0);
	cvTransform(&xyzs0Reshaped, &xyzs1Reshaped, &mRotation, &mShift);

	// now compute the diff

}


